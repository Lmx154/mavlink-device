//! DIAGNOSTIC SIBLING of `mavlink_demo`: the same program (same USB identity, streams, parameters
//! and start-up text), plus a recorder that keeps in RAM, with the time of each, every `Handler`
//! call of the USB device, every class control request (line coding, control lines), every frame
//! the node wrote (time, length, sequence number), the first 32 KiB of written bytes verbatim,
//! every read from the pipe with its bytes, and each return of `Node::run` with the node's
//! counters. Nothing is logged while the computer has the port open: the recorder is printed
//! (`[STAT] DIAG ...` lines) once DTR has been low for 1.5 s after having been high.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};

use embassy_time::Timer;
use embedded_io_async::{ErrorType, Read, Write};
use mavlink_device::RunError;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Delay, Instant};
use embassy_usb::{Builder, Config, UsbDevice};
use mavlink_device::dialect::{
    GPS_RAW_INT_DATA, GpsFixType, HIGHRES_IMU_DATA, HighresImuUpdatedFlags, MavSeverity, MavState,
    SCALED_PRESSURE_DATA,
};
use mavlink_device::{
    Config as NodeConfig, Node, Outgoing, ParamError, ParamName, ParamStore, ParamValue, Stream,
    StreamSlot, Telemetry, Texts,
};
use panic_probe as _;
use static_cell::StaticCell;
use usb_cdc::UsbCdc;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type UsbDriver = Driver<'static, USB>;

const HIGHRES_IMU_ID: u32 = 105;
const SCALED_PRESSURE_ID: u32 = 29;
const GPS_RAW_INT_ID: u32 = 24;

fn now_us() -> u64 {
    Instant::now().as_micros()
}

// ---------------------------------------------------------------------------------------------
// Signals: the shapes of gcs-sim, with a rational sine (no libm).
// ---------------------------------------------------------------------------------------------

/// `sin(2 pi t)` for `t` in turns, `0.0..1.0` (Bhaskara's approximation, error below 0.002).
fn sin_turns(t: f32) -> f32 {
    let (x, sign) = if t < 0.5 { (t, 1.0) } else { (t - 0.5, -1.0) };
    let p = x * (0.5 - x);
    sign * 16.0 * p / (1.25 - 4.0 * p)
}

/// `amplitude * sin(2 pi freq t + phase)` with the time in ms, the frequency in mHz and the phase
/// in radians. The turns are kept in integers so a long uptime loses no precision.
fn sine(now_ms: u64, amplitude: f32, freq_mhz: u64, phase_rad: f32) -> f32 {
    let turns_ppm = (freq_mhz * now_ms) % 1_000_000;
    let phase_ppm = ((phase_rad / core::f32::consts::TAU) * 1_000_000.0) as i64;
    let ppm = (turns_ppm as i64 + phase_ppm).rem_euclid(1_000_000);
    amplitude * sin_turns(ppm as f32 / 1_000_000.0)
}

const GRAVITY: f32 = 9.81;
const HIGH_G_GRAVITY: f32 = 9.8;
const MAG_GAUSS: [f32; 3] = [0.21, -0.03, 0.42];
const TEMPERATURE_DEGC: f32 = 25.0;

fn imu_motion(now_ms: u64, phase: f32) -> [f32; 6] {
    [
        sine(now_ms, 0.3, 310, phase),
        sine(now_ms, 0.25, 470, 1.0 + phase),
        -GRAVITY + sine(now_ms, 0.2, 230, 2.0 + phase),
        sine(now_ms, 0.005, 200, 0.5 + phase),
        sine(now_ms, 0.004, 350, 1.5 + phase),
        sine(now_ms, 0.003, 150, 2.5 + phase),
    ]
}

/// Deterministic noise in [-0.05, 0.05) m/s^2.
fn high_g_noise(now_ms: u64, axis: u64) -> f32 {
    let mut x =
        now_ms.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ (axis + 1).wrapping_mul(0xD1B5_4A32_D192_ED03);
    x ^= x >> 32;
    x = x.wrapping_mul(0xD6E8_FEB8_6659_FD93);
    x ^= x >> 32;
    ((x >> 40) as f32 / 16_777_216.0 - 0.5) * 0.1
}

fn highres_imu(id: u8, now_us: u64) -> HIGHRES_IMU_DATA {
    use HighresImuUpdatedFlags as F;
    let now_ms = now_us / 1000;
    let nan = f32::NAN;
    let blank = HIGHRES_IMU_DATA {
        time_usec: now_us,
        xacc: nan,
        yacc: nan,
        zacc: nan,
        xgyro: nan,
        ygyro: nan,
        zgyro: nan,
        xmag: nan,
        ymag: nan,
        zmag: nan,
        abs_pressure: nan,
        diff_pressure: nan,
        pressure_alt: nan,
        temperature: nan,
        fields_updated: F::empty(),
        id,
    };
    let imu = |phase: f32| {
        let m = imu_motion(now_ms, phase);
        HIGHRES_IMU_DATA {
            xacc: m[0],
            yacc: m[1],
            zacc: m[2],
            xgyro: m[3],
            ygyro: m[4],
            zgyro: m[5],
            temperature: TEMPERATURE_DEGC,
            fields_updated: F::HIGHRES_IMU_UPDATED_XACC
                | F::HIGHRES_IMU_UPDATED_YACC
                | F::HIGHRES_IMU_UPDATED_ZACC
                | F::HIGHRES_IMU_UPDATED_XGYRO
                | F::HIGHRES_IMU_UPDATED_YGYRO
                | F::HIGHRES_IMU_UPDATED_ZGYRO
                | F::HIGHRES_IMU_UPDATED_TEMPERATURE,
            ..blank.clone()
        }
    };
    match id {
        0 => imu(0.0),
        1 => imu(1.3),
        2 => HIGHRES_IMU_DATA {
            xacc: sine(now_ms, 0.5, 1700, 0.0) + high_g_noise(now_ms, 0),
            yacc: sine(now_ms, 0.5, 2300, 1.0) + high_g_noise(now_ms, 1),
            zacc: -HIGH_G_GRAVITY + sine(now_ms, 0.5, 1100, 2.0) + high_g_noise(now_ms, 2),
            fields_updated: F::HIGHRES_IMU_UPDATED_XACC
                | F::HIGHRES_IMU_UPDATED_YACC
                | F::HIGHRES_IMU_UPDATED_ZACC,
            ..blank
        },
        _ => HIGHRES_IMU_DATA {
            xmag: MAG_GAUSS[0],
            ymag: MAG_GAUSS[1],
            zmag: MAG_GAUSS[2],
            temperature: TEMPERATURE_DEGC,
            fields_updated: F::HIGHRES_IMU_UPDATED_XMAG
                | F::HIGHRES_IMU_UPDATED_YMAG
                | F::HIGHRES_IMU_UPDATED_ZMAG
                | F::HIGHRES_IMU_UPDATED_TEMPERATURE,
            ..blank
        },
    }
}

struct Signals {
    text_offered: bool,
}

impl Telemetry for Signals {
    fn status(&mut self, now_us: u64) -> Option<MavState> {
        Some(if now_us < BOOT_US {
            MavState::MAV_STATE_BOOT
        } else {
            MavState::MAV_STATE_STANDBY
        })
    }

    fn texts(&mut self, _now_us: u64, out: &mut Texts<'_>) {
        if !self.text_offered {
            self.text_offered = out
                .push(MavSeverity::MAV_SEVERITY_INFO, "MARV link demo ready")
                .is_ok();
            rec::event(12, self.text_offered as u32, 0, 0);
        }
    }

    fn sample(&mut self, message_id: u32, now_us: u64, out: &mut Outgoing<'_>) -> bool {
        let now_ms = now_us / 1000;
        match message_id {
            HIGHRES_IMU_ID => {
                let mut ok = true;
                for id in 0..4u8 {
                    ok &= out.push(&highres_imu(id, now_us)).is_ok();
                }
                ok
            }
            SCALED_PRESSURE_ID => out
                .push(&SCALED_PRESSURE_DATA {
                    time_boot_ms: now_ms as u32,
                    press_abs: 1013.25 + sine(now_ms, 0.5, 100, 0.0),
                    press_diff: 0.0,
                    temperature: 2500,
                    temperature_press_diff: 0,
                })
                .is_ok(),
            GPS_RAW_INT_ID => out
                .push(&GPS_RAW_INT_DATA {
                    time_usec: now_us,
                    lat: 473_977_000,
                    lon: 85_456_000,
                    alt: 488_000,
                    eph: 100,
                    epv: 150,
                    vel: 0,
                    cog: 0,
                    fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
                    satellites_visible: 12,
                    alt_ellipsoid: 488_000,
                    h_acc: 1500,
                    v_acc: 2500,
                    vel_acc: 200,
                    hdg_acc: 0,
                    yaw: 0,
                })
                .is_ok(),
            _ => false,
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Parameters: thirteen, in RAM, the same names, types and values as the simulator's defaults.
// ---------------------------------------------------------------------------------------------

struct Param {
    name: &'static str,
    value: ParamValue,
    read_only: bool,
}

const fn p(name: &'static str, value: ParamValue, read_only: bool) -> Param {
    Param {
        name,
        value,
        read_only,
    }
}

const LOG_RATE_HZ: &str = "LOG_RATE_HZ";

struct Params {
    list: [Param; 13],
}

impl Params {
    const fn new() -> Self {
        Self {
            list: [
                p("IMU_RATE_HZ", ParamValue::I32(800), false),
                p("IMU_ACC_RANGE", ParamValue::I32(16), false),
                p("IMU_GYR_LPF_HZ", ParamValue::F32(98.0), false),
                p("HIGHG_RANGE", ParamValue::I32(320), false),
                p("BARO_RATE_HZ", ParamValue::I32(50), false),
                p("GPS_RATE_HZ", ParamValue::I32(10), false),
                p("LOG_ENABLE", ParamValue::I32(1), false),
                p(LOG_RATE_HZ, ParamValue::I32(100), false),
                p("BAT_V_SCALE", ParamValue::F32(11.0), false),
                p("BAT_LOW_V", ParamValue::F32(14.0), false),
                p("SYS_TRIM", ParamValue::I32(-3), false),
                p("SYS_UID", ParamValue::I32(1_592_460_852), true),
                p("NAV_TEST_16CHARS", ParamValue::F32(1.5), false),
            ],
        }
    }
}

impl ParamStore for Params {
    fn count(&self) -> u16 {
        self.list.len() as u16
    }

    fn name(&self, i: u16) -> Option<ParamName> {
        ParamName::new(self.list.get(usize::from(i))?.name).ok()
    }

    fn get(&self, i: u16) -> Option<ParamValue> {
        Some(self.list.get(usize::from(i))?.value)
    }

    fn find(&self, n: &ParamName) -> Option<u16> {
        let i = self.list.iter().position(|p| p.name == n.as_str())?;
        u16::try_from(i).ok()
    }

    fn set(&mut self, i: u16, v: ParamValue) -> Result<ParamValue, ParamError> {
        let param = self
            .list
            .get_mut(usize::from(i))
            .ok_or(ParamError::NotFound)?;
        if param.read_only {
            return Err(ParamError::Rejected);
        }
        let v = match (param.name == LOG_RATE_HZ, v) {
            (true, ParamValue::I32(n)) => ParamValue::I32(n.clamp(1, 1000)),
            _ => v,
        };
        param.value = v;
        Ok(v)
    }
}

// ---------------------------------------------------------------------------------------------

/// The first second after start is BOOT.
const BOOT_US: u64 = 1_000_000;

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("MARV");
    config.product = Some("MARV V2 link demo");
    config.serial_number = Some("marv-link-demo");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 0]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<usb_cdc::State<'static>> = StaticCell::new();
    static SPY: StaticCell<Spy> = StaticCell::new();

    let driver = Driver::new(p.USB, Irqs);
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([]),
        CONTROL_BUF.init([0; 64]),
    );
    builder.handler(SPY.init(Spy));
    let cdc = UsbCdc::new(
        &mut builder,
        STATE.init(usb_cdc::State::new()),
        usb_cdc::Config::new(),
    );
    let (tx, rx, link) = cdc.split();
    let (mut tx, mut rx) = (RecTx(tx), RecRx(rx));
    let device = builder.build();
    spawner.spawn(usb_task(device).unwrap());
    spawner.spawn(reporter().unwrap());

    let mut slots = [
        StreamSlot::new(Stream {
            message_id: HIGHRES_IMU_ID,
            default_interval_us: 20_000,
            min_interval_us: 5_000,
        }),
        StreamSlot::new(Stream {
            message_id: SCALED_PRESSURE_ID,
            default_interval_us: 100_000,
            min_interval_us: 20_000,
        }),
        StreamSlot::new(Stream {
            message_id: GPS_RAW_INT_ID,
            default_interval_us: 200_000,
            min_interval_us: 100_000,
        }),
    ];
    let mut node = Node::new(NodeConfig::new(1, 1), &mut slots);
    let mut params = Params::new();
    let mut signals = Signals {
        text_offered: false,
    };
    let mut delay = Delay;

    defmt::info!("[INFO] mavlink_diag: started, recorder ready");

    loop {
        link.wait_up().await;
        rec::event(8, 0, 0, 0);
        let end = node
            .run(
                &mut rx,
                &mut tx,
                &mut delay,
                now_us,
                &mut params,
                &mut signals,
            )
            .await;
        let s = *node.stats();
        let reason = match end {
            RunError::Read(_) => 0,
            RunError::Write(_) => 1,
            RunError::EndOfStream => 2,
        };
        rec::event(9, reason, s.garbage_bytes, s.dropped_checksum);
        rec::event(
            10,
            s.dropped_own_system,
            s.heartbeats_received,
            s.frames_sent,
        );
        let l = link.stats();
        rec::event(11, l.ups, l.downs, l.resets);
        node.on_link_down();
    }
}

// ---------------------------------------------------------------------------------------------
// The recorder.
// ---------------------------------------------------------------------------------------------

mod rec {
    use core::cell::RefCell;
    use critical_section::Mutex;

    pub const TXB: usize = 32 * 1024;
    pub const FRAMES: usize = 12_000;
    pub const RXB: usize = 16 * 1024;
    pub const READS: usize = 4096;
    pub const EVENTS: usize = 1024;

    /// One `write_all` of the node: start and end time, length (bit 15: the write failed),
    /// sequence number and message id (low byte).
    #[derive(Clone, Copy)]
    pub struct Frame {
        pub t0: u32,
        pub t1: u32,
        pub len: u16,
        pub seq: u8,
        pub msg: u8,
    }

    #[derive(Clone, Copy)]
    pub struct Rd {
        pub t: u32,
        pub len: u32,
    }

    #[derive(Clone, Copy)]
    pub struct Ev {
        pub t: u32,
        pub kind: u32,
        pub a: u32,
        pub b: u32,
        pub c: u32,
    }

    pub struct Rec {
        pub tx: [u8; TXB],
        pub tx_n: usize,
        pub frames: [Frame; FRAMES],
        pub frames_n: usize,
        pub rx: [u8; RXB],
        pub rx_n: usize,
        pub reads: [Rd; READS],
        pub reads_n: usize,
        pub ev: [Ev; EVENTS],
        pub ev_n: usize,
        pub overflow: u32,
    }

    pub static REC: Mutex<RefCell<Rec>> = Mutex::new(RefCell::new(Rec {
        tx: [0; TXB],
        tx_n: 0,
        frames: [Frame {
            t0: 0,
            t1: 0,
            len: 0,
            seq: 0,
            msg: 0,
        }; FRAMES],
        frames_n: 0,
        rx: [0; RXB],
        rx_n: 0,
        reads: [Rd { t: 0, len: 0 }; READS],
        reads_n: 0,
        ev: [Ev {
            t: 0,
            kind: 0,
            a: 0,
            b: 0,
            c: 0,
        }; EVENTS],
        ev_n: 0,
        overflow: 0,
    }));

    pub fn now() -> u32 {
        embassy_time::Instant::now().as_micros() as u32
    }

    pub fn event(kind: u32, a: u32, b: u32, c: u32) {
        let t = now();
        critical_section::with(|cs| {
            let mut r = REC.borrow_ref_mut(cs);
            let i = r.ev_n;
            if i < EVENTS {
                r.ev[i] = Ev { t, kind, a, b, c };
                r.ev_n += 1;
            } else {
                r.overflow += 1;
            }
        });
    }

    pub fn frame(t0: u32, t1: u32, bytes: &[u8], ok: bool) {
        critical_section::with(|cs| {
            let mut r = REC.borrow_ref_mut(cs);
            let i = r.frames_n;
            if i < FRAMES {
                r.frames[i] = Frame {
                    t0,
                    t1,
                    len: (bytes.len() as u16) | if ok { 0 } else { 0x8000 },
                    seq: bytes.get(4).copied().unwrap_or(0),
                    msg: bytes.get(7).copied().unwrap_or(0),
                };
                r.frames_n += 1;
            } else {
                r.overflow += 1;
            }
            let n = r.tx_n;
            let room = TXB - n;
            let take = bytes.len().min(room);
            r.tx[n..n + take].copy_from_slice(&bytes[..take]);
            r.tx_n += take;
        });
    }

    pub fn read(bytes: &[u8]) {
        let t = now();
        critical_section::with(|cs| {
            let mut r = REC.borrow_ref_mut(cs);
            let i = r.reads_n;
            if i < READS {
                r.reads[i] = Rd {
                    t,
                    len: bytes.len() as u32,
                };
                r.reads_n += 1;
            } else {
                r.overflow += 1;
            }
            let n = r.rx_n;
            let take = bytes.len().min(RXB - n);
            r.rx[n..n + take].copy_from_slice(&bytes[..take]);
            r.rx_n += take;
        });
    }
}

/// DTR as the computer last set it, when it last changed, and whether it was high since the last dump.
static DTR: AtomicBool = AtomicBool::new(false);
static DTR_CHANGED: AtomicU32 = AtomicU32::new(0);
static DTR_SEEN: AtomicBool = AtomicBool::new(false);

/// A second `Handler`, registered before the CDC class: it only stores and never answers.
struct Spy;

impl embassy_usb::Handler for Spy {
    fn enabled(&mut self, enabled: bool) {
        rec::event(1, enabled as u32, 0, 0);
    }
    fn reset(&mut self) {
        rec::event(2, 0, 0, 0);
        DTR.store(false, Relaxed);
        DTR_CHANGED.store(rec::now(), Relaxed);
    }
    fn addressed(&mut self, addr: u8) {
        rec::event(3, u32::from(addr), 0, 0);
    }
    fn configured(&mut self, configured: bool) {
        rec::event(4, configured as u32, 0, 0);
    }
    fn suspended(&mut self, suspended: bool) {
        rec::event(5, suspended as u32, 0, 0);
    }
    fn control_out(
        &mut self,
        req: embassy_usb::control::Request,
        data: &[u8],
    ) -> Option<embassy_usb::control::OutResponse> {
        if req.request_type == embassy_usb::control::RequestType::Class {
            let word = |i: usize| data.get(i).copied().map_or(0, u32::from);
            let b = word(0) | word(1) << 8 | word(2) << 16 | word(3) << 24;
            let c = word(4) | word(5) << 8 | word(6) << 16 | (data.len() as u32) << 24;
            rec::event(
                6,
                u32::from(req.request)
                    | u32::from(req.value) << 8
                    | (u32::from(req.index) & 0xff) << 24,
                b,
                c,
            );
            if req.request == 0x22 {
                let dtr = req.value & 1 != 0;
                DTR.store(dtr, Relaxed);
                DTR_CHANGED.store(rec::now(), Relaxed);
                if dtr {
                    DTR_SEEN.store(true, Relaxed);
                }
            }
        }
        None
    }
    fn control_in<'a>(
        &'a mut self,
        req: embassy_usb::control::Request,
        _buf: &'a mut [u8],
    ) -> Option<embassy_usb::control::InResponse<'a>> {
        if req.request_type == embassy_usb::control::RequestType::Class {
            rec::event(7, u32::from(req.request) | u32::from(req.value) << 8, 0, 0);
        }
        None
    }
}

/// The pipe's write half, recording each frame (`write_all`) after it was handed over.
struct RecTx<T>(T);

impl<T: ErrorType> ErrorType for RecTx<T> {
    type Error = T::Error;
}

impl<T: Write> Write for RecTx<T> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, T::Error> {
        self.0.write(buf).await
    }
    async fn flush(&mut self) -> Result<(), T::Error> {
        self.0.flush().await
    }
    async fn write_all(&mut self, buf: &[u8]) -> Result<(), T::Error> {
        let t0 = rec::now();
        let r = self.0.write_all(buf).await;
        rec::frame(t0, rec::now(), buf, r.is_ok());
        r
    }
}

/// The pipe's read half, recording each read in the poll it completes in (still cancel-safe).
struct RecRx<R>(R);

impl<R: ErrorType> ErrorType for RecRx<R> {
    type Error = R::Error;
}

impl<R: Read> Read for RecRx<R> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, R::Error> {
        let r = self.0.read(buf).await;
        if let Ok(n) = r {
            rec::read(&buf[..n]);
        }
        r
    }
}

const HEX: &[u8; 16] = b"0123456789abcdef";

fn print_hex(section: &str, index: usize, bytes: &[u8]) {
    let mut out = [0u8; 2 * 240];
    for (i, b) in bytes.iter().take(240).enumerate() {
        out[2 * i] = HEX[usize::from(b >> 4)];
        out[2 * i + 1] = HEX[usize::from(b & 15)];
    }
    let text = core::str::from_utf8(&out[..2 * bytes.len().min(240)]).unwrap_or("");
    defmt::info!("[STAT] DIAG {=str} {=usize} {=str}", section, index, text);
}

#[derive(Default)]
struct Cursor {
    ev: usize,
    fr: usize,
    tx: usize,
    rd: usize,
    rx: usize,
}

/// Prints everything recorded since the last dump.
fn dump(cur: &mut Cursor) {
    let (ev_n, fr_n, tx_n, rd_n, rx_n, overflow) = critical_section::with(|cs| {
        let r = rec::REC.borrow_ref(cs);
        (r.ev_n, r.frames_n, r.tx_n, r.reads_n, r.rx_n, r.overflow)
    });
    defmt::info!(
        "[STAT] DIAG sum now={=u32} ev={=usize} fr={=usize} tx={=usize} rd={=usize} rx={=usize} overflow={=u32}",
        rec::now(),
        ev_n,
        fr_n,
        tx_n,
        rd_n,
        rx_n,
        overflow
    );
    let mut buf = [0u8; 240];
    while cur.ev < ev_n {
        let k = (ev_n - cur.ev).min(12);
        critical_section::with(|cs| {
            let r = rec::REC.borrow_ref(cs);
            for j in 0..k {
                let e = r.ev[cur.ev + j];
                for (w, v) in [e.t, e.kind, e.a, e.b, e.c].iter().enumerate() {
                    buf[20 * j + 4 * w..20 * j + 4 * w + 4].copy_from_slice(&v.to_le_bytes());
                }
            }
        });
        print_hex("ev", cur.ev, &buf[..20 * k]);
        cur.ev += k;
    }
    while cur.fr < fr_n {
        let k = (fr_n - cur.fr).min(20);
        critical_section::with(|cs| {
            let r = rec::REC.borrow_ref(cs);
            for j in 0..k {
                let f = r.frames[cur.fr + j];
                let o = 12 * j;
                buf[o..o + 4].copy_from_slice(&f.t0.to_le_bytes());
                buf[o + 4..o + 8].copy_from_slice(&f.t1.to_le_bytes());
                buf[o + 8..o + 10].copy_from_slice(&f.len.to_le_bytes());
                buf[o + 10] = f.seq;
                buf[o + 11] = f.msg;
            }
        });
        print_hex("fr", cur.fr, &buf[..12 * k]);
        cur.fr += k;
    }
    while cur.tx < tx_n {
        let k = (tx_n - cur.tx).min(240);
        critical_section::with(|cs| {
            let r = rec::REC.borrow_ref(cs);
            buf[..k].copy_from_slice(&r.tx[cur.tx..cur.tx + k]);
        });
        print_hex("tx", cur.tx, &buf[..k]);
        cur.tx += k;
    }
    while cur.rd < rd_n {
        let k = (rd_n - cur.rd).min(30);
        critical_section::with(|cs| {
            let r = rec::REC.borrow_ref(cs);
            for j in 0..k {
                let d = r.reads[cur.rd + j];
                buf[8 * j..8 * j + 4].copy_from_slice(&d.t.to_le_bytes());
                buf[8 * j + 4..8 * j + 8].copy_from_slice(&d.len.to_le_bytes());
            }
        });
        print_hex("rd", cur.rd, &buf[..8 * k]);
        cur.rd += k;
    }
    while cur.rx < rx_n {
        let k = (rx_n - cur.rx).min(240);
        critical_section::with(|cs| {
            let r = rec::REC.borrow_ref(cs);
            buf[..k].copy_from_slice(&r.rx[cur.rx..cur.rx + k]);
        });
        print_hex("rx", cur.rx, &buf[..k]);
        cur.rx += k;
    }
    defmt::info!("[STAT] DIAG end");
}

#[embassy_executor::task]
async fn reporter() -> ! {
    let mut cur = Cursor::default();
    loop {
        Timer::after_millis(100).await;
        let quiet = rec::now().wrapping_sub(DTR_CHANGED.load(Relaxed));
        if DTR_SEEN.load(Relaxed) && !DTR.load(Relaxed) && quiet >= 1_500_000 {
            DTR_SEEN.store(false, Relaxed);
            rec::event(13, 0, 0, 0);
            dump(&mut cur);
        }
    }
}
