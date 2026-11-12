//! DEMONSTRATION ONLY. The first run of the MAVLink device layer on the MARV V2 board, over the
//! USB byte pipe of `usb-cdc`, against the ground station (`MARV-GCS/tools/board_check.sh`).
//! There is no sensor driver and no flight logic: every value is computed from the board's clock
//! with the shapes of the simulator in `MARV-GCS/crates/gcs-sim/src/signals.rs`, and the parameters
//! live in RAM and are lost at reset. It runs until `tools/run.sh` ends it.
//!
//! USB identity: VID 0xc0de, PID 0xcafe, "MARV", "MARV V2 link demo", "marv-link-demo".
//! Nothing is logged after the start-up lines: a defmt line masks interrupts for about 80 us.
#![no_std]
#![no_main]

use core::mem::{size_of, size_of_val};

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

/// The start gate of `docs/link-contract.md` (MARV-GCS): after each link-up nothing is written until
/// the computer has finished opening the port, i.e. a ground-station HEARTBEAT has arrived or DTR
/// has read true at every check for 100 ms (checks at most 5 ms apart). Bytes go to `Node::receive`
/// meanwhile; their replies wait for `Node::run`. Returns false when a read ends (link down).
async fn start_gate(
    rx: &mut usb_cdc::Rx<'static, UsbDriver>,
    link: &usb_cdc::Link<'_>,
    node: &mut Node<'_>,
    params: &mut Params,
) -> bool {
    use embassy_futures::select::{Either, select};
    use embassy_time::{Duration, Timer};
    use embedded_io_async::Read;

    let begun = Instant::now();
    let mut dtr_since: Option<Instant> = None;
    let mut buf = [0u8; 64];
    loop {
        match select(rx.read(&mut buf), Timer::after(Duration::from_millis(5))).await {
            Either::First(Ok(n)) if n > 0 => {
                if let Some(bytes) = buf.get(..n) {
                    node.receive(bytes, now_us(), params);
                }
            }
            Either::First(_) => return false,
            Either::Second(()) => {}
        }
        let now = Instant::now();
        let gcs = node.gcs_present(now.as_micros());
        if link.dtr() {
            dtr_since.get_or_insert(now);
        } else {
            dtr_since = None;
        }
        let dtr_held = dtr_since.is_some_and(|t| (now - t) >= Duration::from_millis(100));
        if gcs || dtr_held {
            defmt::info!(
                "[INFO] mavlink_demo: gate open by {=str} after {=u64} ms, dtr {=bool}",
                if gcs { "HEARTBEAT" } else { "DTR" },
                (now - begun).as_millis(),
                link.dtr()
            );
            return true;
        }
    }
}

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

    let driver = Driver::new(p.USB, Irqs);
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([]),
        CONTROL_BUF.init([0; 64]),
    );
    let cdc = UsbCdc::new(
        &mut builder,
        STATE.init(usb_cdc::State::new()),
        usb_cdc::Config::new(),
    );
    let (mut tx, mut rx, link) = cdc.split();
    let device = builder.build();
    spawner.spawn(usb_task(device).unwrap());

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

    defmt::info!(
        "[INFO] mavlink_demo: size_of Node {=usize} bytes",
        size_of::<Node<'static>>()
    );
    defmt::info!(
        "[INFO] mavlink_demo: size_of Params {=usize}, Signals {=usize}",
        size_of::<Params>(),
        size_of::<Signals>()
    );

    defmt::info!(
        "[INFO] mavlink_demo: size_of run future {=usize} bytes",
        size_of_val(&node.run(
            &mut rx,
            &mut tx,
            &mut delay,
            now_us,
            &mut params,
            &mut signals
        ))
    );

    loop {
        link.wait_up().await;
        if !start_gate(&mut rx, &link, &mut node, &mut params).await {
            node.on_link_down();
            continue;
        }
        let _end = node
            .run(
                &mut rx,
                &mut tx,
                &mut delay,
                now_us,
                &mut params,
                &mut signals,
            )
            .await;
        node.on_link_down();
    }
}
