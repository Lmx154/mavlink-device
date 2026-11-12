//! Host tests. Every frame the node emits is decoded with the std `mavlink` crate's own reader.

use crate::dialect::{
    HEARTBEAT_DATA, HIGHRES_IMU_DATA, MavAutopilot, MavSeverity, MavState, MavType,
    SCALED_PRESSURE_DATA,
};
use crate::*;
use mavlink::dialects::common::MavMessage as Std;

pub(crate) struct NoParams;

impl ParamStore for NoParams {
    fn count(&self) -> u16 {
        0
    }
    fn name(&self, _: u16) -> Option<ParamName> {
        None
    }
    fn get(&self, _: u16) -> Option<ParamValue> {
        None
    }
    fn find(&self, _: &ParamName) -> Option<u16> {
        None
    }
    fn set(&mut self, _: u16, _: ParamValue) -> Result<ParamValue, ParamError> {
        Err(ParamError::NotFound)
    }
}

/// Telemetry that records its calls and pushes `count` messages for the ids it is told about.
#[derive(Default)]
pub(crate) struct Fake {
    pub calls: Vec<(u32, u64)>,
    /// (message id, frames to push, return value)
    pub plan: Vec<(u32, usize, bool)>,
    pub push_results: Vec<Result<(), Full>>,
}

impl Fake {
    pub fn new(plan: &[(u32, usize, bool)]) -> Self {
        Self {
            plan: plan.to_vec(),
            ..Self::default()
        }
    }
    pub fn count(&self, id: u32) -> usize {
        self.calls.iter().filter(|c| c.0 == id).count()
    }
}

impl Telemetry for Fake {
    fn sample(&mut self, message_id: u32, now_us: u64, out: &mut Outgoing<'_>) -> bool {
        self.calls.push((message_id, now_us));
        let Some(&(_, count, ret)) = self.plan.iter().find(|p| p.0 == message_id) else {
            return false;
        };
        for k in 0..count {
            let r = match message_id {
                105 => out.push(&HIGHRES_IMU_DATA {
                    id: k as u8,
                    time_usec: now_us,
                    ..Default::default()
                }),
                29 => out.push(&SCALED_PRESSURE_DATA {
                    time_boot_ms: (now_us / 1000) as u32,
                    ..Default::default()
                }),
                _ => out.push(&dialect::GPS_RAW_INT_DATA {
                    time_usec: now_us,
                    ..Default::default()
                }),
            };
            self.push_results.push(r);
        }
        ret
    }
}

/// Decodes one frame with the std reader; fails on a rejected frame or on trailing bytes.
pub(crate) fn decode(bytes: &[u8]) -> (mavlink::MavHeader, Std) {
    let mut reader = mavlink::MavlinkReader::new(bytes);
    let out = reader
        .read_message::<Std>(mavlink::MavlinkVersion::V2)
        .expect("the std mavlink reader rejected a frame of the node");
    assert!(
        reader
            .read_message::<Std>(mavlink::MavlinkVersion::V2)
            .is_err(),
        "bytes left after the frame"
    );
    (out.0, out.1)
}

pub(crate) fn pull(
    node: &mut Node<'_>,
    now: u64,
    tel: &mut Fake,
) -> Option<(mavlink::MavHeader, Std, Vec<u8>)> {
    let mut frame = [0u8; FRAME_BUF];
    let n = node.poll_transmit(now, &NoParams, tel, &mut frame)?;
    let bytes = frame[..n].to_vec();
    let (h, m) = decode(&bytes);
    Some((h, m, bytes))
}

fn drain(node: &mut Node<'_>, now: u64, tel: &mut Fake) -> Vec<(mavlink::MavHeader, Std)> {
    let mut v = Vec::new();
    while let Some((h, m, _)) = pull(node, now, tel) {
        v.push((h, m));
        assert!(v.len() < 1000);
    }
    v
}

fn slot(message_id: u32, default_interval_us: i32, min_interval_us: u32) -> StreamSlot {
    StreamSlot::new(Stream {
        message_id,
        default_interval_us,
        min_interval_us,
    })
}

fn rx(node: &mut Node<'_>, bytes: &[u8], now: u64) {
    node.receive(bytes, now, &mut NoParams);
}

/// A frame made by the std crate, MAVLink 2.
pub(crate) fn std_frame(system_id: u8, component_id: u8, seq: u8, m: &Std) -> Vec<u8> {
    let mut raw = mavlink::MAVLinkV2MessageRaw::new();
    raw.serialize_message(
        mavlink::MavHeader {
            system_id: u32::from(system_id),
            component_id,
            sequence: seq,
        },
        m,
    );
    raw.raw_bytes().to_vec()
}

pub(crate) fn hb_msg(ty: mavlink::dialects::common::MavType) -> Std {
    Std::HEARTBEAT(mavlink::dialects::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: ty,
        autopilot: mavlink::dialects::common::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: mavlink::dialects::common::MavModeFlag::empty(),
        system_status: mavlink::dialects::common::MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    })
}

pub(crate) fn gcs_hb(seq: u8) -> Vec<u8> {
    std_frame(
        255,
        190,
        seq,
        &hb_msg(mavlink::dialects::common::MavType::MAV_TYPE_GCS),
    )
}

fn hb_of(m: &Std) -> &mavlink::dialects::common::HEARTBEAT_DATA {
    match m {
        Std::HEARTBEAT(h) => h,
        other => panic!("not a heartbeat: {other:?}"),
    }
}

fn imu_id(m: &Std) -> u8 {
    match m {
        Std::HIGHRES_IMU(i) => i.id,
        other => panic!("not an imu: {other:?}"),
    }
}

fn node_with<'a>(slots: &'a mut [StreamSlot]) -> Node<'a> {
    Node::new(Config::new(1, 1), slots)
}

// ---- 1. framing out ----

#[test]
fn frames_are_mavlink2_with_flags_zero_and_the_configured_ids() {
    let mut slots = [];
    let mut node = Node::new(Config::new(7, 9), &mut slots);
    let (h, _, bytes) = pull(&mut node, 0, &mut Fake::default()).unwrap();
    assert_eq!((bytes[0], bytes[2], bytes[3]), (0xFD, 0, 0));
    assert_eq!((bytes[5], bytes[6]), (7, 9));
    assert_eq!((h.system_id, h.component_id), (7, 9));
}

#[test]
fn one_sequence_counter_for_every_frame_consecutive_modulo_256() {
    let mut slots = [slot(29, 10_000, 1_000), slot(105, 10_000, 1_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(29, 1, true), (105, 3, true)]);
    let mut seqs = Vec::new();
    let mut kinds = [false; 3];
    for step in 0..400u64 {
        if step.is_multiple_of(7) {
            let _ = node.queue_text(MavSeverity::MAV_SEVERITY_INFO, "hello");
        }
        while let Some((h, m, _)) = pull(&mut node, step * 10_000, &mut fake) {
            seqs.push(h.sequence);
            match m {
                Std::HEARTBEAT(_) => kinds[0] = true,
                Std::STATUSTEXT(_) => kinds[1] = true,
                _ => kinds[2] = true,
            }
        }
    }
    assert!(seqs.len() > 1500, "{}", seqs.len());
    assert_eq!(kinds, [true; 3]);
    assert_eq!(seqs[0], 0);
    for w in seqs.windows(2) {
        assert_eq!(w[1], w[0].wrapping_add(1));
    }
}

// ---- 2. framing in ----

#[test]
fn a_heartbeat_is_accepted_at_every_split_point_and_byte_by_byte() {
    let f = gcs_hb(3);
    for cut in 0..=f.len() {
        let mut slots = [];
        let mut node = node_with(&mut slots);
        rx(&mut node, &f[..cut], 10);
        rx(&mut node, &f[cut..], 10);
        assert_eq!(node.stats().heartbeats_received, 1, "cut {cut}");
        assert_eq!(node.stats().garbage_bytes, 0);
    }
    let mut slots = [];
    let mut node = node_with(&mut slots);
    for b in &f {
        rx(&mut node, &[*b], 10);
    }
    assert_eq!(node.stats().heartbeats_received, 1);
}

#[test]
fn garbage_is_discarded_and_counted() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    let mut bytes = vec![1, 2, 3, 0];
    bytes.extend(gcs_hb(0));
    bytes.extend([9, 9]);
    bytes.extend(gcs_hb(1));
    rx(&mut node, &bytes, 10);
    assert_eq!(node.stats().heartbeats_received, 2);
    assert_eq!(node.stats().garbage_bytes, 6);
}

#[test]
fn mavlink1_frames_are_dropped_and_counted_separately() {
    let mut raw = mavlink::MAVLinkV1MessageRaw::new();
    raw.serialize_message(
        mavlink::MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: 0,
        },
        &hb_msg(mavlink::dialects::common::MavType::MAV_TYPE_GCS),
    );
    let v1 = raw.raw_bytes().to_vec();
    let mut slots = [];
    let mut node = node_with(&mut slots);
    rx(&mut node, &v1, 10);
    let s = *node.stats();
    assert_eq!(s.dropped_mavlink1, 1);
    assert_eq!(s.heartbeats_received, 0);
    assert!(!node.gcs_present(10));
    rx(&mut node, &gcs_hb(1), 10);
    assert_eq!(node.stats().heartbeats_received, 1);
}

#[test]
fn frames_with_any_incompat_flag_are_dropped_and_counted_separately() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    for flags in [0x01u8, 0x02, 0x80, 0xFF] {
        let mut f = gcs_hb(0);
        f[2] = flags;
        let n = f.len();
        let crc = marv_dialect::calculate_crc(
            &f[1..n - 2],
            <HEARTBEAT_DATA as marv_dialect::MessageData>::EXTRA_CRC,
        );
        f[n - 2..].copy_from_slice(&crc.to_le_bytes());
        if flags & 1 != 0 {
            f.extend([0u8; 13]); // a signature
        }
        rx(&mut node, &f, 10);
    }
    let s = *node.stats();
    assert_eq!(s.dropped_incompat, 4);
    assert_eq!(s.heartbeats_received, 0);
    assert_eq!(s.dropped_checksum, 0);
    rx(&mut node, &gcs_hb(1), 10);
    assert_eq!(node.stats().heartbeats_received, 1);
}

#[test]
fn a_wrong_checksum_is_counted_separately() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    let mut a = gcs_hb(0);
    let n = a.len();
    a[n - 1] ^= 0x01;
    let mut b = gcs_hb(1);
    b[12] ^= 0x01;
    rx(&mut node, &a, 10);
    rx(&mut node, &b, 10);
    let s = *node.stats();
    assert_eq!(s.dropped_checksum, 2);
    assert_eq!(s.heartbeats_received, 0);
    assert_eq!(s.dropped_unknown_message, 0);
    assert!(!node.gcs_present(10));
}

#[test]
fn messages_the_node_does_not_handle_are_dropped_and_counted_separately() {
    let att = Std::ATTITUDE(mavlink::dialects::common::ATTITUDE_DATA::default());
    let mut slots = [];
    let mut node = node_with(&mut slots);
    rx(&mut node, &std_frame(255, 190, 0, &att), 10);
    let s = *node.stats();
    assert_eq!(s.dropped_unknown_message, 1);
    assert_eq!(s.heartbeats_received, 0);
    assert_eq!(s.dropped_not_for_us, 0);
}

#[test]
fn parameter_and_command_frames_for_another_target_are_checked_counted_and_dropped() {
    use mavlink::dialects::common as c;
    let msgs = [
        Std::PARAM_REQUEST_LIST(c::PARAM_REQUEST_LIST_DATA {
            target_system: 9,
            target_component: 1,
        }),
        Std::PARAM_REQUEST_READ(c::PARAM_REQUEST_READ_DATA {
            target_system: 1,
            target_component: 9,
            ..Default::default()
        }),
        Std::PARAM_SET(c::PARAM_SET_DATA {
            target_system: 9,
            ..Default::default()
        }),
        Std::COMMAND_LONG(c::COMMAND_LONG_DATA {
            target_system: 9,
            ..Default::default()
        }),
    ];
    let mut slots = [];
    let mut node = node_with(&mut slots);
    for (i, m) in msgs.iter().enumerate() {
        rx(&mut node, &std_frame(255, 190, i as u8, m), 10);
    }
    assert_eq!(node.stats().dropped_not_for_us, 4);
    let mut bad = std_frame(255, 190, 9, &msgs[3]);
    let n = bad.len();
    bad[n - 2] ^= 0x01;
    rx(&mut node, &bad, 10);
    let s = *node.stats();
    assert_eq!((s.dropped_not_for_us, s.dropped_checksum), (4, 1));
    assert_eq!(
        drain(&mut node, 20, &mut Fake::default()).len(),
        1,
        "only the heartbeat"
    );
}

#[test]
fn frames_with_the_nodes_own_system_id_are_dropped_and_counted_separately() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    let f = std_frame(
        1,
        190,
        0,
        &hb_msg(mavlink::dialects::common::MavType::MAV_TYPE_GCS),
    );
    rx(&mut node, &f, 10);
    let s = *node.stats();
    assert_eq!(s.dropped_own_system, 1);
    assert_eq!(s.heartbeats_received, 0);
    assert!(!node.gcs_present(10));
}

#[test]
fn a_false_start_never_swallows_a_real_frame() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    // A lone 0xFD before a frame.
    let mut bytes = vec![0xFD];
    bytes.extend(gcs_hb(0));
    rx(&mut node, &bytes, 10);
    assert_eq!(node.stats().heartbeats_received, 1);
    // A false start that looks like a 34-byte frame of a handled message, then two real frames.
    let mut bytes = vec![0xFD, 34, 0, 0, 0, 1, 1, 0, 0, 0];
    bytes.extend(gcs_hb(1));
    bytes.extend(gcs_hb(2));
    rx(&mut node, &bytes, 10);
    assert_eq!(node.stats().heartbeats_received, 3);
    assert_eq!(node.stats().dropped_checksum, 1);
    // One at a time, with a long false start whose frame never completes before the real one ends.
    let mut bytes = vec![0xFD, 255, 0, 0, 0, 1, 1, 0, 0, 0];
    bytes.extend(gcs_hb(3));
    bytes.extend(vec![0u8; 300]);
    for b in &bytes {
        rx(&mut node, &[*b], 10);
    }
    assert_eq!(node.stats().heartbeats_received, 4);
}

#[test]
fn the_receive_buffer_is_about_one_maximum_frame() {
    assert_eq!(crate::deframer::RX_BUF, 280);
    let mut slots = [];
    let mut node = node_with(&mut slots);
    let flood: Vec<u8> = (0..20_000u32)
        .map(|i| if i % 3 == 0 { 0xFD } else { 0xFF })
        .collect();
    for chunk in flood.chunks(77) {
        rx(&mut node, chunk, 10);
        assert!(node.rx_len() <= 266);
    }
}

// ---- 3. heartbeat ----

#[test]
fn heartbeat_every_second_carries_config_and_status() {
    let cfg = Config::new(1, 1)
        .with_vehicle_type(MavType::MAV_TYPE_QUADROTOR)
        .with_autopilot(MavAutopilot::MAV_AUTOPILOT_PX4);
    let mut slots = [];
    let mut node = Node::new(cfg, &mut slots);
    let mut fake = Fake::default();
    let (_, m, _) = pull(&mut node, 1234, &mut fake).unwrap();
    let h = hb_of(&m);
    assert_eq!(h.mavtype as u32, MavType::MAV_TYPE_QUADROTOR as u32);
    assert_eq!(h.autopilot as u32, MavAutopilot::MAV_AUTOPILOT_PX4 as u32);
    assert_eq!(h.base_mode.bits(), 0);
    assert_eq!(h.custom_mode, 0);
    assert_eq!(h.system_status as u32, MavState::MAV_STATE_BOOT as u32);
    assert_eq!(h.mavlink_version, 3);
    assert!(pull(&mut node, 1234 + 999_999, &mut fake).is_none());
    assert!(pull(&mut node, 1234 + 1_000_000, &mut fake).is_some());
    assert!(pull(&mut node, 1234 + 1_999_999, &mut fake).is_none());
    assert!(pull(&mut node, 1234 + 2_000_000, &mut fake).is_some());
}

#[test]
fn the_default_heartbeat_is_generic_generic() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    let (_, m, _) = pull(&mut node, 0, &mut Fake::default()).unwrap();
    let h = hb_of(&m);
    assert_eq!(h.mavtype as u32, 0);
    assert_eq!(h.autopilot as u32, 0);
}

#[test]
fn a_late_poll_sends_one_heartbeat_not_a_burst() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::default();
    assert_eq!(drain(&mut node, 0, &mut fake).len(), 1);
    assert_eq!(drain(&mut node, 10_500_000, &mut fake).len(), 1);
    assert!(pull(&mut node, 10_999_999, &mut fake).is_none());
    assert!(pull(&mut node, 11_000_000, &mut fake).is_some());
}

#[test]
fn a_heartbeat_waits_at_most_one_frame_whatever_the_telemetry_load() {
    // Every stream at its fastest, four IMU frames per sample, a link that takes 1 ms per frame.
    let mut slots = [
        slot(105, 5_300, 5_000),
        slot(29, 20_700, 20_000),
        slot(24, 100_300, 100_000),
    ];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(105, 4, true), (29, 1, true), (24, 1, true)]);
    const FRAME_US: u64 = 1_000;
    let mut t = 0u64;
    let mut heartbeats = Vec::new();
    let mut texts_sent = 0;
    let mut next_text = 0u64;
    while t < 12_000_000 {
        if t >= next_text {
            let _ = node.queue_text(MavSeverity::MAV_SEVERITY_INFO, "load");
            next_text += 250_000;
        }
        match pull(&mut node, t, &mut fake) {
            Some((_, Std::HEARTBEAT(_), _)) => {
                heartbeats.push(t);
                t += FRAME_US;
            }
            Some((_, Std::STATUSTEXT(_), _)) => {
                texts_sent += 1;
                t += FRAME_US;
            }
            Some(_) => t += FRAME_US,
            None => t += 100,
        }
    }
    assert!(texts_sent > 10);
    assert!(heartbeats.len() >= 12, "{heartbeats:?}");
    for (k, &sent) in heartbeats.iter().enumerate() {
        let due = k as u64 * 1_000_000;
        assert!(
            sent >= due && sent - due <= FRAME_US,
            "heartbeat {k}: due {due}, sent {sent}"
        );
    }
}

#[test]
fn set_status_changes_the_next_heartbeat() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::default();
    let (_, m, _) = pull(&mut node, 0, &mut fake).unwrap();
    assert_eq!(
        hb_of(&m).system_status as u32,
        MavState::MAV_STATE_BOOT as u32
    );
    node.set_status(MavState::MAV_STATE_STANDBY);
    let (_, m, _) = pull(&mut node, 1_000_000, &mut fake).unwrap();
    assert_eq!(
        hb_of(&m).system_status as u32,
        MavState::MAV_STATE_STANDBY as u32
    );
}

// ---- 4. presence ----

#[test]
fn the_ground_station_is_present_until_the_timeout_passes_without_a_heartbeat() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    assert!(!node.gcs_present(0));
    rx(&mut node, &gcs_hb(0), 1_000_000);
    assert!(node.gcs_present(1_000_000));
    assert!(node.gcs_present(5_999_999));
    assert!(!node.gcs_present(6_000_000));
    rx(&mut node, &gcs_hb(1), 5_000_000);
    assert!(node.gcs_present(9_999_999));
    assert!(!node.gcs_present(10_000_000));
}

#[test]
fn the_timeout_comes_from_the_config() {
    let mut slots = [];
    let mut node = Node::new(Config::new(1, 1).with_gcs_timeout_us(2_000), &mut slots);
    rx(&mut node, &gcs_hb(0), 100);
    assert!(node.gcs_present(2_099));
    assert!(!node.gcs_present(2_100));
}

#[test]
fn only_a_heartbeat_of_type_gcs_counts_as_the_ground_station() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    let other = std_frame(
        2,
        1,
        0,
        &hb_msg(mavlink::dialects::common::MavType::MAV_TYPE_QUADROTOR),
    );
    rx(&mut node, &other, 10);
    assert_eq!(node.stats().heartbeats_received, 1);
    assert!(!node.gcs_present(10));
    assert_eq!(node.stats().last_gcs, None);
}

#[test]
fn stats_record_which_system_and_component_sent_the_last_gcs_heartbeat() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    rx(&mut node, &gcs_hb(0), 10);
    assert_eq!(
        node.stats().last_gcs,
        Some(Peer {
            system_id: 255,
            component_id: 190
        })
    );
    let second = std_frame(
        254,
        191,
        1,
        &hb_msg(mavlink::dialects::common::MavType::MAV_TYPE_GCS),
    );
    rx(&mut node, &second, 20);
    assert_eq!(
        node.stats().last_gcs,
        Some(Peer {
            system_id: 254,
            component_id: 191
        })
    );
}

#[test]
fn losing_the_ground_station_changes_nothing_on_the_wire() {
    let mut slots = [slot(29, 100_000, 20_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(29, 1, true)]);
    rx(&mut node, &gcs_hb(0), 0);
    assert_eq!(drain(&mut node, 0, &mut fake).len(), 2);
    assert!(!node.gcs_present(6_000_000));
    let frames = drain(&mut node, 6_000_000, &mut fake);
    assert_eq!(
        frames.len(),
        2,
        "one heartbeat, one pressure sample, nothing else"
    );
}

// ---- 5. streams ----

#[test]
fn streams_start_at_their_default_interval_and_a_negative_default_is_off() {
    let mut slots = [
        slot(105, 20_000, 5_000),
        slot(29, -1, 20_000),
        slot(24, 0, 100_000),
    ];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(105, 1, true), (29, 1, true), (24, 1, true)]);
    for t in 0..=1_000_000u64 {
        if t.is_multiple_of(1000) {
            drain(&mut node, t, &mut fake);
        }
    }
    assert_eq!(fake.count(105), 51);
    assert_eq!(fake.count(29), 0);
    assert_eq!(fake.count(24), 0);
}

#[test]
fn a_default_below_the_minimum_is_raised_to_it() {
    assert_eq!(slot(105, 1_000, 5_000).interval_us(), 5_000);
    assert_eq!(slot(105, 9_000, 5_000).interval_us(), 9_000);
    assert_eq!(slot(105, -1, 5_000).interval_us(), -1);
}

#[test]
fn a_sample_is_taken_once_per_period_and_every_pushed_frame_is_sent_in_order() {
    let mut slots = [slot(105, 20_000, 5_000), slot(29, 100_000, 20_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(105, 4, true), (29, 1, true)]);
    let (_, m, _) = pull(&mut node, 0, &mut fake).unwrap();
    hb_of(&m);
    assert!(fake.calls.is_empty());
    let (_, m, _) = pull(&mut node, 0, &mut fake).unwrap();
    assert_eq!(imu_id(&m), 0);
    assert_eq!(
        fake.calls,
        [(105, 0)],
        "slot 1 is not sampled while slot 0 has frames to go"
    );
    for want in 1..4 {
        let (_, m, _) = pull(&mut node, 0, &mut fake).unwrap();
        assert_eq!(imu_id(&m), want);
        assert_eq!(fake.calls.len(), 1);
    }
    let (_, m, _) = pull(&mut node, 0, &mut fake).unwrap();
    assert!(matches!(m, Std::SCALED_PRESSURE(_)));
    assert_eq!(fake.calls, [(105, 0), (29, 0)], "slot order");
    assert!(pull(&mut node, 0, &mut fake).is_none());
    assert!(pull(&mut node, 19_999, &mut fake).is_none());
    assert_eq!(fake.calls.len(), 2);
    let (_, m, _) = pull(&mut node, 20_000, &mut fake).unwrap();
    assert_eq!(imu_id(&m), 0);
    assert_eq!(fake.calls.len(), 3);
}

#[test]
fn sample_frames_carry_what_the_hook_put_in_them() {
    let mut slots = [slot(105, 20_000, 5_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(105, 4, true)]);
    let frames = drain(&mut node, 123_456, &mut fake);
    assert_eq!(frames.len(), 5);
    for (k, (_, m)) in frames[1..].iter().enumerate() {
        match m {
            Std::HIGHRES_IMU(i) => assert_eq!((i.id as usize, i.time_usec), (k, 123_456)),
            other => panic!("{other:?}"),
        }
    }
}

#[test]
fn a_sample_returning_false_or_pushing_nothing_sends_nothing_and_uses_up_the_period() {
    let mut slots = [slot(29, 100_000, 20_000), slot(24, 100_000, 20_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(29, 2, false), (24, 0, true)]);
    assert_eq!(
        drain(&mut node, 0, &mut fake).len(),
        1,
        "only the heartbeat"
    );
    assert_eq!(fake.calls, [(29, 0), (24, 0)]);
    assert!(drain(&mut node, 50_000, &mut fake).is_empty());
    assert_eq!(fake.calls.len(), 2, "no second call in the same period");
    assert!(drain(&mut node, 100_000, &mut fake).is_empty());
    assert_eq!(fake.calls.len(), 4);
}

#[test]
fn missed_periods_are_skipped_not_sent_in_a_burst() {
    let mut slots = [slot(105, 20_000, 5_000), slot(29, 100_000, 20_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(105, 1, true), (29, 1, true)]);
    drain(&mut node, 0, &mut fake);
    assert_eq!((fake.count(105), fake.count(29)), (1, 1));
    let frames = drain(&mut node, 3_000_000, &mut fake);
    assert_eq!((fake.count(105), fake.count(29)), (2, 2));
    assert_eq!(frames.len(), 3, "heartbeat and one frame per stream");
    assert!(pull(&mut node, 3_000_000, &mut fake).is_none());
    assert_eq!(
        node.next_deadline_us(),
        3_020_000,
        "still on the grid of the first period"
    );
}

#[test]
fn the_rate_holds_over_ten_minutes_at_50_hz_without_drift() {
    let mut slots = [slot(105, 20_000, 5_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(105, 1, true)]);
    let mut t = 0u64;
    while t < 600_000_000 {
        drain(&mut node, t, &mut fake);
        t += 1_000;
    }
    let n = fake.count(105) as i64;
    assert!((n - 30_000).abs() <= 1, "{n}");
    // Polled every 3 ms, the grid holds as well.
    let mut slots = [slot(105, 20_000, 5_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(105, 1, true)]);
    let mut t = 0u64;
    while t < 600_000_000 {
        drain(&mut node, t, &mut fake);
        t += 3_000;
    }
    let n = fake.count(105) as i64;
    assert!((n - 30_000).abs() <= 1, "{n}");
}

#[test]
fn outgoing_holds_four_frames_and_the_fifth_push_is_full() {
    let mut slots = [slot(105, 20_000, 5_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(105, 5, true)]);
    let frames = drain(&mut node, 0, &mut fake);
    assert_eq!(
        fake.push_results,
        [Ok(()), Ok(()), Ok(()), Ok(()), Err(Full)]
    );
    assert_eq!(frames.len(), 5, "the heartbeat and four");
}

// ---- 6. next_deadline_us ----

#[test]
fn polling_at_the_deadline_yields_a_frame_and_one_microsecond_before_does_not() {
    let mut slots = [slot(105, 20_000, 5_000), slot(29, 330_000, 20_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(105, 2, true), (29, 1, true)]);
    assert_eq!(node.next_deadline_us(), 0);
    drain(&mut node, 0, &mut fake);
    let mut last = 0;
    for _ in 0..300 {
        let d = node.next_deadline_us();
        assert!(d > last, "{d} {last}");
        assert!(pull(&mut node, d - 1, &mut fake).is_none(), "before {d}");
        assert!(pull(&mut node, d, &mut fake).is_some(), "at {d}");
        drain(&mut node, d, &mut fake);
        last = d;
    }
    assert!(last >= 1_000_000, "the heartbeat deadline was met as well");
}

#[test]
fn queued_text_means_poll_now() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::default();
    drain(&mut node, 5, &mut fake);
    assert_eq!(node.next_deadline_us(), 1_000_005);
    node.queue_text(MavSeverity::MAV_SEVERITY_INFO, "x")
        .unwrap();
    assert_eq!(node.next_deadline_us(), 0);
    drain(&mut node, 6, &mut fake);
    assert_eq!(node.next_deadline_us(), 1_000_005);
}

// ---- 7. texts ----

#[test]
fn four_texts_wait_the_fifth_is_full_and_counted_and_they_leave_in_order_before_streams() {
    let mut slots = [slot(29, 100_000, 20_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(29, 1, true)]);
    for (i, text) in ["a", "b", "c", "d"].iter().enumerate() {
        node.queue_text(MavSeverity::MAV_SEVERITY_WARNING, text)
            .unwrap();
        assert_eq!(node.stats().texts_dropped, 0, "{i}");
    }
    assert_eq!(
        node.queue_text(MavSeverity::MAV_SEVERITY_INFO, "e"),
        Err(Full)
    );
    assert_eq!(node.stats().texts_dropped, 1);
    let frames = drain(&mut node, 0, &mut fake);
    assert!(matches!(frames[0].1, Std::HEARTBEAT(_)));
    let texts: Vec<_> = frames[1..5]
        .iter()
        .map(|(_, m)| match m {
            Std::STATUSTEXT(t) => (t.severity as u32, t.text.to_str().unwrap().to_string()),
            other => panic!("{other:?}"),
        })
        .collect();
    let want = |s: &str| (MavSeverity::MAV_SEVERITY_WARNING as u32, s.to_string());
    assert_eq!(texts, [want("a"), want("b"), want("c"), want("d")]);
    assert!(matches!(frames[5].1, Std::SCALED_PRESSURE(_)));
    assert_eq!(frames.len(), 6);
    node.queue_text(MavSeverity::MAV_SEVERITY_INFO, "f")
        .unwrap();
}

#[test]
fn text_over_50_bytes_is_cut_at_50_on_a_character_boundary() {
    let mut slots = [];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::default();
    drain(&mut node, 0, &mut fake);
    let long = "x".repeat(60);
    node.queue_text(MavSeverity::MAV_SEVERITY_INFO, &long)
        .unwrap();
    node.queue_text(MavSeverity::MAV_SEVERITY_INFO, &"\u{20ac}".repeat(20))
        .unwrap(); // 3 bytes each
    node.queue_text(MavSeverity::MAV_SEVERITY_INFO, &"y".repeat(50))
        .unwrap();
    node.queue_text(MavSeverity::MAV_SEVERITY_ERROR, "")
        .unwrap();
    let got: Vec<_> = drain(&mut node, 1, &mut fake)
        .into_iter()
        .map(|(_, m)| match m {
            Std::STATUSTEXT(t) => (t.text.to_str().unwrap().to_string(), t.id, t.chunk_seq),
            other => panic!("{other:?}"),
        })
        .collect();
    assert_eq!(got[0], ("x".repeat(50), 0, 0));
    assert_eq!(got[1], ("\u{20ac}".repeat(16), 0, 0), "48 bytes, not 50");
    assert_eq!(got[2], ("y".repeat(50), 0, 0));
    assert_eq!(got[3], (String::new(), 0, 0));
}

// ---- 8. link down ----

#[test]
fn link_down_returns_to_defaults_and_drops_what_was_pending() {
    let mut slots = [slot(105, 20_000, 5_000), slot(29, 100_000, 20_000)];
    let mut node = node_with(&mut slots);
    let mut fake = Fake::new(&[(105, 4, true), (29, 1, true)]);
    node.slots_mut()[1].interval_us = 500_000; // as a later SET_MESSAGE_INTERVAL would
    let (h0, _, _) = pull(&mut node, 0, &mut fake).unwrap();
    let (_, m, _) = pull(&mut node, 0, &mut fake).unwrap();
    assert_eq!(imu_id(&m), 0);
    node.queue_text(MavSeverity::MAV_SEVERITY_INFO, "gone")
        .unwrap();
    let hb = gcs_hb(0);
    rx(&mut node, &hb, 1_000);
    rx(&mut node, &hb[..8], 1_000);
    assert!(node.gcs_present(1_000));
    let before = *node.stats();

    node.on_link_down();

    assert!(!node.gcs_present(1_000));
    assert_eq!(node.slots_mut()[1].interval_us(), 100_000);
    assert_eq!(node.rx_len(), 0);
    rx(&mut node, &hb[8..], 1_001);
    assert_eq!(
        node.stats().heartbeats_received,
        before.heartbeats_received,
        "the cut frame is gone"
    );
    fake.calls.clear();
    let frames = drain(&mut node, 10_000, &mut fake);
    assert_eq!(
        fake.calls,
        [(105, 10_000), (29, 10_000)],
        "schedules restarted: both due at once"
    );
    assert_eq!(
        frames.len(),
        4 + 1,
        "no text, no leftover IMU frames, one sample of each"
    );
    assert!(frames.iter().all(|(_, m)| !matches!(m, Std::STATUSTEXT(_))));
    assert_eq!(
        frames[0].0.sequence,
        h0.sequence + 2,
        "the sequence counter carries on"
    );
    drain(&mut node, 110_000, &mut fake);
    let pressure: Vec<_> = fake
        .calls
        .iter()
        .filter(|c| c.0 == 29)
        .map(|c| c.1)
        .collect();
    assert_eq!(
        pressure,
        [10_000, 110_000],
        "default interval again, not 500 ms"
    );
    assert!(node.stats().frames_sent > before.frames_sent);
}

// ---- config, params ----

#[test]
fn config_defaults() {
    let c = Config::new(1, 1);
    assert_eq!(c.gcs_timeout_us, 5_000_000);
    assert_eq!(c.param_list_interval_us, 2_000);
    assert_eq!(c.version, VersionInfo::new());
    assert_eq!(c.vehicle_type, MavType::MAV_TYPE_GENERIC);
    assert_eq!(c.autopilot, MavAutopilot::MAV_AUTOPILOT_GENERIC);
    assert_eq!(
        c.with_param_list_interval_us(5_000).param_list_interval_us,
        5_000
    );
}

#[test]
fn param_names_are_validated() {
    assert!(ParamName::new("A").is_ok());
    assert_eq!(ParamName::new("RATE_IMU_0").unwrap().as_str(), "RATE_IMU_0");
    assert!(ParamName::new("ABCDEFGHIJKLMNOP").is_ok());
    for bad in ["", "ABCDEFGHIJKLMNOPQ", "lower", "A-B", "A B", "\u{e9}"] {
        assert_eq!(ParamName::new(bad), Err(ParamError::InvalidName), "{bad:?}");
    }
}

#[test]
fn the_node_size_is_reported() {
    println!("size_of::<Node>() = {}", core::mem::size_of::<Node<'_>>());
    println!(
        "size_of::<StreamSlot>() = {}",
        core::mem::size_of::<StreamSlot>()
    );
}
