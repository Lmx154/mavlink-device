//! The accept list, the hand-over hook and the RADIO_STATUS slowdown.

use crate::dialect::{HEARTBEAT_DATA, RADIO_STATUS_DATA, RC_CHANNELS_OVERRIDE_DATA};
use crate::tests::{Fake, NoParams, gcs_hb, std_frame};
use crate::*;
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MessageData, calculate_crc};
use mavlink::dialects::common::MavMessage as Std;
use proptest::prelude::*;

const OWN: u8 = 1;
/// A message of "our own dialect": an id and a CRC_EXTRA the node knows nothing else about.
const CUSTOM_ID: u32 = 42_001;
const CUSTOM_EXTRA: u8 = 77;

type Seen = Vec<(u8, u8, u32, Vec<u8>, u64)>;

fn rc_pair() -> (u32, u8) {
    (
        RC_CHANNELS_OVERRIDE_DATA::ID,
        RC_CHANNELS_OVERRIDE_DATA::EXTRA_CRC,
    )
}

fn config(pairs: &[(u32, u8)]) -> Config {
    Config::new(OWN, 1).with_accept_list(pairs).unwrap()
}

/// A frame built by hand, with the checksum for `extra`.
fn raw(stx: u8, flags: u8, sys: u8, comp: u8, id: u32, payload: &[u8], extra: u8) -> Vec<u8> {
    let id = id.to_le_bytes();
    let mut f = vec![
        stx,
        payload.len() as u8,
        flags,
        0,
        0,
        sys,
        comp,
        id[0],
        id[1],
        id[2],
    ];
    f.extend_from_slice(payload);
    let crc = calculate_crc(&f[1..], extra);
    f.extend_from_slice(&crc.to_le_bytes());
    f
}

fn custom(sys: u8, comp: u8, payload: &[u8]) -> Vec<u8> {
    raw(0xFD, 0, sys, comp, CUSTOM_ID, payload, CUSTOM_EXTRA)
}

fn rc_override(sys: u8, comp: u8, c1: u16, c2: u16) -> Vec<u8> {
    let m = RC_CHANNELS_OVERRIDE_DATA {
        chan1_raw: c1,
        chan2_raw: c2,
        ..Default::default()
    };
    let mut r = MAVLinkV2MessageRaw::new();
    r.serialize_message_data(
        MavHeader {
            system_id: u32::from(sys),
            component_id: comp,
            sequence: 3,
        },
        &m,
    );
    r.raw_bytes().to_vec()
}

fn radio_status(sys: u8, txbuf: u8) -> Vec<u8> {
    let m = RADIO_STATUS_DATA {
        rssi: 200,
        txbuf,
        ..Default::default()
    };
    let mut r = MAVLinkV2MessageRaw::new();
    r.serialize_message_data(
        MavHeader {
            system_id: u32::from(sys),
            component_id: 68,
            sequence: 0,
        },
        &m,
    );
    r.raw_bytes().to_vec()
}

fn feed(node: &mut Node<'_>, bytes: &[u8], now: u64, seen: &mut Seen) {
    node.receive_with(bytes, now, &mut NoParams, &mut |m: Incoming<'_>| {
        seen.push((
            m.system_id,
            m.component_id,
            m.message_id,
            m.payload.to_vec(),
            m.now_us,
        ));
    });
}

fn one_shot(node: &mut Node<'_>, bytes: &[u8]) -> Seen {
    let mut seen = Seen::new();
    feed(node, bytes, 123, &mut seen);
    seen
}

// ---- the accept list ----

#[test]
fn the_accept_list_holds_16_pairs_and_refuses_17_without_truncating() {
    let pairs: Vec<(u32, u8)> = (0..17).map(|i| (1000 + i, i as u8)).collect();
    assert!(Config::new(1, 1).with_accept_list(&pairs[..16]).is_ok());
    assert_eq!(
        Config::new(1, 1).with_accept_list(&pairs).unwrap_err(),
        AcceptListFull
    );
    assert!(
        Config::new(1, 1)
            .with_accept_list(&[])
            .unwrap()
            .accept
            .is_empty()
    );
    let c = Config::new(1, 1).with_accept_list(&pairs[..16]).unwrap();
    assert_eq!(c.accept.len(), 16);
    assert_eq!(MAX_ACCEPT, 16);
}

#[test]
fn the_default_accept_list_is_empty_and_hands_nothing_over() {
    let mut slots = [];
    let mut node = Node::new(Config::new(OWN, 1), &mut slots);
    assert!(one_shot(&mut node, &rc_override(255, 190, 1500, 1600)).is_empty());
    assert_eq!(node.stats().dropped_unknown_message, 1);
}

// ---- the hand-over ----

#[test]
fn a_listed_frame_with_a_good_checksum_is_handed_over_once_with_its_ids_payload_and_time() {
    let mut slots = [];
    let mut node = Node::new(config(&[rc_pair()]), &mut slots);
    let f = rc_override(255, 190, 1500, 1600);
    let seen = one_shot(&mut node, &f);
    assert_eq!(seen.len(), 1);
    let (sys, comp, id, payload, now) = &seen[0];
    assert_eq!((*sys, *comp, *id, *now), (255, 190, 70, 123));
    assert_eq!(payload.as_slice(), &f[10..f.len() - 2]);
    let s = *node.stats();
    assert_eq!(s.dropped_listed_checksum, 0);
    assert_eq!(s.dropped_unknown_message, 0);
    assert_eq!(s.dropped_own_system, 0);
}

#[test]
fn a_truncated_rc_channels_override_decodes_with_the_dialect_type() {
    let mut slots = [];
    let mut node = Node::new(config(&[rc_pair()]), &mut slots);
    let f = rc_override(255, 190, 1500, 1600);
    let mut got = None;
    node.receive_with(&f, 5, &mut NoParams, &mut |m| {
        assert!(m.payload.len() < RC_CHANNELS_OVERRIDE_DATA::ENCODED_LEN);
        assert_eq!(m.payload.len(), 4, "the trailing zeros are cut on the wire");
        assert!(m.decode::<HEARTBEAT_DATA>().is_none());
        got = m.decode::<RC_CHANNELS_OVERRIDE_DATA>();
    });
    let m = got.expect("decoded");
    assert_eq!((m.chan1_raw, m.chan2_raw, m.chan3_raw), (1500, 1600, 0));
    assert_eq!((m.target_system, m.target_component), (0, 0));
    assert_eq!(m.chan18_raw, 0);
}

#[test]
fn a_listed_frame_of_the_custom_dialect_is_handed_over() {
    let mut slots = [];
    let mut node = Node::new(config(&[(CUSTOM_ID, CUSTOM_EXTRA)]), &mut slots);
    let seen = one_shot(&mut node, &custom(7, 9, &[1, 2, 3]));
    assert_eq!(seen, vec![(7, 9, CUSTOM_ID, vec![1, 2, 3], 123)]);
    let seen = one_shot(&mut node, &custom(7, 9, &[]));
    assert_eq!(seen, vec![(7, 9, CUSTOM_ID, vec![], 123)]);
}

#[test]
fn receive_without_a_hook_drops_listed_frames_silently() {
    let mut slots = [];
    let mut node = Node::new(config(&[rc_pair()]), &mut slots);
    node.receive(&rc_override(255, 190, 1500, 1600), 1, &mut NoParams);
    let s = *node.stats();
    assert_eq!(s.dropped_listed_checksum, 0);
    assert_eq!(s.dropped_unknown_message, 0);
}

#[test]
fn bytes_one_at_a_time_and_every_two_part_split_give_the_same_single_hand_over() {
    let mut slots = [];
    let mut node = Node::new(config(&[rc_pair()]), &mut slots);
    let mut stream = vec![0x11, 0x22];
    stream.extend(rc_override(255, 190, 1500, 1600));
    let want = one_shot(&mut node, &stream);
    assert_eq!(want.len(), 1);
    let mut seen = Seen::new();
    for b in &stream {
        feed(&mut node, std::slice::from_ref(b), 123, &mut seen);
    }
    assert_eq!(seen, want);
    for cut in 0..=stream.len() {
        let mut node = Node::new(config(&[rc_pair()]), &mut slots);
        let mut seen = Seen::new();
        feed(&mut node, &stream[..cut], 123, &mut seen);
        feed(&mut node, &stream[cut..], 123, &mut seen);
        assert_eq!(seen, want, "cut at {cut}");
        assert_eq!(node.stats().garbage_bytes, 2);
    }
}

proptest! {
    #[test]
    fn listed_frames_in_arbitrary_splits_with_noise_are_each_handed_over_once(
        n in 1usize..6,
        noise in proptest::collection::vec(
            proptest::collection::vec(any::<u8>().prop_filter("not a start byte", |b| *b != 0xFD && *b != 0xFE), 0..30),
            6,
        ),
        cuts in proptest::collection::vec(0usize..40, 0..30),
    ) {
        let mut bytes = Vec::new();
        let mut want = Vec::new();
        for (k, nz) in noise.iter().take(n).enumerate() {
            bytes.extend(nz);
            bytes.extend(rc_override(255, 190, 1000 + k as u16, 2000));
            want.push(1000 + k as u16);
            bytes.extend(gcs_hb(k as u8));
        }
        let mut slots = [];
        let mut node = Node::new(config(&[rc_pair()]), &mut slots);
        let mut seen = Seen::new();
        let mut at = 0;
        for &c in &cuts {
            let end = (at + 1 + c).min(bytes.len());
            feed(&mut node, &bytes[at..end], 9, &mut seen);
            at = end;
        }
        feed(&mut node, &bytes[at..], 9, &mut seen);
        let got: Vec<u16> = seen.iter().map(|s| u16::from_le_bytes([s.3[0], s.3[1]])).collect();
        prop_assert_eq!(got, want);
        prop_assert_eq!(node.stats().heartbeats_received as usize, n);
        prop_assert_eq!(node.stats().dropped_listed_checksum, 0);
    }
}

// ---- I8, I9, I10 and the other refusals ----

#[test]
fn i8_a_listed_frame_with_a_wrong_checksum_is_never_handed_over_and_is_counted() {
    let good = rc_override(255, 190, 1500, 1600);
    for i in 1..good.len() {
        let mut bad = good.clone();
        bad[i] ^= 0x01;
        let mut slots = [];
        let mut node = Node::new(config(&[rc_pair()]), &mut slots);
        let seen = one_shot(&mut node, &bad);
        assert!(seen.is_empty(), "byte {i} flipped");
        // Flipping the length, the flags or the id byte changes how the frame is read; flipping
        // anything else leaves a listed candidate with a wrong checksum.
        if i >= 3 && i != 7 && i != 8 && i != 9 && i != 2 {
            assert_eq!(node.stats().dropped_listed_checksum, 1, "byte {i} flipped");
        }
    }
    let mut slots = [];
    let mut node = Node::new(config(&[rc_pair()]), &mut slots);
    let mut bad = good.clone();
    let n = bad.len();
    bad[n - 1] ^= 0xFF;
    assert!(one_shot(&mut node, &bad).is_empty());
    assert_eq!(node.stats().dropped_listed_checksum, 1);
    assert_eq!(node.stats().dropped_checksum, 0);
    // The same frame, whole, still gets through afterwards.
    assert_eq!(one_shot(&mut node, &good).len(), 1);
}

#[test]
fn i8_a_listed_frame_checked_with_another_extra_than_the_senders_is_not_handed_over() {
    let mut slots = [];
    let mut node = Node::new(config(&[(CUSTOM_ID, CUSTOM_EXTRA + 1)]), &mut slots);
    assert!(one_shot(&mut node, &custom(7, 9, &[1, 2, 3])).is_empty());
    assert_eq!(node.stats().dropped_listed_checksum, 1);
}

#[test]
fn i9_an_unlisted_unknown_id_is_never_handed_over_and_is_counted_as_before() {
    let mut slots = [];
    let mut node = Node::new(config(&[rc_pair()]), &mut slots);
    let att = Std::ATTITUDE(mavlink::dialects::common::ATTITUDE_DATA::default());
    let mut stream = std_frame(255, 190, 0, &att);
    stream.extend(custom(7, 9, &[1])); // CUSTOM_ID is not listed in this node
    let seen = one_shot(&mut node, &stream);
    assert!(seen.is_empty());
    let s = *node.stats();
    assert!(s.dropped_unknown_message >= 2);
    assert_eq!(s.dropped_listed_checksum, 0);
    assert_eq!(s.dropped_checksum, 0);
}

#[test]
fn i10_a_listed_frame_with_the_nodes_own_system_id_is_never_handed_over() {
    let mut slots = [];
    let mut node = Node::new(config(&[rc_pair(), (CUSTOM_ID, CUSTOM_EXTRA)]), &mut slots);
    assert!(one_shot(&mut node, &rc_override(OWN, 190, 1500, 1600)).is_empty());
    assert!(one_shot(&mut node, &custom(OWN, 1, &[1])).is_empty());
    assert_eq!(node.stats().dropped_own_system, 2);
    assert_eq!(one_shot(&mut node, &custom(OWN + 1, 1, &[1])).len(), 1);
}

#[test]
fn a_listed_id_as_mavlink1_or_with_an_incompatibility_flag_is_not_handed_over() {
    let mut slots = [];
    let mut node = Node::new(config(&[(CUSTOM_ID, CUSTOM_EXTRA)]), &mut slots);
    // MAVLink 1: start byte 0xFE, six header bytes, the one-byte id.
    let v1 = [0xFE, 1, 0, 7, 9, 70, 0xAA, 0, 0];
    assert!(one_shot(&mut node, &v1).is_empty());
    assert_eq!(node.stats().dropped_mavlink1, 1);
    for flags in [1u8, 0x80, 0xFF] {
        let mut signed = raw(0xFD, flags, 7, 9, CUSTOM_ID, &[1, 2, 3], CUSTOM_EXTRA);
        signed.extend([0u8; 13]);
        assert!(one_shot(&mut node, &signed).is_empty(), "flags {flags:#x}");
    }
    assert_eq!(node.stats().dropped_incompat, 3);
    assert_eq!(node.stats().dropped_listed_checksum, 0);
    assert_eq!(one_shot(&mut node, &custom(7, 9, &[1])).len(), 1);
}

#[test]
fn a_list_entry_for_an_id_the_node_handles_itself_does_not_hand_the_frame_over() {
    let mut slots = [];
    let mut node = Node::new(
        config(&[(HEARTBEAT_DATA::ID, HEARTBEAT_DATA::EXTRA_CRC)]),
        &mut slots,
    );
    assert!(one_shot(&mut node, &gcs_hb(0)).is_empty());
    assert_eq!(node.stats().heartbeats_received, 1);
}

// ---- RADIO_STATUS ----

#[test]
fn radio_status_is_handled_by_the_node_and_handed_over_only_when_listed() {
    let mut slots = [];
    let mut plain = Node::new(config(&[]), &mut slots);
    assert!(one_shot(&mut plain, &radio_status(255, 10)).is_empty());
    assert_eq!(plain.stream_slowdown_ms(), 60);
    assert_eq!(plain.stats().dropped_unknown_message, 0);
    let mut slots2 = [];
    let mut listed = Node::new(
        config(&[(RADIO_STATUS_DATA::ID, RADIO_STATUS_DATA::EXTRA_CRC)]),
        &mut slots2,
    );
    let f = radio_status(255, 10);
    let seen = one_shot(&mut listed, &f);
    assert_eq!(seen.len(), 1);
    assert_eq!((seen[0].0, seen[0].1, seen[0].2), (255, 68, 109));
    assert_eq!(listed.stream_slowdown_ms(), 60);
}

#[test]
fn radio_status_with_a_wrong_checksum_or_the_own_system_id_does_not_move_the_slowdown() {
    let mut slots = [];
    let mut node = Node::new(config(&[]), &mut slots);
    let mut bad = radio_status(255, 10);
    let n = bad.len();
    bad[n - 1] ^= 1;
    one_shot(&mut node, &bad);
    assert_eq!(node.stats().dropped_checksum, 1);
    one_shot(&mut node, &radio_status(OWN, 10));
    assert_eq!(node.stats().dropped_own_system, 1);
    assert_eq!(node.stream_slowdown_ms(), 0);
}

#[test]
fn a_radio_status_whose_zero_bytes_were_cut_is_read_with_txbuf_zero() {
    let mut slots = [];
    let mut node = Node::new(config(&[]), &mut slots);
    let zero = RADIO_STATUS_DATA::default();
    let mut r = MAVLinkV2MessageRaw::new();
    r.serialize_message_data(
        MavHeader {
            system_id: 255,
            component_id: 68,
            sequence: 0,
        },
        &zero,
    );
    assert_eq!(r.payload().len(), 1);
    one_shot(&mut node, r.raw_bytes());
    assert_eq!(node.stream_slowdown_ms(), 60);
}

/// (slowdown before, txbuf, slowdown after): one vector per branch and per bound.
const VECTORS: &[(u32, u8, u32)] = &[
    // branch 1: txbuf < 20, slowdown < 2000: +60
    (0, 0, 60),
    (0, 19, 60),
    (1940, 19, 2000),
    (1980, 10, 2040),
    (2000, 10, 2000),
    (2040, 0, 2040),
    // branch 2: txbuf < 50 (and not < 20), slowdown < 2000: +20
    (0, 20, 20),
    (0, 49, 20),
    (1980, 30, 2000),
    (2000, 30, 2000),
    // between the thresholds nothing moves
    (500, 50, 500),
    (500, 90, 500),
    // branch 3: txbuf > 95, slowdown > 200: -40
    (240, 96, 200),
    (1000, 100, 960),
    (201, 96, 161),
    // the 200 ms condition of branch 3: at 200 it falls through to branch 4 (-20)
    (200, 96, 180),
    (200, 100, 180),
    // branch 4: txbuf > 90, slowdown != 0: -20, floor 0
    (300, 95, 280),
    (300, 91, 280),
    (300, 90, 300),
    (20, 100, 0),
    (10, 100, 0),
    (0, 100, 0),
    (0, 91, 0),
];

#[test]
fn i19_every_slowdown_branch_and_bound_has_a_vector() {
    for &(before, txbuf, after) in VECTORS {
        assert_eq!(
            crate::node::next_slowdown_ms(before, txbuf),
            after,
            "slowdown {before} txbuf {txbuf}"
        );
    }
}

#[test]
fn i19_the_node_follows_the_vectors_from_received_frames() {
    let mut slots = [];
    let mut node = Node::new(config(&[]), &mut slots);
    // Climb from 0 to 2000 in the +60 branch, then in the +20 branch, and stop there.
    let mut want = 0u32;
    for _ in 0..40 {
        one_shot(&mut node, &radio_status(255, 0));
        want = if want < 2000 { want + 60 } else { want };
        assert_eq!(node.stream_slowdown_ms(), want);
    }
    assert_eq!(want, 2040);
    for _ in 0..5 {
        one_shot(&mut node, &radio_status(255, 30));
    }
    assert_eq!(node.stream_slowdown_ms(), 2040);
    // Down: -40 above 200, then -20, floor 0.
    let mut seen = vec![];
    for _ in 0..80 {
        one_shot(&mut node, &radio_status(255, 100));
        seen.push(node.stream_slowdown_ms());
    }
    assert_eq!(seen[0], 2000);
    assert_eq!(seen.last(), Some(&0));
    assert!(seen.windows(2).all(|w| w[1] <= w[0]));
    node.on_link_down();
    assert_eq!(node.stream_slowdown_ms(), 0);
    one_shot(&mut node, &radio_status(255, 0));
    assert_eq!(node.stream_slowdown_ms(), 60);
    node.on_link_down();
    assert_eq!(node.stream_slowdown_ms(), 0);
}

fn pressure_slot() -> StreamSlot {
    StreamSlot::new(Stream {
        message_id: 29,
        default_interval_us: 100_000,
        min_interval_us: 20_000,
    })
}

/// A node with a 100 ms stream, a RADIO_STATUS with `txbuf` 0 (+60 ms) before the first sample;
/// `true` when the next deadline of the stream is later by exactly 60 ms than without it.
fn slowdown_moves_the_deadline_by_exactly_the_slowdown(switch_off: bool) -> bool {
    let mut plain_slots = [pressure_slot()];
    let mut plain = Node::new(config(&[]), &mut plain_slots);
    let mut slow_slots = [pressure_slot()];
    let mut slow = Node::new(config(&[]), &mut slow_slots);
    #[cfg(feature = "_test-hooks")]
    slow.set_slowdown_off(switch_off);
    #[cfg(not(feature = "_test-hooks"))]
    assert!(!switch_off);
    one_shot(&mut slow, &radio_status(255, 0));
    assert_eq!(slow.stream_slowdown_ms(), 60);
    let mut frame = [0u8; FRAME_BUF];
    let mut tel = Fake::new(&[(29, 1, true)]);
    let mut tel2 = Fake::new(&[(29, 1, true)]);
    let mut ok = true;
    for now in [0u64, 100_000, 160_000, 400_000] {
        while plain
            .poll_transmit(now, &NoParams, &mut tel, &mut frame)
            .is_some()
        {}
        while slow
            .poll_transmit(now, &NoParams, &mut tel2, &mut frame)
            .is_some()
        {}
        let a = plain.slots_mut()[0].deadline_us().unwrap();
        let b = slow.slots_mut()[0].deadline_us().unwrap();
        if now == 0 {
            ok &= a == 100_000 && b == 160_000;
        }
        if now == 160_000 {
            // the slow stream was due again at 160 ms and is rescheduled 160 ms later
            ok &= b == 320_000;
            ok &= tel2.count(29) == 2;
        }
        if now == 400_000 {
            ok &= b > 400_000 && (b - 160_000).is_multiple_of(160_000);
        }
    }
    ok
}

#[test]
fn i19_a_streams_next_deadline_is_later_by_exactly_the_slowdown() {
    assert!(slowdown_moves_the_deadline_by_exactly_the_slowdown(false));
}

#[cfg(feature = "_test-hooks")]
#[test]
fn negative_control_the_slowdown_test_fails_when_the_hook_switches_the_slowdown_off() {
    assert!(!slowdown_moves_the_deadline_by_exactly_the_slowdown(true));
}

#[cfg(feature = "_test-hooks")]
#[test]
fn the_hook_switches_only_the_effect_off_not_the_tracking() {
    let mut slots = [pressure_slot()];
    let mut node = Node::new(config(&[]), &mut slots);
    node.set_slowdown_off(true);
    one_shot(&mut node, &radio_status(255, 0));
    assert_eq!(node.stream_slowdown_ms(), 60);
}

#[test]
fn heartbeats_are_not_slowed() {
    let mut slots = [];
    let mut node = Node::new(config(&[]), &mut slots);
    for _ in 0..30 {
        one_shot(&mut node, &radio_status(255, 0));
    }
    let mut frame = [0u8; FRAME_BUF];
    let mut tel = Fake::default();
    let mut times = vec![];
    for k in 0..4_000u64 {
        let now = k * 1000;
        while node
            .poll_transmit(now, &NoParams, &mut tel, &mut frame)
            .is_some()
        {
            times.push(now);
        }
    }
    assert_eq!(times, vec![0, 1_000_000, 2_000_000, 3_000_000]);
}

// ---- no RADIO_STATUS: the schedule is what it was ----

fn transmit_schedule(node: &mut Node<'_>, tel: &mut Fake) -> Vec<(u64, u8)> {
    let mut frame = [0u8; FRAME_BUF];
    let mut out = vec![];
    for k in 0..=500u64 {
        let now = k * 10_000;
        while let Some(n) = node.poll_transmit(now, &NoParams, tel, &mut frame) {
            out.push((now, frame[7]));
            assert!(n > 10);
        }
    }
    out
}

fn expected_schedule() -> Vec<(u64, u8)> {
    let mut e = vec![];
    for k in 0..=500u64 {
        let now = k * 10_000;
        if now % 1_000_000 == 0 {
            e.push((now, 0)); // HEARTBEAT, id 0
        }
        if now % 100_000 == 0 {
            e.push((now, 29)); // SCALED_PRESSURE, 100 ms
        }
        if now % 250_000 == 0 {
            e.push((now, 24)); // GPS_RAW_INT, 250 ms
        }
    }
    e.sort_by_key(|x| x.0);
    e
}

fn sorted(mut v: Vec<(u64, u8)>) -> Vec<(u64, u8)> {
    v.sort_by_key(|x| x.0);
    v
}

fn two_slots() -> [StreamSlot; 2] {
    [
        pressure_slot(),
        StreamSlot::new(Stream {
            message_id: 24,
            default_interval_us: 250_000,
            min_interval_us: 20_000,
        }),
    ]
}

#[test]
fn without_radio_status_the_transmit_schedule_is_the_configured_one() {
    let mut slots = two_slots();
    let mut node = Node::new(config(&[]), &mut slots);
    let mut tel = Fake::new(&[(29, 1, true), (24, 1, true)]);
    assert_eq!(
        sorted(transmit_schedule(&mut node, &mut tel)),
        expected_schedule()
    );
    assert_eq!(node.stream_slowdown_ms(), 0);
}

#[test]
fn other_traffic_and_a_radio_status_with_a_full_buffer_leave_the_schedule_alone() {
    let mut slots = two_slots();
    let mut node = Node::new(config(&[rc_pair()]), &mut slots);
    let mut seen = Seen::new();
    feed(&mut node, &rc_override(255, 190, 1500, 1600), 0, &mut seen);
    feed(&mut node, &gcs_hb(0), 0, &mut seen);
    feed(&mut node, &radio_status(255, 100), 0, &mut seen);
    feed(&mut node, &radio_status(255, 70), 0, &mut seen);
    let mut tel = Fake::new(&[(29, 1, true), (24, 1, true)]);
    assert_eq!(
        sorted(transmit_schedule(&mut node, &mut tel)),
        expected_schedule()
    );
    assert_eq!(node.stream_slowdown_ms(), 0);
}

// ---- I20 ----

#[test]
fn i20_two_nodes_share_no_state() {
    let mut slots_a = two_slots();
    let mut slots_b = two_slots();
    let mut a = Node::new(config(&[rc_pair()]), &mut slots_a);
    let mut b = Node::new(config(&[(CUSTOM_ID, CUSTOM_EXTRA)]), &mut slots_b);
    let b_before = *b.stats();
    let seen_a = one_shot(&mut a, &rc_override(255, 190, 1500, 1600));
    let mut stream_a = radio_status(255, 0);
    stream_a.extend(radio_status(255, 0));
    one_shot(&mut a, &stream_a);
    assert_eq!(seen_a.len(), 1);
    assert_eq!(a.stream_slowdown_ms(), 120);
    assert_eq!(b.stream_slowdown_ms(), 0);
    assert_eq!(*b.stats(), b_before);
    // B gets a different stream: its own listed id, and A's id, which B does not list.
    let mut stream_b = custom(7, 9, &[5]);
    stream_b.extend(rc_override(255, 190, 1, 2));
    let seen_b = one_shot(&mut b, &stream_b);
    assert_eq!(seen_b.len(), 1);
    assert_eq!(seen_b[0].2, CUSTOM_ID);
    assert_eq!(b.stats().dropped_unknown_message, 1);
    assert_eq!(a.stats().dropped_unknown_message, 0);
    assert_eq!(a.stream_slowdown_ms(), 120);
    a.on_link_down();
    assert_eq!(a.stream_slowdown_ms(), 0);
    one_shot(&mut b, &radio_status(255, 30));
    assert_eq!(b.stream_slowdown_ms(), 20);
    assert_eq!(a.stream_slowdown_ms(), 0);
    // Partial frames are per node too.
    let f = custom(7, 9, &[5]);
    let mut seen = Seen::new();
    feed(&mut b, &f[..5], 1, &mut seen);
    feed(&mut a, &f[5..], 1, &mut seen);
    feed(&mut b, &f[5..], 1, &mut seen);
    assert_eq!(seen.len(), 1);
}

#[test]
fn the_parameter_list_interval_is_set_per_node_through_config() {
    assert_eq!(Config::new(1, 1).param_list_interval_us, 2_000);
    let c = Config::new(1, 1).with_param_list_interval_us(40_000);
    assert_eq!(c.param_list_interval_us, 40_000);
    assert_eq!(Config::new(1, 1).param_list_interval_us, 2_000);
}
