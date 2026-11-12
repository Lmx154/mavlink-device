//! Property tests: arbitrary bytes in arbitrary splits, and arbitrary use of the whole node.

use crate::tests::{Fake, NoParams, gcs_hb, pull};
use crate::*;
use proptest::prelude::*;

fn feed_split(node: &mut Node<'_>, bytes: &[u8], cuts: &[usize], mut now: u64) {
    let mut at = 0;
    for &c in cuts {
        let end = (at + 1 + c).min(bytes.len());
        node.receive(&bytes[at..end], now, &mut NoParams);
        now += 1;
        at = end;
    }
    node.receive(&bytes[at..], now, &mut NoParams);
}

proptest! {
    #[test]
    fn arbitrary_bytes_in_arbitrary_splits_never_panic_and_the_buffer_stays_bounded(
        bytes in proptest::collection::vec(any::<u8>(), 0..3000),
        cuts in proptest::collection::vec(0usize..400, 0..40),
        stray_fd in proptest::collection::vec(0usize..3000, 0..30),
    ) {
        let mut bytes = bytes;
        for i in stray_fd {
            if let Some(b) = bytes.get_mut(i) {
                *b = 0xFD;
            }
        }
        let mut slots = [];
        let mut node = Node::new(Config::new(1, 1), &mut slots);
        let mut at = 0;
        for &c in &cuts {
            let end = (at + 1 + c).min(bytes.len());
            node.receive(&bytes[at..end], 5, &mut NoParams);
            prop_assert!(node.rx_len() <= 266);
            at = end;
        }
        node.receive(&bytes[at..], 5, &mut NoParams);
        prop_assert!(node.rx_len() <= 266);
        let _ = node.gcs_present(6);
    }

    #[test]
    fn valid_frames_in_arbitrary_splits_with_noise_between_are_all_accepted(
        n in 1usize..8,
        noise in proptest::collection::vec(
            proptest::collection::vec(any::<u8>().prop_filter("not a start byte", |b| *b != 0xFD && *b != 0xFE), 0..40),
            8,
        ),
        cuts in proptest::collection::vec(0usize..30, 0..30),
    ) {
        let mut bytes = Vec::new();
        let mut noise_len = 0;
        for (k, nz) in noise.iter().take(n).enumerate() {
            bytes.extend(nz);
            noise_len += nz.len();
            bytes.extend(gcs_hb(k as u8));
        }
        let mut slots = [];
        let mut node = Node::new(Config::new(1, 1), &mut slots);
        feed_split(&mut node, &bytes, &cuts, 10);
        prop_assert_eq!(node.stats().heartbeats_received as usize, n);
        prop_assert_eq!(node.stats().garbage_bytes as usize, noise_len);
        prop_assert!(node.gcs_present(11));
    }

    #[test]
    fn a_real_frame_behind_a_false_start_is_found_once_more_bytes_arrive(
        false_len in 0u8..=255,
        flags_ok in any::<bool>(),
        tail in proptest::collection::vec(0u8..0xFD, 270..300),
    ) {
        let mut bytes = vec![0xFD, false_len, if flags_ok { 0 } else { 1 }, 0, 0, 1, 1, 0, 0, 0];
        bytes.extend(gcs_hb(1));
        bytes.extend(tail);
        let mut slots = [];
        let mut node = Node::new(Config::new(1, 1), &mut slots);
        node.receive(&bytes, 10, &mut NoParams);
        prop_assert_eq!(node.stats().heartbeats_received, 1);
    }

    #[test]
    fn arbitrary_use_emits_only_frames_the_std_reader_accepts(
        steps in proptest::collection::vec((0u64..400_000, 0u8..6, 0usize..6), 1..60),
        intervals in proptest::collection::vec(-1i32..300_000, 3),
    ) {
        let mut slots = [
            StreamSlot::new(Stream { message_id: 105, default_interval_us: intervals[0], min_interval_us: 5_000 }),
            StreamSlot::new(Stream { message_id: 29, default_interval_us: intervals[1], min_interval_us: 20_000 }),
            StreamSlot::new(Stream { message_id: 24, default_interval_us: intervals[2], min_interval_us: 100_000 }),
        ];
        let mut node = Node::new(Config::new(1, 1), &mut slots);
        let mut fake = Fake::new(&[(105, 4, true), (29, 1, true), (24, 6, true)]);
        let mut now = 0u64;
        let mut last_seq: Option<u8> = None;
        for (dt, action, count) in steps {
            now += dt;
            match action {
                0 => { let _ = node.queue_text(dialect::MavSeverity::MAV_SEVERITY_DEBUG, "text \u{20ac}\u{20ac}\u{20ac}"); }
                1 => node.on_link_down(),
                2 => node.receive(&gcs_hb(0)[..count.min(21)], now, &mut NoParams),
                _ => {}
            }
            let mut guard = 0;
            while let Some((h, _, _)) = pull(&mut node, now, &mut fake) {
                if let Some(prev) = last_seq {
                    prop_assert_eq!(h.sequence, prev.wrapping_add(1));
                }
                last_seq = Some(h.sequence);
                guard += 1;
                prop_assert!(guard < 100);
            }
            prop_assert!(node.next_deadline_us() > now);
        }
    }
}
