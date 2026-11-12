//! End to end on the host: the node against a small ground-station model, over a link that loses
//! 20 % of the frames in each direction. The model is written here with the std `mavlink` crate.

use crate::service_tests::{Mem, cmd, list_req, read_idx, set_f32, set_i32, slot};
use crate::tests::{Fake, decode, std_frame};
use crate::*;
use mavlink::dialects::common as c;
use mavlink::dialects::common::MavMessage as Std;
use std::collections::BTreeMap;

struct Rng(u64);

impl Rng {
    /// True for 80 % of the calls.
    fn passes(&mut self) -> bool {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0 % 100 >= 20
    }
}

#[derive(Default)]
struct Gcs {
    seq: u8,
    count: Option<u16>,
    values: BTreeMap<u16, (String, u32, u32)>,
    echoes: Vec<(String, u32, u32, u16)>,
    acks: Vec<(u32, u32)>,
    last_value_us: u64,
    lost_to_gcs: u32,
    lost_to_node: u32,
}

struct World<'a> {
    node: Node<'a>,
    mem: Mem,
    tel: Fake,
    gcs: Gcs,
    rng: Rng,
    now: u64,
}

impl<'a> World<'a> {
    fn new(node: Node<'a>, mem: Mem, tel: Fake) -> Self {
        Self {
            node,
            mem,
            tel,
            gcs: Gcs::default(),
            rng: Rng(0x9E37_79B9_7F4A_7C15),
            now: 0,
        }
    }

    /// The ground station sends a message over the lossy link.
    fn send(&mut self, m: &Std) {
        self.gcs.seq = self.gcs.seq.wrapping_add(1);
        let bytes = std_frame(255, 190, self.gcs.seq, m);
        if self.rng.passes() {
            self.node.receive(&bytes, self.now, &mut self.mem);
        } else {
            self.gcs.lost_to_node += 1;
        }
    }

    /// One millisecond: the node sends what it has, the link loses some, the model reads the rest.
    fn step(&mut self) {
        let mut frame = [0u8; FRAME_BUF];
        while let Some(n) = self
            .node
            .poll_transmit(self.now, &self.mem, &mut self.tel, &mut frame)
        {
            if !self.rng.passes() {
                self.gcs.lost_to_gcs += 1;
                continue;
            }
            let (_, m) = decode(&frame[..n]);
            match m {
                Std::PARAM_VALUE(p) => {
                    self.gcs.count = Some(p.param_count);
                    self.gcs.last_value_us = self.now;
                    let name = p.param_id.to_str().unwrap().to_string();
                    let bits = p.param_value.to_bits();
                    self.gcs
                        .echoes
                        .push((name.clone(), bits, p.param_type as u32, p.param_index));
                    self.gcs
                        .values
                        .insert(p.param_index, (name, bits, p.param_type as u32));
                }
                Std::COMMAND_ACK(a) => self.gcs.acks.push((a.command as u32, a.result as u32)),
                _ => {}
            }
        }
        self.now += 1_000;
    }

    fn run_until(&mut self, limit_us: u64, mut done: impl FnMut(&mut Self) -> bool) -> bool {
        while self.now < limit_us {
            if done(self) {
                return true;
            }
            self.step();
        }
        done(self)
    }
}

#[test]
fn a_list_of_300_arrives_complete_over_a_link_that_loses_a_fifth_of_the_frames_each_way() {
    let mut slots = [slot(105, 5_300, 5_000), slot(29, 20_700, 20_000)];
    let mut w = World::new(
        Node::new(Config::new(1, 1), &mut slots),
        Mem::numbered(300),
        Fake::new(&[(105, 4, true), (29, 1, true)]),
    );
    w.send(&list_req(1, 1));
    let mut last_action = 0u64;
    let ok = w.run_until(120_000_000, |w| {
        let total = w.gcs.count.map(usize::from);
        if total == Some(w.gcs.values.len()) {
            return true;
        }
        // The model: silence for 1 s -> ask again, for the list or for each missing index.
        if w.now - w.gcs.last_value_us.max(last_action) >= 1_000_000 {
            last_action = w.now;
            match total {
                None => w.send(&list_req(1, 1)),
                Some(n) => {
                    let missing: Vec<u16> = (0..n as u16)
                        .filter(|i| !w.gcs.values.contains_key(i))
                        .take(30)
                        .collect();
                    for i in missing {
                        w.send(&read_idx(i as i16));
                    }
                }
            }
        }
        false
    });
    assert!(
        ok,
        "only {} of 300 after {} s",
        w.gcs.values.len(),
        w.now / 1_000_000
    );
    assert!(
        w.gcs.lost_to_gcs > 50 && w.gcs.lost_to_node > 0,
        "the link did lose frames"
    );
    for (i, (name, bits, ty)) in &w.gcs.values {
        assert_eq!(name, &format!("P{i:03}"));
        let want = w.mem.items[usize::from(*i)].1;
        let (wb, wt) = match want {
            ParamValue::I32(v) => (v as u32, 6),
            ParamValue::F32(v) => (v.to_bits(), 9),
        };
        assert_eq!((*bits, *ty), (wb, wt), "{name}");
    }
    assert!(
        w.node.stats().dropped_reply_full > 0,
        "30 reads at once overflow the queue of 4 and are retried"
    );
}

#[test]
fn a_set_is_echoed_a_refused_set_echoes_the_old_value_and_an_unknown_name_gets_silence() {
    let mut slots = [];
    let mut mem = Mem::numbered(10);
    mem.items.push(("NOSET".into(), ParamValue::I32(42)));
    let mut w = World::new(
        Node::new(Config::new(1, 1), &mut slots),
        mem,
        Fake::default(),
    );

    // Each request is repeated every 500 ms until an echo of that name arrives (3 s at most).
    fn ask(w: &mut World<'_>, m: &Std, name: &str, tries: u32) -> Option<(u32, u32)> {
        w.gcs.echoes.clear();
        for _ in 0..tries {
            w.send(m);
            let until = w.now + 500_000;
            if w.run_until(until, |w| w.gcs.echoes.iter().any(|e| e.0 == name)) {
                break;
            }
        }
        w.gcs
            .echoes
            .iter()
            .find(|e| e.0 == name)
            .map(|e| (e.1, e.2))
    }

    let got = ask(&mut w, &set_i32("P002", -123_456), "P002", 20);
    assert_eq!(got, Some(((-123_456i32) as u32, 6)));
    assert_eq!(w.mem.value("P002"), ParamValue::I32(-123_456));

    let got = ask(&mut w, &set_f32("P003", 99.5), "P003", 20);
    assert_eq!(got, Some((99.5f32.to_bits(), 9)));

    let got = ask(&mut w, &set_i32("NOSET", 7), "NOSET", 20);
    assert_eq!(got, Some((42, 6)), "refused: the old value");
    assert_eq!(w.mem.value("NOSET"), ParamValue::I32(42));

    // Wrong type for the stored value: refused without calling set.
    let calls = w.mem.set_calls.len();
    let got = ask(&mut w, &set_f32("P002", 1.0), "P002", 20);
    assert_eq!(got, Some(((-123_456i32) as u32, 6)));
    assert_eq!(w.mem.set_calls.len(), calls);

    // An unknown name: five tries, silence.
    w.gcs.echoes.clear();
    let before = w.node.stats().dropped_unknown_param;
    for _ in 0..5 {
        w.send(&set_i32("NO_SUCH_PARAM", 1));
        let until = w.now + 500_000;
        w.run_until(until, |_| false);
    }
    assert!(w.gcs.echoes.is_empty());
    assert!(w.node.stats().dropped_unknown_param > before);
}

#[test]
fn link_down_restores_every_streams_default_rate() {
    let mut slots = [
        slot(105, 20_000, 5_000),
        slot(29, 100_000, 20_000),
        slot(24, 200_000, 100_000),
    ];
    let mut w = World::new(
        Node::new(Config::new(1, 1), &mut slots),
        Mem::new(&[]),
        Fake::new(&[(105, 1, true), (29, 1, true), (24, 1, true)]),
    );
    let set = |w: &mut World<'_>, id: f32, iv: f32| {
        w.gcs.acks.clear();
        for _ in 0..20 {
            w.send(&cmd(c::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, id, iv, 0.0));
            let until = w.now + 500_000;
            if w.run_until(until, |w| !w.gcs.acks.is_empty()) {
                break;
            }
        }
        assert_eq!(w.gcs.acks.first(), Some(&(511, 0)), "accepted");
    };
    set(&mut w, 105.0, 5_000.0);
    set(&mut w, 29.0, 1_000_000.0);
    set(&mut w, 24.0, -1.0);
    let count = |w: &mut World<'_>, id: u32, span: u64| {
        let from = w.now;
        let n0 = w.tel.count(id);
        let until = from + span;
        w.run_until(until, |_| false);
        w.tel.count(id) - n0
    };
    // Counted over 2 s from the raw telemetry calls (the link loss only touches the frames).
    assert_eq!(count(&mut w, 105, 2_000_000), 400);
    assert_eq!(count(&mut w, 29, 2_000_000), 2);
    assert_eq!(count(&mut w, 24, 2_000_000), 0);

    w.node.on_link_down();

    assert_eq!(count(&mut w, 105, 2_000_000), 100);
    assert_eq!(count(&mut w, 29, 2_000_000), 20);
    assert_eq!(count(&mut w, 24, 2_000_000), 10);
}
