//! Host tests of the parameter protocol and the commands. Every frame the node emits is decoded
//! with the std `mavlink` crate; every request is made by it.

use crate::dialect::MavSeverity;
use crate::tests::{Fake, decode, std_frame};
use crate::*;
use mavlink::dialects::common as c;
use mavlink::dialects::common::MavMessage as Std;

/// A parameter store in memory. "CLAMP" limits an I32 to 100 and says so, "NOSET" refuses.
pub(crate) struct Mem {
    pub items: Vec<(String, ParamValue)>,
    pub set_calls: Vec<(u16, ParamValue)>,
}

impl Mem {
    pub fn new(items: &[(&str, ParamValue)]) -> Self {
        Self {
            items: items.iter().map(|(n, v)| (n.to_string(), *v)).collect(),
            set_calls: Vec::new(),
        }
    }
    /// `n` parameters P000.., alternating I32 and F32.
    pub fn numbered(n: u16) -> Self {
        Self {
            items: (0..n)
                .map(|i| {
                    let v = if i % 2 == 0 {
                        ParamValue::I32(i32::from(i) * 10 - 5)
                    } else {
                        ParamValue::F32(f32::from(i) + 0.25)
                    };
                    (format!("P{i:03}"), v)
                })
                .collect(),
            set_calls: Vec::new(),
        }
    }
    pub fn value(&self, name: &str) -> ParamValue {
        self.items.iter().find(|i| i.0 == name).unwrap().1
    }
}

impl ParamStore for Mem {
    fn count(&self) -> u16 {
        self.items.len() as u16
    }
    fn name(&self, i: u16) -> Option<ParamName> {
        ParamName::new(&self.items.get(usize::from(i))?.0).ok()
    }
    fn get(&self, i: u16) -> Option<ParamValue> {
        Some(self.items.get(usize::from(i))?.1)
    }
    fn find(&self, n: &ParamName) -> Option<u16> {
        self.items
            .iter()
            .position(|i| i.0 == n.as_str())
            .map(|p| p as u16)
    }
    fn set(&mut self, i: u16, v: ParamValue) -> Result<ParamValue, ParamError> {
        self.set_calls.push((i, v));
        let item = self
            .items
            .get_mut(usize::from(i))
            .ok_or(ParamError::NotFound)?;
        match (item.0.as_str(), v) {
            ("NOSET", _) => Err(ParamError::Rejected),
            ("CLAMP", ParamValue::I32(x)) => {
                item.1 = ParamValue::I32(x.min(100));
                Ok(item.1)
            }
            _ => {
                item.1 = v;
                Ok(v)
            }
        }
    }
}

pub(crate) fn slot(message_id: u32, default_interval_us: i32, min_interval_us: u32) -> StreamSlot {
    StreamSlot::new(Stream {
        message_id,
        default_interval_us,
        min_interval_us,
    })
}

pub(crate) fn pullp(
    node: &mut Node<'_>,
    now: u64,
    tel: &mut Fake,
    params: &Mem,
) -> Option<(mavlink::MavHeader, Std, Vec<u8>)> {
    let mut frame = [0u8; FRAME_BUF];
    let n = node.poll_transmit(now, params, tel, &mut frame)?;
    let bytes = frame[..n].to_vec();
    let (h, m) = decode(&bytes);
    Some((h, m, bytes))
}

pub(crate) fn drainp(node: &mut Node<'_>, now: u64, tel: &mut Fake, params: &Mem) -> Vec<Std> {
    let mut v = Vec::new();
    while let Some((_, m, _)) = pullp(node, now, tel, params) {
        v.push(m);
        assert!(v.len() < 2000);
    }
    v
}

pub(crate) fn send(node: &mut Node<'_>, params: &mut Mem, m: &Std, now: u64) {
    node.receive(&std_frame(255, 190, 0, m), now, params);
}

pub(crate) fn list_req(sys: u8, comp: u8) -> Std {
    Std::PARAM_REQUEST_LIST(c::PARAM_REQUEST_LIST_DATA {
        target_system: sys,
        target_component: comp,
    })
}

pub(crate) fn read_idx(index: i16) -> Std {
    Std::PARAM_REQUEST_READ(c::PARAM_REQUEST_READ_DATA {
        param_index: index,
        target_system: 1,
        target_component: 1,
        param_id: "".into(),
    })
}

pub(crate) fn read_name(name: &str) -> Std {
    Std::PARAM_REQUEST_READ(c::PARAM_REQUEST_READ_DATA {
        param_index: -1,
        target_system: 1,
        target_component: 1,
        param_id: name.into(),
    })
}

pub(crate) fn set_msg(name: &str, value: f32, ty: c::MavParamType) -> Std {
    Std::PARAM_SET(c::PARAM_SET_DATA {
        param_value: value,
        target_system: 1,
        target_component: 1,
        param_id: name.into(),
        param_type: ty,
    })
}

pub(crate) fn set_i32(name: &str, v: i32) -> Std {
    set_msg(
        name,
        f32::from_bits(v as u32),
        c::MavParamType::MAV_PARAM_TYPE_INT32,
    )
}

pub(crate) fn set_f32(name: &str, v: f32) -> Std {
    set_msg(name, v, c::MavParamType::MAV_PARAM_TYPE_REAL32)
}

pub(crate) fn cmd(command: c::MavCmd, p1: f32, p2: f32, p3: f32) -> Std {
    Std::COMMAND_LONG(c::COMMAND_LONG_DATA {
        param1: p1,
        param2: p2,
        param3: p3,
        command,
        target_system: 1,
        target_component: 1,
        ..Default::default()
    })
}

pub(crate) fn pv(m: &Std) -> &c::PARAM_VALUE_DATA {
    match m {
        Std::PARAM_VALUE(p) => p,
        other => panic!("not a PARAM_VALUE: {other:?}"),
    }
}

pub(crate) fn ack(m: &Std) -> &c::COMMAND_ACK_DATA {
    match m {
        Std::COMMAND_ACK(a) => a,
        other => panic!("not a COMMAND_ACK: {other:?}"),
    }
}

fn first_ack(frames: Vec<Std>) -> c::COMMAND_ACK_DATA {
    frames
        .iter()
        .find_map(|m| {
            if let Std::COMMAND_ACK(a) = m {
                Some(a.clone())
            } else {
                None
            }
        })
        .expect("an ACK")
}

fn name_of(p: &c::PARAM_VALUE_DATA) -> String {
    p.param_id.to_str().unwrap().to_string()
}

fn heartbeat_only<'a>(mut node: Node<'a>, mem: &Mem) -> Node<'a> {
    let frames = drainp(&mut node, 0, &mut Fake::default(), mem);
    assert!(matches!(frames[..], [Std::HEARTBEAT(_)]));
    node
}

// ---- 1. addressing ----

#[test]
fn addressing_requests_are_acted_on_only_for_our_system_and_component_or_zero() {
    let mut mem = Mem::new(&[("A", ParamValue::I32(1))]);
    let kinds: [fn(u8, u8) -> Std; 4] = [
        list_req,
        |s, k| {
            let Std::PARAM_REQUEST_READ(mut r) = read_idx(0) else {
                unreachable!()
            };
            (r.target_system, r.target_component) = (s, k);
            Std::PARAM_REQUEST_READ(r)
        },
        |s, k| {
            let Std::PARAM_SET(mut r) = set_i32("A", 1) else {
                unreachable!()
            };
            (r.target_system, r.target_component) = (s, k);
            Std::PARAM_SET(r)
        },
        |s, k| {
            let Std::COMMAND_LONG(mut r) =
                cmd(c::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0, 0.0)
            else {
                unreachable!()
            };
            (r.target_system, r.target_component) = (s, k);
            Std::COMMAND_LONG(r)
        },
    ];
    for (k, make) in kinds.iter().enumerate() {
        for (sys, comp, ours) in [
            (1, 1, true),
            (0, 0, true),
            (1, 0, true),
            (0, 1, true),
            (2, 1, false),
            (1, 2, false),
            (255, 190, false),
            (0, 2, false),
            (2, 0, false),
        ] {
            let mut slots = [];
            let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
            send(&mut node, &mut mem, &make(sys, comp), 10);
            let frames = drainp(&mut node, 10, &mut Fake::default(), &mem);
            assert_eq!(
                frames.len(),
                usize::from(ours),
                "kind {k} target {sys}/{comp}"
            );
            assert_eq!(
                node.stats().dropped_not_for_us,
                u32::from(!ours),
                "kind {k} target {sys}/{comp}"
            );
        }
    }
}

// ---- 2. list ----

#[test]
fn list_sends_every_parameter_in_order_one_per_interval_with_count_and_index() {
    let mut mem = Mem::numbered(5);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    send(&mut node, &mut mem, &list_req(1, 1), 1_000);
    assert_eq!(node.next_deadline_us(), 1_000, "the first item is due now");
    for k in 0..5u64 {
        let t = 1_000 + k * 2_000;
        if k > 0 {
            assert!(
                pullp(&mut node, t - 1, &mut tel, &mem).is_none(),
                "item {k} early"
            );
            assert_eq!(node.next_deadline_us(), t, "the list's pace");
        }
        let (_, m, bytes) = pullp(&mut node, t, &mut tel, &mem).unwrap();
        let p = pv(&m);
        assert_eq!((p.param_index, p.param_count), (k as u16, 5));
        assert_eq!(name_of(p), format!("P{k:03}"));
        let want = if k % 2 == 0 { 6 } else { 9 };
        assert_eq!(p.param_type as u32, want);
        assert_eq!(bytes[10..14], mem.items[k as usize].1.to_wire_for_test());
        assert!(pullp(&mut node, t, &mut tel, &mem).is_none());
    }
    assert!(
        pullp(&mut node, 100_000, &mut tel, &mem).is_none(),
        "the list is over"
    );
    assert!(node.next_deadline_us() > 100_000);
}

impl ParamValue {
    fn to_wire_for_test(self) -> [u8; 4] {
        match self {
            Self::I32(v) => v.to_le_bytes(),
            Self::F32(v) => v.to_le_bytes(),
        }
    }
}

#[test]
fn the_list_interval_comes_from_the_config() {
    let mut mem = Mem::numbered(3);
    let mut slots = [];
    let cfg = Config::new(1, 1).with_param_list_interval_us(7_000);
    let mut node = Node::new(cfg, &mut slots);
    let mut tel = Fake::default();
    drainp(&mut node, 0, &mut tel, &mem);
    send(&mut node, &mut mem, &list_req(0, 0), 100);
    assert!(pullp(&mut node, 100, &mut tel, &mem).is_some());
    assert!(pullp(&mut node, 7_099, &mut tel, &mem).is_none());
    assert!(pullp(&mut node, 7_100, &mut tel, &mem).is_some());
}

#[test]
fn a_new_list_request_restarts_the_list_at_index_zero() {
    let mut mem = Mem::numbered(10);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    send(&mut node, &mut mem, &list_req(1, 1), 0);
    let mut idx = Vec::new();
    for k in 0..4u64 {
        idx.push(pv(&pullp(&mut node, k * 2_000, &mut tel, &mem).unwrap().1).param_index);
    }
    assert_eq!(idx, [0, 1, 2, 3]);
    send(&mut node, &mut mem, &list_req(1, 1), 8_000);
    for want in 0..10u16 {
        let t = 8_000 + u64::from(want) * 2_000;
        let (_, m, _) = pullp(&mut node, t, &mut tel, &mem).unwrap();
        assert_eq!(pv(&m).param_index, want);
    }
    assert!(pullp(&mut node, 900_000, &mut tel, &mem).is_none());
}

#[test]
fn a_store_without_parameters_answers_the_list_with_one_empty_value() {
    let mut mem = Mem::new(&[]);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    send(&mut node, &mut mem, &list_req(1, 1), 5);
    let (_, m, bytes) = pullp(&mut node, 5, &mut tel, &mem).unwrap();
    let p = pv(&m);
    assert_eq!(
        (
            p.param_count,
            p.param_index,
            p.param_type as u32,
            p.param_value.to_bits()
        ),
        (0, 65535, 9, 0)
    );
    assert_eq!(p.param_id.to_str().unwrap(), "");
    assert_eq!(&bytes[10..14], [0, 0, 0, 0]);
    assert!(
        pullp(&mut node, 900_000, &mut tel, &mem).is_none(),
        "exactly one"
    );
}

#[test]
fn a_name_of_exactly_16_characters_has_no_terminator_and_a_shorter_one_is_padded_with_zeros() {
    let long = "ABCDEFGHIJKLMNOP";
    let mut mem = Mem::new(&[(long, ParamValue::I32(1)), ("AB", ParamValue::I32(2))]);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    send(&mut node, &mut mem, &list_req(1, 1), 0);
    let (_, m, bytes) = pullp(&mut node, 0, &mut tel, &mem).unwrap();
    assert_eq!(name_of(pv(&m)), long);
    assert_eq!(&bytes[18..34], long.as_bytes());
    let (_, m, bytes) = pullp(&mut node, 2_000, &mut tel, &mem).unwrap();
    assert_eq!(name_of(pv(&m)), "AB");
    assert_eq!(&bytes[18..34], b"AB\0\0\0\0\0\0\0\0\0\0\0\0\0\0");
    // And a request by a 16-character name, unterminated, finds it.
    send(&mut node, &mut mem, &read_name(long), 3_000);
    let (_, m, _) = pullp(&mut node, 3_000, &mut tel, &mem).unwrap();
    assert_eq!(pv(&m).param_index, 0);
}

// ---- 3. encoding ----

#[test]
fn values_travel_bytewise_literal_bytes() {
    let mut mem = Mem::new(&[
        ("NEG", ParamValue::I32(-3)),
        ("BIG", ParamValue::I32(800)),
        ("FLT", ParamValue::F32(1.5)),
    ]);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    let want: [([u8; 4], u32); 3] = [
        ([0xFD, 0xFF, 0xFF, 0xFF], 6),
        ([0x20, 0x03, 0x00, 0x00], 6),
        ([0x00, 0x00, 0xC0, 0x3F], 9),
    ];
    for (i, (bytes4, ty)) in want.iter().enumerate() {
        send(&mut node, &mut mem, &read_idx(i as i16), 0);
        let (_, m, bytes) = pullp(&mut node, 0, &mut tel, &mem).unwrap();
        assert_eq!(&bytes[10..14], bytes4, "param {i}");
        assert_eq!(pv(&m).param_type as u32, *ty);
    }
    // A PARAM_SET with those bytes in the float field writes the same values.
    send(
        &mut node,
        &mut mem,
        &set_msg(
            "NEG",
            f32::from_le_bytes([0xFD, 0xFF, 0xFF, 0xFF]),
            c::MavParamType::MAV_PARAM_TYPE_INT32,
        ),
        0,
    );
    drainp(&mut node, 0, &mut tel, &mem);
    assert_eq!(mem.set_calls, [(0, ParamValue::I32(-3))]);
    send(
        &mut node,
        &mut mem,
        &set_msg(
            "BIG",
            f32::from_le_bytes([0x20, 0x03, 0, 0]),
            c::MavParamType::MAV_PARAM_TYPE_INT32,
        ),
        0,
    );
    drainp(&mut node, 0, &mut tel, &mem);
    assert_eq!(mem.set_calls[1], (1, ParamValue::I32(800)));
}

#[test]
fn an_i32_whose_bits_are_a_signalling_nan_comes_back_bit_exact() {
    let snan = 0x7FA0_0001u32;
    assert!(f32::from_bits(snan).is_nan());
    let mut mem = Mem::new(&[
        ("SNAN", ParamValue::I32(snan as i32)),
        ("OTHER", ParamValue::I32(0)),
    ]);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    send(&mut node, &mut mem, &read_idx(0), 0);
    let (_, m, bytes) = pullp(&mut node, 0, &mut tel, &mem).unwrap();
    assert_eq!(&bytes[10..14], snan.to_le_bytes());
    assert_eq!(pv(&m).param_value.to_bits(), snan);
    // The same pattern written by the host is stored as the same i32 and echoed.
    send(&mut node, &mut mem, &set_i32("OTHER", snan as i32), 0);
    let (_, m, bytes) = pullp(&mut node, 0, &mut tel, &mem).unwrap();
    assert_eq!(mem.value("OTHER"), ParamValue::I32(snan as i32));
    assert_eq!(&bytes[10..14], snan.to_le_bytes());
    assert_eq!(pv(&m).param_value.to_bits(), snan);
}

// ---- 4. read ----

#[test]
fn read_by_index_or_by_name_and_no_reply_for_an_unknown_name_an_invalid_name_or_a_bad_index() {
    let mut mem = Mem::numbered(4);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    send(&mut node, &mut mem, &read_idx(2), 0);
    send(&mut node, &mut mem, &read_name("P003"), 0);
    let frames = drainp(&mut node, 0, &mut tel, &mem);
    let got: Vec<_> = frames
        .iter()
        .map(|m| (name_of(pv(m)), pv(m).param_index, pv(m).param_count))
        .collect();
    assert_eq!(
        got,
        [("P002".to_string(), 2, 4), ("P003".to_string(), 3, 4)]
    );
    // An index wins over a name when the index is not -1.
    let Std::PARAM_REQUEST_READ(mut both) = read_name("P003") else {
        unreachable!()
    };
    both.param_index = 1;
    send(&mut node, &mut mem, &Std::PARAM_REQUEST_READ(both), 0);
    assert_eq!(pv(&drainp(&mut node, 0, &mut tel, &mem)[0]).param_index, 1);
    assert_eq!(node.stats().dropped_unknown_param, 0);
    for (i, m) in [
        read_idx(4),
        read_idx(-2),
        read_idx(i16::MAX),
        read_name("NOPE"),
        read_name("lower"),
        read_name(""),
    ]
    .iter()
    .enumerate()
    {
        send(&mut node, &mut mem, m, 0);
        assert!(drainp(&mut node, 0, &mut tel, &mem).is_empty(), "case {i}");
        assert_eq!(node.stats().dropped_unknown_param, i as u32 + 1);
    }
}

// ---- 5. set ----

#[test]
fn set_echoes_the_value_now_in_effect_with_index_and_count() {
    let mut mem = Mem::new(&[
        ("FIRST", ParamValue::F32(0.5)),
        ("SECOND", ParamValue::I32(7)),
        ("CLAMP", ParamValue::I32(1)),
    ]);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    send(&mut node, &mut mem, &set_i32("SECOND", -800), 0);
    let f = drainp(&mut node, 0, &mut tel, &mem);
    assert_eq!(f.len(), 1);
    let p = pv(&f[0]);
    assert_eq!(
        (
            name_of(p),
            p.param_index,
            p.param_count,
            p.param_type as u32
        ),
        ("SECOND".into(), 1, 3, 6)
    );
    assert_eq!(p.param_value.to_bits(), (-800i32) as u32);
    send(&mut node, &mut mem, &set_f32("FIRST", 2.5), 0);
    let f = drainp(&mut node, 0, &mut tel, &mem);
    assert_eq!(
        (pv(&f[0]).param_value, pv(&f[0]).param_type as u32),
        (2.5, 9)
    );
    assert_eq!(
        mem.set_calls,
        [(1, ParamValue::I32(-800)), (0, ParamValue::F32(2.5))]
    );
    // Ok(v) is the value in effect, not the one asked for.
    send(&mut node, &mut mem, &set_i32("CLAMP", 5_000), 0);
    let f = drainp(&mut node, 0, &mut tel, &mem);
    assert_eq!(pv(&f[0]).param_value.to_bits(), 100);
}

#[test]
fn a_refused_set_echoes_the_old_value_and_a_wrong_type_never_reaches_the_store() {
    let mut mem = Mem::new(&[
        ("NOSET", ParamValue::I32(42)),
        ("INT", ParamValue::I32(5)),
        ("FLT", ParamValue::F32(1.5)),
    ]);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    // The store says no: set() was called, the old value comes back.
    send(&mut node, &mut mem, &set_i32("NOSET", 1), 0);
    let f = drainp(&mut node, 0, &mut tel, &mem);
    assert_eq!(pv(&f[0]).param_value.to_bits(), 42);
    assert_eq!(mem.set_calls.len(), 1);
    // A type that is neither INT32 nor REAL32, or not the stored one: set() is not called.
    let wrong = [
        set_msg("INT", 1.0, c::MavParamType::MAV_PARAM_TYPE_UINT8),
        set_msg("INT", 1.0, c::MavParamType::MAV_PARAM_TYPE_UINT32),
        set_msg("FLT", 1.0, c::MavParamType::MAV_PARAM_TYPE_UINT8),
        set_msg("INT", 1.0, c::MavParamType::MAV_PARAM_TYPE_REAL32),
        set_i32("FLT", 3),
    ];
    for (i, m) in wrong.iter().enumerate() {
        send(&mut node, &mut mem, m, 0);
        let f = drainp(&mut node, 0, &mut tel, &mem);
        assert_eq!(f.len(), 1, "case {i}");
        let p = pv(&f[0]);
        let old = mem.items[p.param_index as usize].1;
        assert_eq!(
            p.param_value.to_le_bytes(),
            old.to_wire_for_test(),
            "case {i}"
        );
    }
    assert_eq!(
        mem.set_calls.len(),
        1,
        "set() was never called for a wrong type"
    );
    assert_eq!(mem.value("INT"), ParamValue::I32(5));
    assert_eq!(mem.value("FLT"), ParamValue::F32(1.5));
}

#[test]
fn a_set_of_an_unknown_or_invalid_name_gets_silence() {
    let mut mem = Mem::numbered(3);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    for (i, name) in ["NOPE", "lower", "", "TOOLONGNAME_ABCDEFG"]
        .iter()
        .enumerate()
    {
        send(&mut node, &mut mem, &set_i32(name, 1), 0);
        assert!(drainp(&mut node, 0, &mut tel, &mem).is_empty());
        assert_eq!(node.stats().dropped_unknown_param, i as u32 + 1);
    }
    assert!(mem.set_calls.is_empty());
}

// ---- 6. commands ----

fn node_with_table(slots: &mut [StreamSlot]) -> Node<'_> {
    let cfg = Config::new(1, 1).with_version(
        VersionInfo::new()
            .with_flight_sw_version(0x0102_0300)
            .with_middleware_sw_version(11)
            .with_os_sw_version(22)
            .with_board_version(33)
            .with_vendor_id(0x2E8A)
            .with_product_id(0x000A)
            .with_uid(0x0123_4567_89AB_CDEF),
    );
    Node::new(cfg, slots)
}

#[test]
fn request_message_autopilot_version_is_accepted_then_one_version_with_the_capabilities() {
    let mem = Mem::new(&[]);
    let mut mem2 = Mem::new(&[]);
    let mut slots = [];
    let mut node = heartbeat_only(node_with_table(&mut slots), &mem);
    send(
        &mut node,
        &mut mem2,
        &cmd(c::MavCmd::MAV_CMD_REQUEST_MESSAGE, 148.0, 0.0, 0.0),
        0,
    );
    let f = drainp(&mut node, 0, &mut Fake::default(), &mem);
    assert_eq!(f.len(), 2);
    let a = ack(&f[0]);
    assert_eq!((a.command as u32, a.result as u32), (512, 0));
    assert_eq!((a.target_system, a.target_component), (255, 190));
    let Std::AUTOPILOT_VERSION(v) = &f[1] else {
        panic!("{:?}", f[1])
    };
    assert_eq!(v.capabilities.bits(), 8192 | 16);
    assert_eq!(
        (
            v.flight_sw_version,
            v.middleware_sw_version,
            v.os_sw_version,
            v.board_version
        ),
        (0x0102_0300, 11, 22, 33)
    );
    assert_eq!(
        (v.vendor_id, v.product_id, v.uid),
        (0x2E8A, 0x000A, 0x0123_4567_89AB_CDEF)
    );
}

#[test]
fn version_info_is_zero_by_default() {
    let mem = Mem::new(&[]);
    let mut mem2 = Mem::new(&[]);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    send(
        &mut node,
        &mut mem2,
        &cmd(c::MavCmd::MAV_CMD_REQUEST_MESSAGE, 148.0, 0.0, 0.0),
        0,
    );
    let f = drainp(&mut node, 0, &mut Fake::default(), &mem);
    let Std::AUTOPILOT_VERSION(v) = &f[1] else {
        panic!()
    };
    assert_eq!(
        (v.flight_sw_version, v.vendor_id, v.product_id, v.uid),
        (0, 0, 0, 0)
    );
    assert_eq!(v.capabilities.bits(), 8192 | 16);
}

#[test]
fn request_message_heartbeat_gives_one_extra_copy_and_leaves_the_schedule_alone() {
    let mem = Mem::new(&[]);
    let mut mem2 = Mem::new(&[]);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    send(
        &mut node,
        &mut mem2,
        &cmd(c::MavCmd::MAV_CMD_REQUEST_MESSAGE, 0.0, 0.0, 0.0),
        500,
    );
    let f = drainp(&mut node, 500, &mut tel, &mem);
    assert_eq!(ack(&f[0]).result as u32, 0);
    assert!(matches!(f[1], Std::HEARTBEAT(_)));
    assert_eq!(f.len(), 2);
    assert!(pullp(&mut node, 999_999, &mut tel, &mem).is_none());
    assert!(pullp(&mut node, 1_000_000, &mut tel, &mem).is_some());
}

#[test]
fn request_message_of_a_stream_takes_one_extra_sample_with_all_its_frames_outside_its_schedule() {
    let mem = Mem::new(&[]);
    let mut mem2 = Mem::new(&[]);
    let mut slots = [
        slot(105, 100_000, 5_000),
        slot(29, 1_000_000, 20_000),
        slot(24, -1, 100_000),
    ];
    let mut node = Node::new(Config::new(1, 1), &mut slots);
    let mut tel = Fake::new(&[(105, 4, true), (29, 1, true), (24, 1, true)]);
    let first = drainp(&mut node, 0, &mut tel, &mem);
    assert_eq!(
        first.len(),
        1 + 4 + 1,
        "the heartbeat and the first samples of 105 and 29"
    );
    tel.calls.clear();
    send(
        &mut node,
        &mut mem2,
        &cmd(c::MavCmd::MAV_CMD_REQUEST_MESSAGE, 105.0, 0.0, 0.0),
        10_000,
    );
    let f = drainp(&mut node, 10_000, &mut tel, &mem);
    assert_eq!(ack(&f[0]).result as u32, 0);
    assert_eq!(
        f.len(),
        1 + 4,
        "the ACK, then every frame the sample pushed"
    );
    assert!(f[1..].iter().all(|m| matches!(m, Std::HIGHRES_IMU(_))));
    assert_eq!(tel.calls, [(105, 10_000)]);
    // The schedule is untouched: the next scheduled sample is still at 100 ms.
    assert!(pullp(&mut node, 99_999, &mut tel, &mem).is_none());
    assert_eq!(tel.calls.len(), 1);
    assert!(pullp(&mut node, 100_000, &mut tel, &mem).is_some());
    // A stream that is off can still be asked for.
    drainp(&mut node, 100_000, &mut tel, &mem);
    tel.calls.clear();
    send(
        &mut node,
        &mut mem2,
        &cmd(c::MavCmd::MAV_CMD_REQUEST_MESSAGE, 24.0, 0.0, 0.0),
        100_000,
    );
    let f = drainp(&mut node, 100_000, &mut tel, &mem);
    assert!(matches!(f[1], Std::GPS_RAW_INT(_)), "{f:?}");
    assert_eq!(tel.count(24), 1);
}

#[test]
fn request_message_of_anything_else_is_denied_and_nothing_follows() {
    let mem = Mem::new(&[]);
    let mut mem2 = Mem::new(&[]);
    let mut slots = [slot(29, 1_000_000, 20_000)];
    let mut node = Node::new(Config::new(1, 1), &mut slots);
    let mut tel = Fake::new(&[(29, 1, true)]);
    drainp(&mut node, 0, &mut tel, &mem);
    tel.calls.clear();
    for (i, p1) in [30.0f32, 253.0, -1.0, 0.5, f32::NAN, f32::INFINITY, 1e12]
        .iter()
        .enumerate()
    {
        send(
            &mut node,
            &mut mem2,
            &cmd(c::MavCmd::MAV_CMD_REQUEST_MESSAGE, *p1, 0.0, 0.0),
            1,
        );
        let f = drainp(&mut node, 1, &mut tel, &mem);
        assert_eq!(f.len(), 1, "case {i} ({p1})");
        assert_eq!(ack(&f[0]).result as u32, 2, "case {i} ({p1})");
    }
    assert!(tel.calls.is_empty());
}

fn interval_of(node: &mut Node<'_>, i: usize) -> i32 {
    node.slots_mut()[i].interval_us()
}

#[test]
fn set_message_interval_changes_restarts_and_restores() {
    let mem = Mem::new(&[]);
    let mut mem2 = Mem::new(&[]);
    let mut slots = [slot(105, 20_000, 5_000), slot(29, 100_000, 20_000)];
    let mut node = Node::new(Config::new(1, 1), &mut slots);
    let mut tel = Fake::new(&[(105, 1, true), (29, 1, true)]);
    drainp(&mut node, 0, &mut tel, &mem);
    let set = |node: &mut Node<'_>, mem2: &mut Mem, id: f32, iv: f32, now: u64| {
        send(
            node,
            mem2,
            &cmd(c::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, id, iv, 0.0),
            now,
        );
    };
    // A new interval: ACCEPTED, and the stream restarts from now (one sample at once).
    tel.calls.clear();
    set(&mut node, &mut mem2, 29.0, 250_000.0, 7_000);
    let f = drainp(&mut node, 7_000, &mut tel, &mem);
    assert_eq!(ack(&f[0]).command as u32, 511);
    assert_eq!(ack(&f[0]).result as u32, 0);
    assert_eq!(interval_of(&mut node, 1), 250_000);
    assert_eq!(tel.calls, [(29, 7_000)]);
    drainp(&mut node, 256_999, &mut tel, &mem);
    assert_eq!(tel.count(29), 1);
    drainp(&mut node, 257_000, &mut tel, &mem);
    assert_eq!(tel.count(29), 2, "7 000 + 250 000");
    // Off.
    set(&mut node, &mut mem2, 29.0, -1.0, 300_000);
    assert_eq!(
        first_ack(drainp(&mut node, 300_000, &mut tel, &mem)).result as u32,
        0
    );
    tel.calls.clear();
    drainp(&mut node, 5_000_000, &mut tel, &mem);
    assert_eq!(tel.count(29), 0);
    // Default again.
    set(&mut node, &mut mem2, 29.0, 0.0, 6_000_000);
    assert_eq!(
        first_ack(drainp(&mut node, 6_000_000, &mut tel, &mem)).result as u32,
        0
    );
    assert_eq!(interval_of(&mut node, 1), 100_000);
    assert_eq!(tel.count(29), 1);
    // Boundaries: the minimum and 10 s are allowed.
    for (iv, want) in [(20_000.0, 0), (10_000_000.0, 0)] {
        set(&mut node, &mut mem2, 29.0, iv, 7_000_000);
        assert_eq!(
            first_ack(drainp(&mut node, 7_000_000, &mut tel, &mem)).result as u32,
            want,
            "{iv}"
        );
        assert_eq!(interval_of(&mut node, 1), iv as i32);
    }
}

#[test]
fn set_message_interval_denials_change_nothing() {
    let mem = Mem::new(&[]);
    let mut mem2 = Mem::new(&[]);
    let mut slots = [slot(105, 20_000, 5_000), slot(29, 100_000, 20_000)];
    let mut node = Node::new(Config::new(1, 1), &mut slots);
    let mut tel = Fake::new(&[(105, 1, true), (29, 1, true)]);
    drainp(&mut node, 0, &mut tel, &mem);
    let cases: [(f32, f32, f32, &str); 11] = [
        (30.0, 100_000.0, 0.0, "not in the table"),
        (0.0, 1_000_000.0, 0.0, "heartbeat"),
        (29.0, 100_000.0, 1.0, "param3"),
        (29.0, 19_999.0, 0.0, "faster than the minimum"),
        (29.0, 10_000_001.0, 0.0, "slower than 10 s"),
        (29.0, -2.0, 0.0, "negative"),
        (29.0, f32::NAN, 0.0, "nan"),
        (29.0, 0.4, 0.0, "below one microsecond"),
        (29.5, 100_000.0, 0.0, "fractional id"),
        (-1.0, 100_000.0, 0.0, "negative id"),
        (148.0, 100_000.0, 0.0, "autopilot version is not a stream"),
    ];
    for (id, iv, p3, why) in cases {
        send(
            &mut node,
            &mut mem2,
            &cmd(c::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, id, iv, p3),
            1,
        );
        let f = drainp(&mut node, 1, &mut tel, &mem);
        assert_eq!(f.len(), 1, "{why}");
        assert_eq!(ack(&f[0]).result as u32, 2, "{why}");
    }
    assert_eq!(
        (interval_of(&mut node, 0), interval_of(&mut node, 1)),
        (20_000, 100_000)
    );
    assert_eq!(tel.calls.len(), 2, "no restart: no extra sample");
}

#[test]
fn every_other_command_is_unsupported_and_nothing_else_happens() {
    let mem = Mem::new(&[("A", ParamValue::I32(1))]);
    let mut mem2 = Mem::new(&[("A", ParamValue::I32(1))]);
    let mut slots = [slot(29, 100_000, 20_000)];
    let mut node = Node::new(Config::new(1, 1), &mut slots);
    let mut tel = Fake::new(&[(29, 1, true)]);
    drainp(&mut node, 0, &mut tel, &mem);
    tel.calls.clear();
    let before = *node.stats();
    for (command, number) in [
        (c::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 400),
        (c::MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 246),
        (c::MavCmd::MAV_CMD_DO_SET_MODE, 176),
        (c::MavCmd::MAV_CMD_NAV_WAYPOINT, 16),
    ] {
        send(&mut node, &mut mem2, &cmd(command, 1.0, 1.0, 1.0), 5);
        let f = drainp(&mut node, 5, &mut tel, &mem);
        assert_eq!(f.len(), 1, "command {number}: one ACK and nothing else");
        let a = ack(&f[0]);
        assert_eq!((a.command as u32, a.result as u32), (number, 3));
        assert_eq!((a.target_system, a.target_component), (255, 190));
    }
    assert!(tel.calls.is_empty());
    assert!(mem2.set_calls.is_empty());
    assert_eq!(interval_of(&mut node, 0), 100_000);
    let after = *node.stats();
    assert_eq!(after.dropped_malformed, before.dropped_malformed);
    // Presence of the ground station does not matter and is not acted on.
    assert!(!node.gcs_present(5));
}

#[test]
fn a_command_the_dialect_does_not_know_gets_no_reply_and_is_counted() {
    let mem = Mem::new(&[]);
    let mut mem2 = Mem::new(&[]);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut f = std_frame(
        255,
        190,
        0,
        &cmd(c::MavCmd::MAV_CMD_NAV_WAYPOINT, 1.0, 0.0, 0.0),
    );
    // The command number sits after the seven floats of the payload; 12345 is no MAV_CMD.
    f[10 + 28..10 + 30].copy_from_slice(&12345u16.to_le_bytes());
    let n = f.len();
    let crc = marv_dialect::calculate_crc(
        &f[1..n - 2],
        <marv_dialect::common::COMMAND_LONG_DATA as marv_dialect::MessageData>::EXTRA_CRC,
    );
    f[n - 2..].copy_from_slice(&crc.to_le_bytes());
    node.receive(&f, 0, &mut mem2);
    assert!(drainp(&mut node, 0, &mut Fake::default(), &mem).is_empty());
    assert_eq!(node.stats().dropped_malformed, 1);
    assert_eq!(node.stats().dropped_checksum, 0);
}

// ---- 7. replies, order, deadline ----

#[test]
fn at_most_four_replies_wait_and_the_rest_are_dropped_and_counted() {
    let mem = Mem::numbered(10);
    let mut mem2 = Mem::numbered(10);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    for i in 0..6 {
        send(&mut node, &mut mem2, &read_idx(i), 0);
    }
    assert_eq!(node.stats().dropped_reply_full, 2);
    let f = drainp(&mut node, 0, &mut tel, &mem);
    let idx: Vec<_> = f.iter().map(|m| pv(m).param_index).collect();
    assert_eq!(
        idx,
        [0, 1, 2, 3],
        "in order of arrival; 4 and 5 were dropped"
    );
    // Room again.
    send(&mut node, &mut mem2, &read_idx(4), 0);
    assert_eq!(drainp(&mut node, 0, &mut tel, &mem).len(), 1);
    // A set is not applied when there is no room for its echo.
    for i in 0..4 {
        send(&mut node, &mut mem2, &read_idx(i), 0);
    }
    send(&mut node, &mut mem2, &set_i32("P000", 77), 0);
    send(
        &mut node,
        &mut mem2,
        &cmd(c::MavCmd::MAV_CMD_REQUEST_MESSAGE, 148.0, 0.0, 0.0),
        0,
    );
    assert_eq!(node.stats().dropped_reply_full, 4);
    assert!(
        mem2.set_calls.is_empty(),
        "a dropped request is not acted on"
    );
    assert_eq!(drainp(&mut node, 0, &mut tel, &mem).len(), 4);
}

#[test]
fn a_command_with_a_follow_up_takes_one_place_in_the_queue() {
    let mem = Mem::new(&[]);
    let mut mem2 = Mem::new(&[]);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    for _ in 0..4 {
        send(
            &mut node,
            &mut mem2,
            &cmd(c::MavCmd::MAV_CMD_REQUEST_MESSAGE, 148.0, 0.0, 0.0),
            0,
        );
    }
    send(
        &mut node,
        &mut mem2,
        &cmd(c::MavCmd::MAV_CMD_REQUEST_MESSAGE, 148.0, 0.0, 0.0),
        0,
    );
    assert_eq!(node.stats().dropped_reply_full, 1);
    let f = drainp(&mut node, 0, &mut Fake::default(), &mem);
    assert_eq!(f.len(), 8, "four ACKs, four versions");
    let kinds: Vec<_> = f.iter().map(|m| matches!(m, Std::COMMAND_ACK(_))).collect();
    assert_eq!(kinds, [true, false, true, false, true, false, true, false]);
}

#[test]
fn order_is_heartbeat_replies_texts_list_then_streams() {
    let mem = Mem::numbered(3);
    let mut mem2 = Mem::numbered(3);
    let mut slots = [slot(29, 100_000, 20_000)];
    let mut node = Node::new(Config::new(1, 1), &mut slots);
    let mut tel = Fake::new(&[(29, 1, true)]);
    drainp(&mut node, 0, &mut tel, &mem);
    let t = 1_000_000;
    node.queue_text(MavSeverity::MAV_SEVERITY_INFO, "text")
        .unwrap();
    send(&mut node, &mut mem2, &list_req(1, 1), t);
    send(&mut node, &mut mem2, &read_idx(2), t);
    let f = drainp(&mut node, t, &mut tel, &mem);
    let kinds: Vec<String> = f
        .iter()
        .map(|m| match m {
            Std::HEARTBEAT(_) => "hb".into(),
            Std::PARAM_VALUE(p) => format!("pv{}", p.param_index),
            Std::STATUSTEXT(_) => "text".into(),
            Std::SCALED_PRESSURE(_) => "stream".into(),
            other => panic!("{other:?}"),
        })
        .collect();
    assert_eq!(kinds, ["hb", "pv2", "text", "pv0", "stream"]);
    let f = drainp(&mut node, t + 2_000, &mut tel, &mem);
    assert_eq!(f.len(), 1);
    assert_eq!(pv(&f[0]).param_index, 1);
}

#[test]
fn the_deadline_is_now_while_a_reply_waits_and_the_list_pace_afterwards() {
    let mem = Mem::numbered(3);
    let mut mem2 = Mem::numbered(3);
    let mut slots = [];
    let mut node = Node::new(Config::new(1, 1), &mut slots);
    let mut tel = Fake::default();
    drainp(&mut node, 0, &mut tel, &mem);
    assert_eq!(node.next_deadline_us(), 1_000_000);
    send(&mut node, &mut mem2, &read_idx(0), 50);
    assert_eq!(node.next_deadline_us(), 0, "a reply is waiting");
    drainp(&mut node, 50, &mut tel, &mem);
    assert_eq!(node.next_deadline_us(), 1_000_000);
    send(&mut node, &mut mem2, &list_req(1, 1), 100);
    assert_eq!(node.next_deadline_us(), 100);
    assert!(pullp(&mut node, 100, &mut tel, &mem).is_some());
    assert_eq!(node.next_deadline_us(), 2_100);
    assert!(pullp(&mut node, 2_100, &mut tel, &mem).is_some());
    assert_eq!(node.next_deadline_us(), 4_100);
}

#[test]
fn a_heartbeat_waits_for_at_most_one_frame_during_a_list_of_300_with_every_stream_at_its_fastest() {
    let mut mem = Mem::numbered(300);
    let mut slots = [
        slot(105, 5_000, 5_000),
        slot(29, 20_000, 20_000),
        slot(24, 100_000, 100_000),
    ];
    let mut node = Node::new(Config::new(1, 1), &mut slots);
    let mut tel = Fake::new(&[(105, 4, true), (29, 1, true), (24, 1, true)]);
    const FRAME_US: u64 = 1_000;
    let mut t = 0u64;
    let mut heartbeats = Vec::new();
    let mut listed = Vec::new();
    let mut next_request = 0u64;
    while t < 5_000_000 {
        if t >= next_request {
            send(&mut node, &mut mem, &list_req(1, 1), t);
            let _ = node.queue_text(MavSeverity::MAV_SEVERITY_INFO, "load");
            send(&mut node, &mut mem, &read_idx(299), t);
            next_request += 1_500_000;
        }
        match pullp(&mut node, t, &mut tel, &mem) {
            Some((_, Std::HEARTBEAT(_), _)) => {
                heartbeats.push(t);
                t += FRAME_US;
            }
            Some((_, Std::PARAM_VALUE(p), _)) => {
                listed.push((t, p.param_index));
                t += FRAME_US;
            }
            Some(_) => t += FRAME_US,
            None => t += 100,
        }
    }
    assert!(heartbeats.len() >= 5, "{heartbeats:?}");
    for (k, &sent) in heartbeats.iter().enumerate() {
        let due = k as u64 * 1_000_000;
        assert!(
            sent >= due && sent - due <= FRAME_US,
            "heartbeat {k}: due {due}, sent {sent}"
        );
    }
    assert_eq!(listed[0].1, 299, "the read reply goes before the list");
    assert!(
        (0..300u16).all(|i| listed.iter().any(|l| l.1 == i)),
        "the whole list arrived"
    );
    // Within one list run, items arrive in index order and at least 2 ms apart.
    let run: Vec<_> = listed
        .iter()
        .filter(|l| l.0 < 1_400_000 && l.1 != 299)
        .collect();
    assert!(run.len() >= 290);
    for w in run.windows(2) {
        assert!(w[1].0 - w[0].0 >= 2_000, "{} {}", w[0].0, w[1].0);
        assert_eq!(w[1].1, w[0].1 + 1);
    }
}

// ---- 8. link down ----

#[test]
fn link_down_drops_waiting_replies_and_the_list_in_progress() {
    let mem = Mem::numbered(50);
    let mut mem2 = Mem::numbered(50);
    let mut slots = [];
    let mut node = heartbeat_only(Node::new(Config::new(1, 1), &mut slots), &mem);
    let mut tel = Fake::default();
    send(&mut node, &mut mem2, &list_req(1, 1), 0);
    assert!(pullp(&mut node, 0, &mut tel, &mem).is_some());
    send(&mut node, &mut mem2, &read_idx(3), 0);
    send(
        &mut node,
        &mut mem2,
        &cmd(c::MavCmd::MAV_CMD_REQUEST_MESSAGE, 148.0, 0.0, 0.0),
        0,
    );
    node.on_link_down();
    assert!(drainp(&mut node, 0, &mut tel, &mem).is_empty());
    assert!(
        drainp(&mut node, 500_000, &mut tel, &mem).is_empty(),
        "the list does not go on"
    );
    assert_eq!(node.next_deadline_us(), 1_000_000);
    // And room is back.
    for i in 0..4 {
        send(&mut node, &mut mem2, &read_idx(i), 600_000);
    }
    assert_eq!(drainp(&mut node, 600_000, &mut tel, &mem).len(), 4);
}
