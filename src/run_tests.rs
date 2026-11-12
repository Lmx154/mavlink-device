//! Host tests of the async `run`: a fake reader, writer and delay on a made-up clock, polled by hand.

use crate::service_tests::{Mem, ack, cmd, list_req, pv, read_idx, slot};
use crate::tests::{Fake, decode, gcs_hb};
use crate::*;
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{ErrorKind, ErrorType, Read, Write};
use mavlink::dialects::common::MavMessage as Std;
use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::future::{Future, poll_fn};
use std::pin::pin;
use std::rc::Rc;
use std::task::{Context, Poll, Waker};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IoErr {
    Read,
    Write,
}

impl std::fmt::Display for IoErr {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for IoErr {}

impl embedded_io_async::Error for IoErr {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

enum Ev {
    Bytes(Vec<u8>),
    Err,
    Eof,
}

type Clock = Rc<Cell<u64>>;
type Writes = Rc<RefCell<Vec<(u64, Vec<u8>)>>>;

struct FakeRx {
    clock: Clock,
    script: VecDeque<(u64, Ev)>,
    cancelled: Rc<Cell<u32>>,
}

struct Guard(Rc<Cell<u32>>, bool);

impl Drop for Guard {
    fn drop(&mut self) {
        if !self.1 {
            self.0.set(self.0.get() + 1);
        }
    }
}

impl ErrorType for FakeRx {
    type Error = IoErr;
}

impl Read for FakeRx {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, IoErr> {
        let mut guard = Guard(self.cancelled.clone(), false);
        let out = poll_fn(|_| {
            let due = matches!(self.script.front(), Some((t, _)) if *t <= self.clock.get());
            if !due {
                return Poll::Pending;
            }
            match self.script.pop_front() {
                Some((_, Ev::Bytes(b))) => {
                    let n = b.len().min(buf.len());
                    buf[..n].copy_from_slice(&b[..n]);
                    if n < b.len() {
                        self.script.push_front((0, Ev::Bytes(b[n..].to_vec())));
                    }
                    Poll::Ready(Ok(n))
                }
                Some((_, Ev::Err)) => Poll::Ready(Err(IoErr::Read)),
                Some((_, Ev::Eof)) => Poll::Ready(Ok(0)),
                None => Poll::Pending,
            }
        })
        .await;
        guard.1 = true;
        out
    }
}

struct FakeTx {
    clock: Clock,
    writes: Writes,
    /// Bytes accepted per `write` call.
    chunk: usize,
    /// The write call (counted from 0) that fails.
    fail_at: Option<usize>,
}

impl ErrorType for FakeTx {
    type Error = IoErr;
}

impl Write for FakeTx {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, IoErr> {
        let mut w = self.writes.borrow_mut();
        if self.fail_at == Some(w.len()) {
            return Err(IoErr::Write);
        }
        let n = buf.len().min(self.chunk);
        w.push((self.clock.get(), buf[..n].to_vec()));
        Ok(n)
    }

    async fn flush(&mut self) -> Result<(), IoErr> {
        Ok(())
    }
}

struct FakeDelay {
    clock: Clock,
}

impl DelayNs for FakeDelay {
    async fn delay_ns(&mut self, ns: u32) {
        let target = self.clock.get() + u64::from(ns).div_ceil(1000);
        poll_fn(|_| {
            if self.clock.get() >= target {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }
}

struct Rig {
    clock: Clock,
    cancelled: Rc<Cell<u32>>,
    writes: Writes,
    rx: FakeRx,
    tx: FakeTx,
    delay: FakeDelay,
}

impl Rig {
    fn new(script: Vec<(u64, Ev)>) -> Self {
        let clock: Clock = Rc::new(Cell::new(0));
        let cancelled = Rc::new(Cell::new(0));
        let writes = Rc::new(RefCell::new(Vec::new()));
        Self {
            rx: FakeRx {
                clock: clock.clone(),
                script: script.into(),
                cancelled: cancelled.clone(),
            },
            tx: FakeTx {
                clock: clock.clone(),
                writes: writes.clone(),
                chunk: usize::MAX,
                fail_at: None,
            },
            delay: FakeDelay {
                clock: clock.clone(),
            },
            clock,
            cancelled,
            writes,
        }
    }

    /// Polls `run` by hand, advancing the clock 1 ms at a time, until it returns or `until_us`.
    fn drive(
        &mut self,
        node: &mut Node<'_>,
        mem: &mut Mem,
        tel: &mut Fake,
        until_us: u64,
    ) -> Option<RunError<IoErr, IoErr>> {
        let clock = self.clock.clone();
        let fut = node.run(
            &mut self.rx,
            &mut self.tx,
            &mut self.delay,
            move || clock.get(),
            mem,
            tel,
        );
        let mut fut = pin!(fut);
        let mut cx = Context::from_waker(Waker::noop());
        loop {
            if let Poll::Ready(e) = fut.as_mut().poll(&mut cx) {
                return Some(e);
            }
            if self.clock.get() >= until_us {
                return None;
            }
            self.clock.set(self.clock.get() + 1_000);
        }
    }

    /// The written bytes cut into whole frames: panics if a frame is cut or interleaved.
    fn frames(&self) -> Vec<(u64, Std)> {
        let mut out = Vec::new();
        let mut stream = Vec::new();
        let mut at = Vec::new();
        for (t, b) in self.writes.borrow().iter() {
            stream.extend(b);
            at.extend(std::iter::repeat_n(*t, b.len()));
        }
        let mut i = 0;
        while i < stream.len() {
            assert_eq!(stream[i], 0xFD, "a frame starts at byte {i}");
            let n = 12 + usize::from(stream[i + 1]);
            assert!(i + n <= stream.len(), "a frame is cut at the end");
            out.push((at[i], decode(&stream[i..i + n]).1));
            i += n;
        }
        out
    }
}

fn node<'a>(slots: &'a mut [StreamSlot]) -> Node<'a> {
    Node::new(Config::new(1, 1), slots)
}

#[test]
fn run_sends_a_heartbeat_once_a_second() {
    let mut rig = Rig::new(vec![]);
    let mut slots = [];
    let mut n = node(&mut slots);
    let mut mem = Mem::new(&[]);
    let mut tel = Fake::default();
    assert_eq!(rig.drive(&mut n, &mut mem, &mut tel, 5_500_000), None);
    let f = rig.frames();
    assert_eq!(f.len(), 6);
    for (k, (t, m)) in f.iter().enumerate() {
        assert!(matches!(m, Std::HEARTBEAT(_)));
        assert_eq!(*t, k as u64 * 1_000_000);
    }
    assert!(
        rig.cancelled.get() > 0,
        "the read was dropped each time the timer won"
    );
}

#[test]
fn run_feeds_what_it_reads_and_a_list_request_gives_the_whole_list() {
    let mut mem = Mem::numbered(40);
    let req = crate::tests::std_frame(255, 190, 0, &list_req(1, 1));
    // Delivered in two pieces to show that the split does not matter.
    let script = vec![
        (10_000, Ev::Bytes(req[..5].to_vec())),
        (11_000, Ev::Bytes(req[5..].to_vec())),
        (12_000, Ev::Bytes(gcs_hb(0))),
    ];
    let mut rig = Rig::new(script);
    let mut slots = [slot(29, 100_000, 20_000)];
    let mut n = node(&mut slots);
    let mut tel = Fake::new(&[(29, 1, true)]);
    assert_eq!(rig.drive(&mut n, &mut mem, &mut tel, 500_000), None);
    let f = rig.frames();
    let list: Vec<_> = f
        .iter()
        .filter_map(|(t, m)| {
            if let Std::PARAM_VALUE(p) = m {
                Some((*t, p.param_index))
            } else {
                None
            }
        })
        .collect();
    assert_eq!(list.len(), 40);
    assert!(list.iter().enumerate().all(|(k, l)| l.1 as usize == k));
    assert!(list.windows(2).all(|w| w[1].0 - w[0].0 >= 2_000));
    assert_eq!(
        list[0].0, 11_000,
        "the first item right after the request is whole"
    );
    assert!(n.gcs_present(500_000));
    assert_eq!(n.stats().heartbeats_received, 1);
    assert!(f.iter().any(|(_, m)| matches!(m, Std::SCALED_PRESSURE(_))));
}

#[test]
fn run_answers_a_command_and_a_read() {
    let mut mem = Mem::numbered(3);
    let script = vec![
        (
            5_000,
            Ev::Bytes(crate::tests::std_frame(255, 190, 0, &read_idx(1))),
        ),
        (
            6_000,
            Ev::Bytes(crate::tests::std_frame(
                255,
                190,
                1,
                &cmd(
                    mavlink::dialects::common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    1.0,
                    0.0,
                    0.0,
                ),
            )),
        ),
    ];
    let mut rig = Rig::new(script);
    let mut slots = [];
    let mut n = node(&mut slots);
    let mut tel = Fake::default();
    assert_eq!(rig.drive(&mut n, &mut mem, &mut tel, 50_000), None);
    let f = rig.frames();
    assert_eq!(f.len(), 3);
    assert_eq!(pv(&f[1].1).param_index, 1);
    assert_eq!(ack(&f[2].1).result as u32, 3);
}

#[test]
fn run_ends_on_a_read_error() {
    let mut rig = Rig::new(vec![(30_000, Ev::Err)]);
    let mut slots = [];
    let mut n = node(&mut slots);
    let r = rig.drive(&mut n, &mut Mem::new(&[]), &mut Fake::default(), 1_000_000);
    assert_eq!(r, Some(RunError::Read(IoErr::Read)));
    assert_eq!(rig.clock.get(), 30_000);
}

#[test]
fn run_ends_on_a_write_error() {
    let mut rig = Rig::new(vec![]);
    rig.tx.fail_at = Some(0);
    let mut slots = [];
    let mut n = node(&mut slots);
    let r = rig.drive(&mut n, &mut Mem::new(&[]), &mut Fake::default(), 1_000_000);
    assert_eq!(r, Some(RunError::Write(IoErr::Write)));
    // A failure on the second heartbeat, one second later.
    let mut rig = Rig::new(vec![]);
    rig.tx.fail_at = Some(1);
    let mut slots = [];
    let mut n = node(&mut slots);
    let r = rig.drive(&mut n, &mut Mem::new(&[]), &mut Fake::default(), 5_000_000);
    assert_eq!(r, Some(RunError::Write(IoErr::Write)));
    assert_eq!(rig.clock.get(), 1_000_000);
}

#[test]
fn run_ends_on_end_of_stream() {
    let mut rig = Rig::new(vec![(2_500_000, Ev::Eof)]);
    let mut slots = [];
    let mut n = node(&mut slots);
    let r = rig.drive(&mut n, &mut Mem::new(&[]), &mut Fake::default(), 5_000_000);
    assert_eq!(r, Some(RunError::EndOfStream));
    assert_eq!(rig.clock.get(), 2_500_000);
    assert_eq!(rig.frames().len(), 3, "heartbeats at 0, 1 s, 2 s");
}

#[test]
fn no_frame_is_ever_written_partially_even_when_the_writer_takes_a_few_bytes_per_call() {
    let mut mem = Mem::numbered(20);
    let req = crate::tests::std_frame(255, 190, 0, &list_req(0, 0));
    let mut rig = Rig::new(vec![(1_000, Ev::Bytes(req))]);
    rig.tx.chunk = 7;
    let mut slots = [slot(105, 10_000, 5_000), slot(29, 20_000, 20_000)];
    let mut n = node(&mut slots);
    let mut tel = Fake::new(&[(105, 4, true), (29, 1, true)]);
    assert_eq!(rig.drive(&mut n, &mut mem, &mut tel, 1_200_000), None);
    assert!(rig.writes.borrow().iter().all(|w| w.1.len() <= 7));
    let f = rig.frames();
    assert!(f.len() > 400, "{}", f.len());
    assert_eq!(
        f.iter()
            .filter(|(_, m)| matches!(m, Std::PARAM_VALUE(_)))
            .count(),
        20
    );
    // The sequence numbers are consecutive in write order.
    let seqs: Vec<u8> = rig
        .writes
        .borrow()
        .iter()
        .filter(|w| w.1[0] == 0xFD)
        .map(|w| w.1[4])
        .collect();
    assert!(seqs.windows(2).all(|w| w[1] == w[0].wrapping_add(1)));
}
