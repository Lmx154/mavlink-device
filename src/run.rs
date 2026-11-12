//! The async driver: a [`Node`] on a reader and a writer.

use crate::node::Node;
use crate::out::{FRAME_BUF, Telemetry};
use crate::params::ParamStore;
use core::future::{Future, poll_fn};
use core::pin::pin;
use core::task::Poll;
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{Read, Write};

/// Bytes read from the link at a time.
const READ_BUF: usize = 64;
/// Longest sleep of `run`: the telemetry hooks are polled at least this often.
const MAX_WAIT_US: u64 = 100_000;

/// Why [`Node::run`] returned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RunError<R, W> {
    /// The reader failed.
    Read(R),
    /// The writer failed; a frame may have been partly written.
    Write(W),
    /// The reader returned 0 bytes: the stream ended.
    EndOfStream,
}

enum Woke<E> {
    Read(Result<usize, E>),
    Timer,
}

impl Node<'_> {
    /// Drives the node forever: writes every frame [`poll_transmit`](Self::poll_transmit) has (each
    /// with `write_all`, so never interleaved), then waits for the first of bytes from `rx` (given to
    /// [`receive`](Self::receive)) and the time of [`next_deadline_us`](Self::next_deadline_us)
    /// (through `delay`). It returns only on a read error, a write error or a read of 0 bytes.
    ///
    /// While it is awaited the node cannot be touched: the firmware changes the reported status and
    /// sends texts through [`Telemetry::status`] and [`Telemetry::texts`]. It wakes at least every
    /// 100 ms so that a text offered by the hook goes out within about that time when idle.
    ///
    /// `rx.read` must be cancel-safe: when the timer wins, the read future is dropped, and bytes it
    /// had taken must not be lost. `now_us` must be non-decreasing and on the same clock as `delay`.
    /// Only one task may await this future. It does not call [`on_link_down`](Self::on_link_down):
    /// do that when the link goes down, before running again.
    pub async fn run<R: Read, W: Write, D: DelayNs>(
        &mut self,
        rx: &mut R,
        tx: &mut W,
        delay: &mut D,
        mut now_us: impl FnMut() -> u64,
        params: &mut impl ParamStore,
        tel: &mut impl Telemetry,
    ) -> RunError<R::Error, W::Error> {
        let mut frame = [0u8; FRAME_BUF];
        let mut buf = [0u8; READ_BUF];
        loop {
            while let Some(n) = self.poll_transmit(now_us(), &*params, tel, &mut frame) {
                let Some(bytes) = frame.get(..n) else {
                    continue;
                };
                if let Err(e) = tx.write_all(bytes).await {
                    return RunError::Write(e);
                }
            }
            let wait_us = (self.next_deadline_us().saturating_sub(now_us())).clamp(1, MAX_WAIT_US);
            let wait_us = u32::try_from(wait_us).unwrap_or(u32::MAX);
            let woke = {
                let mut read = pin!(rx.read(&mut buf));
                let mut timer = pin!(delay.delay_us(wait_us));
                poll_fn(|cx| {
                    if let Poll::Ready(r) = read.as_mut().poll(cx) {
                        Poll::Ready(Woke::Read(r))
                    } else if timer.as_mut().poll(cx).is_ready() {
                        Poll::Ready(Woke::Timer)
                    } else {
                        Poll::Pending
                    }
                })
                .await
            };
            match woke {
                Woke::Timer => {}
                Woke::Read(Err(e)) => return RunError::Read(e),
                Woke::Read(Ok(0)) => return RunError::EndOfStream,
                Woke::Read(Ok(n)) => {
                    if let Some(bytes) = buf.get(..n) {
                        self.receive(bytes, now_us(), params);
                    }
                }
            }
        }
    }
}
