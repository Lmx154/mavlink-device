//! MAVLink 2 deframer with its own bounded buffer.

use crate::node::Stats;
use marv_dialect::common::{
    COMMAND_LONG_DATA, HEARTBEAT_DATA, PARAM_REQUEST_LIST_DATA, PARAM_REQUEST_READ_DATA,
    PARAM_SET_DATA,
};
use marv_dialect::{MessageData, calculate_crc};

/// Receive buffer: the longest candidate frame is 12 + 255 = 267 bytes (a candidate with an
/// incompatibility flag is dropped on sight, so a signature never needs room).
pub(crate) const RX_BUF: usize = 280;

const STX_V1: u8 = 0xFE;
const STX_V2: u8 = 0xFD;
/// STX plus the nine header bytes.
const HEADER_LEN: usize = 10;
/// Header, checksum.
const OVERHEAD: usize = HEADER_LEN + 2;

/// What the node does with a valid frame of a message it knows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Kind {
    Heartbeat,
    ParamRequestList,
    ParamRequestRead,
    ParamSet,
    CommandLong,
}

fn classify(id: u32) -> Option<(Kind, u8)> {
    if id == HEARTBEAT_DATA::ID {
        Some((Kind::Heartbeat, HEARTBEAT_DATA::EXTRA_CRC))
    } else if id == PARAM_REQUEST_LIST_DATA::ID {
        Some((Kind::ParamRequestList, PARAM_REQUEST_LIST_DATA::EXTRA_CRC))
    } else if id == PARAM_REQUEST_READ_DATA::ID {
        Some((Kind::ParamRequestRead, PARAM_REQUEST_READ_DATA::EXTRA_CRC))
    } else if id == PARAM_SET_DATA::ID {
        Some((Kind::ParamSet, PARAM_SET_DATA::EXTRA_CRC))
    } else if id == COMMAND_LONG_DATA::ID {
        Some((Kind::CommandLong, COMMAND_LONG_DATA::EXTRA_CRC))
    } else {
        None
    }
}

/// A frame that passed every check except the sender's system id.
pub(crate) struct Frame<'a> {
    pub system_id: u8,
    pub component_id: u8,
    pub kind: Kind,
    pub payload: &'a [u8],
}

pub(crate) struct Deframer {
    buf: [u8; RX_BUF],
    len: usize,
    /// Bytes of the frame last handed out, dropped at the next call.
    consume: usize,
}

struct Found {
    total: usize,
    system_id: u8,
    component_id: u8,
    kind: Kind,
    payload_len: usize,
}

impl Deframer {
    pub(crate) const fn new() -> Self {
        Self {
            buf: [0; RX_BUF],
            len: 0,
            consume: 0,
        }
    }

    #[cfg(test)]
    pub(crate) fn len(&self) -> usize {
        self.len
    }

    pub(crate) fn clear(&mut self) {
        self.len = 0;
        self.consume = 0;
    }

    fn drop_front(&mut self, n: usize) {
        let n = n.min(self.len);
        self.buf.copy_within(n..self.len, 0);
        self.len -= n;
    }

    fn release(&mut self) {
        let n = core::mem::take(&mut self.consume);
        self.drop_front(n);
    }

    /// Copies as much of `bytes` as fits; returns how many bytes it took.
    pub(crate) fn feed(&mut self, bytes: &[u8]) -> usize {
        self.release();
        let n = (RX_BUF - self.len).min(bytes.len());
        if let (Some(dst), Some(src)) = (self.buf.get_mut(self.len..self.len + n), bytes.get(..n)) {
            dst.copy_from_slice(src);
            self.len += n;
            n
        } else {
            0
        }
    }

    /// The next frame whole in the buffer, counting what it throws away. `None` when only part of
    /// a candidate is left (it is always shorter than the buffer).
    pub(crate) fn next_frame(&mut self, stats: &mut Stats) -> Option<Frame<'_>> {
        self.release();
        let found = self.find(stats)?;
        self.consume = found.total;
        let payload = self.buf.get(HEADER_LEN..HEADER_LEN + found.payload_len)?;
        Some(Frame {
            system_id: found.system_id,
            component_id: found.component_id,
            kind: found.kind,
            payload,
        })
    }

    fn find(&mut self, stats: &mut Stats) -> Option<Found> {
        loop {
            let live = self.buf.get(..self.len)?;
            let start = live
                .iter()
                .position(|&b| b == STX_V1 || b == STX_V2)
                .unwrap_or(live.len());
            if start > 0 {
                stats.garbage_bytes = stats.garbage_bytes.saturating_add(start as u32);
                self.drop_front(start);
            }
            let live = self.buf.get(..self.len)?;
            let stx = *live.first()?;
            if stx == STX_V1 {
                stats.dropped_mavlink1 = stats.dropped_mavlink1.saturating_add(1);
                self.drop_front(1);
                continue;
            }
            let flags = *live.get(2)?;
            if flags != 0 {
                stats.dropped_incompat = stats.dropped_incompat.saturating_add(1);
                self.drop_front(1);
                continue;
            }
            let (Some(&id0), Some(&id1), Some(&id2)) = (live.get(7), live.get(8), live.get(9))
            else {
                return None;
            };
            let id = u32::from_le_bytes([id0, id1, id2, 0]);
            let Some((kind, extra_crc)) = classify(id) else {
                stats.dropped_unknown_message = stats.dropped_unknown_message.saturating_add(1);
                self.drop_front(1);
                continue;
            };
            let payload_len = usize::from(*live.get(1)?);
            let total = OVERHEAD + payload_len;
            if live.len() < total {
                return None;
            }
            let crc_input = live.get(1..HEADER_LEN + payload_len)?;
            let (Some(&c0), Some(&c1)) = (
                live.get(HEADER_LEN + payload_len),
                live.get(HEADER_LEN + payload_len + 1),
            ) else {
                return None;
            };
            if calculate_crc(crc_input, extra_crc) != u16::from_le_bytes([c0, c1]) {
                stats.dropped_checksum = stats.dropped_checksum.saturating_add(1);
                self.drop_front(1);
                continue;
            }
            return Some(Found {
                total,
                system_id: *live.get(5)?,
                component_id: *live.get(6)?,
                kind,
                payload_len,
            });
        }
    }
}
