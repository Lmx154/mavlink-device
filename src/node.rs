//! The sans-IO node: bytes and time in, frames out.

use crate::accept::Incoming as Handover;
use crate::config::{Config, StreamSlot};
use crate::deframer::{Deframer, Frame, Kind};
use crate::out::{Batch, FRAME_BUF, Full, Outgoing, Telemetry, Text, TextQueue, Texts};
use crate::params::ParamStore;
use marv_dialect::common::{
    COMMAND_LONG_DATA, HEARTBEAT_DATA, MavModeFlag, MavSeverity, MavState, PARAM_REQUEST_LIST_DATA,
    PARAM_REQUEST_READ_DATA, PARAM_SET_DATA, RADIO_STATUS_DATA, STATUSTEXT_DATA,
};
use marv_dialect::types::CharArray;
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, MessageData, calculate_crc};

mod service;

use service::{Incoming, List, REPLIES, Reply};

/// Heartbeat period, not changeable.
const HEARTBEAT_PERIOD_US: u64 = 1_000_000;

/// Slowdown bounds and steps of the RADIO_STATUS flow control, in milliseconds: ArduPilot's
/// `GCS_MAVLINK::handle_radio_status`.
const SLOWDOWN_CEILING_MS: u32 = 2000;
const SLOWDOWN_FLOOR_FAST_MS: u32 = 200;

/// The stream slowdown after a RADIO_STATUS with `txbuf` percent free, from the slowdown before.
pub(crate) fn next_slowdown_ms(slowdown_ms: u32, txbuf: u8) -> u32 {
    if txbuf < 20 && slowdown_ms < SLOWDOWN_CEILING_MS {
        slowdown_ms.saturating_add(60)
    } else if txbuf < 50 && slowdown_ms < SLOWDOWN_CEILING_MS {
        slowdown_ms.saturating_add(20)
    } else if txbuf > 95 && slowdown_ms > SLOWDOWN_FLOOR_FAST_MS {
        slowdown_ms.saturating_sub(40)
    } else if txbuf > 90 && slowdown_ms != 0 {
        slowdown_ms.saturating_sub(20)
    } else {
        slowdown_ms
    }
}

/// The system and component that sent a HEARTBEAT.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Peer {
    /// MAVLink system id.
    pub system_id: u8,
    /// MAVLink component id.
    pub component_id: u8,
}

/// Counters; they saturate and survive [`Node::on_link_down`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Stats {
    /// Bytes thrown away outside any frame.
    pub garbage_bytes: u32,
    /// MAVLink 1 frames (start byte 0xFE) dropped.
    pub dropped_mavlink1: u32,
    /// MAVLink 2 frames with any incompatibility flag (signed ones too) dropped.
    pub dropped_incompat: u32,
    /// Frames of a handled message with a wrong checksum dropped (also false starts that grew
    /// into a candidate frame).
    pub dropped_checksum: u32,
    /// Frames of a message the node does not handle dropped. Their checksum cannot be checked.
    pub dropped_unknown_message: u32,
    /// Valid frames carrying the node's own system id dropped.
    pub dropped_own_system: u32,
    /// Valid frames whose payload did not decode dropped (a COMMAND_LONG whose command number the
    /// dialect does not know, or a PARAM_SET with an unknown `param_type`, included).
    pub dropped_malformed: u32,
    /// Frames of a message on the accept list with a wrong checksum (computed with the listed
    /// CRC_EXTRA) dropped, false starts that grew into a candidate frame included.
    pub dropped_listed_checksum: u32,
    /// Valid HEARTBEATs received, from any sender.
    pub heartbeats_received: u32,
    /// PARAM_REQUEST_LIST, PARAM_REQUEST_READ, PARAM_SET and COMMAND_LONG frames dropped because
    /// their target system or component is neither the node's nor 0.
    pub dropped_not_for_us: u32,
    /// Requests dropped without a reply because they named no parameter: an unknown or invalid
    /// name, or an index out of range (PARAM_REQUEST_READ and PARAM_SET).
    pub dropped_unknown_param: u32,
    /// Requests dropped because the queue of 4 replies was full (the ground station retries).
    pub dropped_reply_full: u32,
    /// Frames sent, replies and list items included.
    pub frames_sent: u32,
    /// Texts refused because the queue was full.
    pub texts_dropped: u32,
    /// Sender of the last ground-station HEARTBEAT.
    pub last_gcs: Option<Peer>,
}

/// The sans-IO core of the device layer.
///
/// Feed it received bytes with [`receive`](Self::receive), ask it for frames with
/// [`poll_transmit`](Self::poll_transmit) and sleep until [`next_deadline_us`](Self::next_deadline_us).
/// It has no clock: every call takes `now_us` from the caller, and the caller keeps it
/// non-decreasing.
pub struct Node<'a> {
    config: Config,
    streams: &'a mut [StreamSlot],
    rx: Deframer,
    seq: u8,
    status: MavState,
    heartbeat_due_us: Option<u64>,
    gcs_seen_us: Option<u64>,
    texts: TextQueue,
    batch: Batch,
    replies: [Option<Reply>; REPLIES],
    reply_count: usize,
    list: Option<List>,
    stats: Stats,
    slowdown_ms: u32,
    #[cfg(feature = "_test-hooks")]
    slowdown_off: bool,
}

impl<'a> Node<'a> {
    /// A node with system status BOOT. Every slot of `streams` is reset to its default interval.
    pub fn new(config: Config, streams: &'a mut [StreamSlot]) -> Self {
        for slot in streams.iter_mut() {
            slot.reset();
        }
        Self {
            config,
            streams,
            rx: Deframer::new(),
            seq: 0,
            status: MavState::MAV_STATE_BOOT,
            heartbeat_due_us: None,
            gcs_seen_us: None,
            texts: TextQueue::new(),
            batch: Batch::new(),
            replies: [None; REPLIES],
            reply_count: 0,
            list: None,
            stats: Stats::default(),
            slowdown_ms: 0,
            #[cfg(feature = "_test-hooks")]
            slowdown_off: false,
        }
    }

    /// Takes bytes from the link, in any split. Garbage and frames the node does not accept are
    /// dropped and counted in [`Stats`]. Parameter requests and commands are acted on here (a write
    /// calls [`ParamStore::set`]) and their replies are queued for
    /// [`poll_transmit`](Self::poll_transmit). Messages of the accept list are dropped here; use
    /// [`receive_with`](Self::receive_with) to get them.
    pub fn receive(&mut self, bytes: &[u8], now_us: u64, params: &mut impl ParamStore) {
        self.receive_with(bytes, now_us, params, &mut |_| {});
    }

    /// As [`receive`](Self::receive), and every frame of a message on the accept list
    /// ([`Config::with_accept_list`]) is handed to `on_message`, once, from inside this call, when it is
    /// MAVLink 2 with incompatibility flags 0, its checksum is right for the listed CRC_EXTRA and
    /// its system id is not the node's own. The node does not interpret, queue or target-filter
    /// those messages. RADIO_STATUS is handled by the node whoever sends it (it drives the stream
    /// slowdown, see [`stream_slowdown_ms`](Self::stream_slowdown_ms)) and is also handed over when
    /// it is listed. `on_message` must not block; the [`crate::Incoming`] borrows the receive
    /// buffer and must not be kept. Entries of the list for other ids the node handles itself
    /// (HEARTBEAT, the parameter messages, COMMAND_LONG) have no effect.
    pub fn receive_with(
        &mut self,
        bytes: &[u8],
        now_us: u64,
        params: &mut impl ParamStore,
        on_message: &mut dyn FnMut(Handover<'_>),
    ) {
        let mut rest = bytes;
        while !rest.is_empty() {
            let taken = self.rx.feed(rest);
            if taken == 0 {
                self.rx.clear();
                continue;
            }
            rest = rest.get(taken..).unwrap_or(&[]);
            while let Some(frame) = self.rx.next_frame(&mut self.stats, &self.config.accept) {
                if matches!(frame.kind, Kind::RadioStatus | Kind::Listed) {
                    hand_over(
                        &mut self.stats,
                        &mut self.slowdown_ms,
                        &self.config,
                        &frame,
                        now_us,
                        on_message,
                    );
                    continue;
                }
                let sender = Peer {
                    system_id: frame.system_id,
                    component_id: frame.component_id,
                };
                let incoming = decode(&mut self.stats, self.config.system_id, &frame);
                if let Some(incoming) = incoming {
                    self.on_incoming(sender, incoming, now_us, params);
                }
            }
        }
    }

    /// The next frame to send, whole, written to the start of `frame`; `Some(len)`, or `None` when
    /// nothing is due. Priority: a due HEARTBEAT, replies (in order of arrival), queued texts, the
    /// next due item of a parameter list, the rest of the frames of the last sample, then due streams
    /// in slot order. Call it until it returns `None`.
    pub fn poll_transmit(
        &mut self,
        now_us: u64,
        params: &impl ParamStore,
        tel: &mut impl Telemetry,
        frame: &mut [u8; FRAME_BUF],
    ) -> Option<usize> {
        if self.heartbeat_due_us.is_none_or(|d| d <= now_us) {
            self.heartbeat_due_us = Some(match self.heartbeat_due_us {
                None => now_us.saturating_add(HEARTBEAT_PERIOD_US),
                Some(d) => d.saturating_add(
                    HEARTBEAT_PERIOD_US
                        .saturating_mul(now_us.saturating_sub(d) / HEARTBEAT_PERIOD_US + 1),
                ),
            });
            if let Some(status) = tel.status(now_us) {
                self.status = status;
            }
            return self.emit_heartbeat(frame);
        }
        if let Some(n) = self.poll_reply(now_us, params, tel, frame) {
            return Some(n);
        }
        if self.texts.has_room() {
            tel.texts(
                now_us,
                &mut Texts {
                    queue: &mut self.texts,
                    stats: &mut self.stats,
                },
            );
        }
        if let Some(text) = self.pop_text() {
            let statustext = STATUSTEXT_DATA {
                severity: text.severity,
                text: CharArray::new(text.bytes),
                id: 0,
                chunk_seq: 0,
            };
            return self.emit(&statustext, frame);
        }
        if let Some(n) = self.poll_list(now_us, params, frame) {
            return Some(n);
        }
        if let Some(n) = self.send_batch_frame(frame) {
            return Some(n);
        }
        let slowdown_us = self.slowdown_us();
        loop {
            let slot = self.streams.iter_mut().find(|s| s.is_due(now_us))?;
            let message_id = slot.stream().message_id;
            slot.advance(now_us, slowdown_us);
            if !sample_into(&mut self.batch, &self.config, tel, message_id, now_us) {
                continue;
            }
            if let Some(n) = self.send_batch_frame(frame) {
                return Some(n);
            }
        }
    }

    /// The earliest time at which [`poll_transmit`](Self::poll_transmit) could have something new:
    /// zero while replies, texts or pushed frames wait, else the next heartbeat, list item or stream.
    /// A value not after the current time means: poll now. An idle task can sleep until it.
    pub fn next_deadline_us(&self) -> u64 {
        if self.texts.len() > 0 || self.batch.is_pending() || self.reply_count > 0 {
            return 0;
        }
        let heartbeat = self.heartbeat_due_us.unwrap_or(0);
        let list = self.list.map_or(u64::MAX, |l| l.due_us);
        self.streams
            .iter()
            .filter_map(StreamSlot::deadline_us)
            .fold(heartbeat.min(list), u64::min)
    }

    /// The link went down: every stream returns to its default interval with a fresh schedule,
    /// queued texts, waiting replies, a parameter list in progress and pending frames are dropped,
    /// the receive buffer is cleared and the ground station is no longer present. The sequence counter, the heartbeat schedule, the status and
    /// [`Stats`] carry on.
    pub fn on_link_down(&mut self) {
        for slot in self.streams.iter_mut() {
            slot.reset();
        }
        self.texts.clear();
        self.batch.clear();
        self.replies = [None; REPLIES];
        self.reply_count = 0;
        self.list = None;
        self.rx.clear();
        self.gcs_seen_us = None;
        self.slowdown_ms = 0;
    }

    /// Sets the `system_status` of the next HEARTBEAT. The latest status given, by this or by
    /// [`Telemetry::status`], wins.
    pub fn set_status(&mut self, status: MavState) {
        self.status = status;
    }

    /// Queues a STATUSTEXT; at most 4 wait, shared with [`Telemetry::texts`]. Text beyond 50 bytes is cut, on a character boundary.
    /// A full queue returns [`Full`] and is counted in [`Stats::texts_dropped`].
    pub fn queue_text(&mut self, severity: MavSeverity, text: &str) -> Result<(), Full> {
        self.texts.push(&mut self.stats, severity, text)
    }

    /// Whether a ground-station HEARTBEAT arrived less than `gcs_timeout_us` before `now_us`. The
    /// node takes no action when it is lost.
    pub fn gcs_present(&self, now_us: u64) -> bool {
        self.gcs_seen_us
            .is_some_and(|t| now_us.saturating_sub(t) < self.config.gcs_timeout_us)
    }

    /// The counters.
    pub fn stats(&self) -> &Stats {
        &self.stats
    }

    /// The stream slowdown in milliseconds: it is added to the interval of every stream each time
    /// the stream is rescheduled (HEARTBEAT excepted). It starts at 0, follows the `txbuf` of the
    /// RADIO_STATUS messages received (see the README) and is reset to 0 by
    /// [`on_link_down`](Self::on_link_down). It stays 0 on a link that never carries RADIO_STATUS.
    pub fn stream_slowdown_ms(&self) -> u32 {
        self.slowdown_ms
    }

    /// Test hook (feature `_test-hooks`): with `off` the slowdown is still tracked and reported but
    /// no longer added to the stream intervals.
    #[cfg(feature = "_test-hooks")]
    pub fn set_slowdown_off(&mut self, off: bool) {
        self.slowdown_off = off;
    }

    #[cfg(feature = "_test-hooks")]
    fn slowdown_us(&self) -> u64 {
        if self.slowdown_off {
            0
        } else {
            u64::from(self.slowdown_ms) * 1000
        }
    }

    #[cfg(not(feature = "_test-hooks"))]
    fn slowdown_us(&self) -> u64 {
        u64::from(self.slowdown_ms) * 1000
    }

    #[cfg(test)]
    pub(crate) fn slots_mut(&mut self) -> &mut [StreamSlot] {
        self.streams
    }

    #[cfg(test)]
    pub(crate) fn rx_len(&self) -> usize {
        self.rx.len()
    }

    fn pop_text(&mut self) -> Option<Text> {
        self.texts.pop()
    }

    fn emit_heartbeat(&mut self, frame: &mut [u8; FRAME_BUF]) -> Option<usize> {
        let heartbeat = HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: self.config.vehicle_type,
            autopilot: self.config.autopilot,
            base_mode: MavModeFlag::empty(),
            system_status: self.status,
            mavlink_version: 3,
        };
        self.emit(&heartbeat, frame)
    }

    /// Builds a frame of the node's own with the next sequence number.
    fn emit<M: MessageData>(&mut self, m: &M, frame: &mut [u8; FRAME_BUF]) -> Option<usize> {
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message_data(
            MavHeader {
                system_id: u32::from(self.config.system_id),
                component_id: self.config.component_id,
                sequence: self.seq,
            },
            m,
        );
        let bytes = raw.raw_bytes();
        frame.get_mut(..bytes.len())?.copy_from_slice(bytes);
        self.seq = self.seq.wrapping_add(1);
        self.stats.frames_sent = self.stats.frames_sent.saturating_add(1);
        Some(bytes.len())
    }

    /// Sends the next pushed frame, stamping the sequence number and the checksum now so that the
    /// numbers stay consecutive in wire order.
    fn send_batch_frame(&mut self, frame: &mut [u8; FRAME_BUF]) -> Option<usize> {
        let pending = self.batch.peek()?;
        let n = pending.len();
        let body_end = n.checked_sub(2)?;
        frame.get_mut(..n)?.copy_from_slice(pending.bytes());
        *frame.get_mut(4)? = self.seq;
        let crc = calculate_crc(frame.get(1..body_end)?, pending.extra_crc());
        frame
            .get_mut(body_end..n)?
            .copy_from_slice(&crc.to_le_bytes());
        self.batch.advance();
        self.seq = self.seq.wrapping_add(1);
        self.stats.frames_sent = self.stats.frames_sent.saturating_add(1);
        Some(n)
    }
}

/// A frame of RADIO_STATUS or of a listed message: the slowdown for the first, the hook for either
/// when listed.
fn hand_over(
    stats: &mut Stats,
    slowdown_ms: &mut u32,
    config: &Config,
    frame: &Frame<'_>,
    now_us: u64,
    on_message: &mut dyn FnMut(Handover<'_>),
) {
    if frame.system_id == config.system_id {
        stats.dropped_own_system = stats.dropped_own_system.saturating_add(1);
        return;
    }
    if frame.kind == Kind::RadioStatus {
        match RADIO_STATUS_DATA::deser(MavlinkVersion::V2, frame.payload) {
            Ok(status) => *slowdown_ms = next_slowdown_ms(*slowdown_ms, status.txbuf),
            Err(_) => {
                stats.dropped_malformed = stats.dropped_malformed.saturating_add(1);
                return;
            }
        }
        if config.accept.extra_crc(frame.message_id).is_none() {
            return;
        }
    }
    on_message(Handover {
        system_id: frame.system_id,
        component_id: frame.component_id,
        message_id: frame.message_id,
        payload: frame.payload,
        now_us,
    });
}

fn decode(stats: &mut Stats, own_system_id: u8, frame: &Frame<'_>) -> Option<Incoming> {
    if frame.system_id == own_system_id {
        stats.dropped_own_system = stats.dropped_own_system.saturating_add(1);
        return None;
    }
    let v = MavlinkVersion::V2;
    let decoded = match frame.kind {
        Kind::Heartbeat => HEARTBEAT_DATA::deser(v, frame.payload).map(Incoming::Heartbeat),
        Kind::ParamRequestList => {
            PARAM_REQUEST_LIST_DATA::deser(v, frame.payload).map(Incoming::ParamRequestList)
        }
        Kind::ParamRequestRead => {
            PARAM_REQUEST_READ_DATA::deser(v, frame.payload).map(Incoming::ParamRequestRead)
        }
        Kind::ParamSet => PARAM_SET_DATA::deser(v, frame.payload).map(Incoming::ParamSet),
        Kind::CommandLong => COMMAND_LONG_DATA::deser(v, frame.payload).map(Incoming::CommandLong),
        Kind::RadioStatus | Kind::Listed => return None,
    };
    if decoded.is_err() {
        stats.dropped_malformed = stats.dropped_malformed.saturating_add(1);
    }
    decoded.ok()
}

/// Asks the hook for one sample of `message_id` into the (empty) batch; `false` when it sent nothing.
fn sample_into(
    batch: &mut Batch,
    config: &Config,
    tel: &mut impl Telemetry,
    message_id: u32,
    now_us: u64,
) -> bool {
    batch.clear();
    let mut out = Outgoing {
        batch,
        system_id: config.system_id,
        component_id: config.component_id,
    };
    if tel.sample(message_id, now_us, &mut out) {
        true
    } else {
        batch.clear();
        false
    }
}
