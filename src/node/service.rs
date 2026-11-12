//! Incoming heartbeats, parameter requests and commands, and the replies they queue.

use super::{Node, Peer, sample_into};
use crate::out::{FRAME_BUF, Telemetry};
use crate::params::{ParamName, ParamStore, ParamValue};
use marv_dialect::MessageData;
use marv_dialect::common::{
    AUTOPILOT_VERSION_DATA, COMMAND_ACK_DATA, COMMAND_LONG_DATA, HEARTBEAT_DATA, MavCmd,
    MavParamType, MavProtocolCapability, MavResult, MavType, PARAM_REQUEST_LIST_DATA,
    PARAM_REQUEST_READ_DATA, PARAM_SET_DATA, PARAM_VALUE_DATA,
};
use marv_dialect::types::CharArray;

/// Replies that can wait to be sent.
pub(super) const REPLIES: usize = 4;
/// Longest interval a ground station may ask for, in microseconds.
const MAX_INTERVAL_US: f32 = 10_000_000.0;
/// Message ids are 24 bits.
const MAX_MESSAGE_ID: u32 = 0xFF_FFFF;

pub(super) enum Incoming {
    Heartbeat(HEARTBEAT_DATA),
    ParamRequestList(PARAM_REQUEST_LIST_DATA),
    ParamRequestRead(PARAM_REQUEST_READ_DATA),
    ParamSet(PARAM_SET_DATA),
    CommandLong(COMMAND_LONG_DATA),
}

/// A message sent on request, after the ACK of the command that asked for it.
#[derive(Clone, Copy)]
pub(super) enum Requested {
    Version,
    Heartbeat,
    Stream(u32),
}

#[derive(Clone, Copy)]
pub(super) enum Reply {
    /// A PARAM_VALUE of parameter `index` carrying the value that was in effect when it was queued.
    Param {
        index: u16,
        value: ParamValue,
    },
    /// A COMMAND_ACK; the entry becomes `then` once it is sent.
    Ack {
        command: MavCmd,
        result: MavResult,
        target: Peer,
        then: Option<Requested>,
    },
    Requested(Requested),
}

/// A parameter list in progress.
#[derive(Clone, Copy)]
pub(super) struct List {
    next: u16,
    pub(super) due_us: u64,
}

fn name_of(id: &[u8; 16]) -> Option<ParamName> {
    let end = id.iter().position(|&b| b == 0).unwrap_or(id.len());
    ParamName::from_bytes(id.get(..end)?).ok()
}

/// A message id from a command parameter: a whole number of at most 24 bits.
fn message_id(v: f32) -> Option<u32> {
    let id = v as u32;
    (id as f32 == v && id <= MAX_MESSAGE_ID).then_some(id)
}

fn param_value(name: &ParamName, value: ParamValue, index: u16, count: u16) -> PARAM_VALUE_DATA {
    let mut id = [0u8; 16];
    for (dst, src) in id.iter_mut().zip(name.as_bytes()) {
        *dst = *src;
    }
    let (param_value, param_type) = value.to_wire();
    PARAM_VALUE_DATA {
        param_value,
        param_count: count,
        param_index: index,
        param_id: CharArray::new(id),
        param_type,
    }
}

impl Node<'_> {
    pub(super) fn on_incoming(
        &mut self,
        sender: Peer,
        incoming: Incoming,
        now_us: u64,
        params: &mut impl ParamStore,
    ) {
        match incoming {
            Incoming::Heartbeat(heartbeat) => {
                self.stats.heartbeats_received = self.stats.heartbeats_received.saturating_add(1);
                if heartbeat.mavtype == MavType::MAV_TYPE_GCS {
                    self.gcs_seen_us = Some(now_us);
                    self.stats.last_gcs = Some(sender);
                }
            }
            Incoming::ParamRequestList(m) => {
                if self.for_us(m.target_system, m.target_component) {
                    self.list = Some(List {
                        next: 0,
                        due_us: now_us,
                    });
                }
            }
            Incoming::ParamRequestRead(m) => {
                if self.for_us(m.target_system, m.target_component) {
                    self.on_read(&m, params);
                }
            }
            Incoming::ParamSet(m) => {
                if self.for_us(m.target_system, m.target_component) {
                    self.on_set(&m, params);
                }
            }
            Incoming::CommandLong(m) => {
                if self.for_us(m.target_system, m.target_component) {
                    self.on_command(sender, &m);
                }
            }
        }
    }

    fn for_us(&mut self, target_system: u8, target_component: u8) -> bool {
        let ours = (target_system == 0 || target_system == self.config.system_id)
            && (target_component == 0 || target_component == self.config.component_id);
        if !ours {
            self.stats.dropped_not_for_us = self.stats.dropped_not_for_us.saturating_add(1);
        }
        ours
    }

    /// Whether a reply fits; counts a request dropped for want of room.
    fn reply_room(&mut self) -> bool {
        let room = self.reply_count < REPLIES;
        if !room {
            self.stats.dropped_reply_full = self.stats.dropped_reply_full.saturating_add(1);
        }
        room
    }

    fn unknown_param(&mut self) {
        self.stats.dropped_unknown_param = self.stats.dropped_unknown_param.saturating_add(1);
    }

    /// Queues a reply; the caller has checked [`Self::reply_room`].
    fn push_reply(&mut self, reply: Reply) {
        if let Some(slot) = self.replies.get_mut(self.reply_count) {
            *slot = Some(reply);
            self.reply_count += 1;
        }
    }

    fn remove_reply(&mut self, i: usize) {
        let Some(last) = self.reply_count.checked_sub(1) else {
            return;
        };
        if let Some(tail) = self.replies.get_mut(i..=last) {
            tail.rotate_left(1);
        }
        if let Some(slot) = self.replies.get_mut(last) {
            *slot = None;
        }
        self.reply_count = last;
    }

    fn on_read(&mut self, m: &PARAM_REQUEST_READ_DATA, params: &impl ParamStore) {
        let index = if m.param_index == -1 {
            name_of(&m.param_id).and_then(|name| params.find(&name))
        } else {
            u16::try_from(m.param_index).ok()
        };
        let Some((index, value)) = index.and_then(|i| Some((i, params.get(i)?))) else {
            self.unknown_param();
            return;
        };
        if self.reply_room() {
            self.push_reply(Reply::Param { index, value });
        }
    }

    fn on_set(&mut self, m: &PARAM_SET_DATA, params: &mut impl ParamStore) {
        let found = name_of(&m.param_id).and_then(|name| params.find(&name));
        let Some((index, current)) = found.and_then(|i| Some((i, params.get(i)?))) else {
            self.unknown_param();
            return;
        };
        if !self.reply_room() {
            return;
        }
        let in_effect = match ParamValue::from_wire(m.param_value, m.param_type) {
            Some(wanted) if wanted.same_type(current) => match params.set(index, wanted) {
                Ok(value) => value,
                Err(_) => params.get(index).unwrap_or(current),
            },
            _ => current,
        };
        self.push_reply(Reply::Param {
            index,
            value: in_effect,
        });
    }

    fn on_command(&mut self, sender: Peer, m: &COMMAND_LONG_DATA) {
        if !self.reply_room() {
            return;
        }
        let (result, then) = match m.command {
            MavCmd::MAV_CMD_REQUEST_MESSAGE => self.request_message(m.param1),
            MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL => (self.set_message_interval(m), None),
            _ => (MavResult::MAV_RESULT_UNSUPPORTED, None),
        };
        self.push_reply(Reply::Ack {
            command: m.command,
            result,
            target: sender,
            then,
        });
    }

    fn in_table(&self, id: u32) -> bool {
        self.streams.iter().any(|s| s.stream().message_id == id)
    }

    fn request_message(&self, param1: f32) -> (MavResult, Option<Requested>) {
        let denied = (MavResult::MAV_RESULT_DENIED, None);
        let Some(id) = message_id(param1) else {
            return denied;
        };
        let accepted = |r| (MavResult::MAV_RESULT_ACCEPTED, Some(r));
        if id == AUTOPILOT_VERSION_DATA::ID {
            accepted(Requested::Version)
        } else if id == HEARTBEAT_DATA::ID {
            accepted(Requested::Heartbeat)
        } else if self.in_table(id) {
            accepted(Requested::Stream(id))
        } else {
            denied
        }
    }

    fn set_message_interval(&mut self, m: &COMMAND_LONG_DATA) -> MavResult {
        let denied = MavResult::MAV_RESULT_DENIED;
        let Some(id) = message_id(m.param1) else {
            return denied;
        };
        if id == HEARTBEAT_DATA::ID || m.param3 != 0.0 {
            return denied;
        }
        let Some(slot) = self
            .streams
            .iter_mut()
            .find(|s| s.stream().message_id == id)
        else {
            return denied;
        };
        let wanted = m.param2;
        if wanted == -1.0 {
            slot.set_interval(-1);
        } else if wanted == 0.0 {
            slot.reset();
        } else if wanted >= 1.0
            && wanted >= slot.stream().min_interval_us as f32
            && wanted <= MAX_INTERVAL_US
        {
            slot.set_interval(wanted as i32);
        } else {
            return denied;
        }
        MavResult::MAV_RESULT_ACCEPTED
    }

    /// The first reply that can go out now. A requested stream sample waits while frames of an
    /// earlier sample are still pending.
    pub(super) fn poll_reply(
        &mut self,
        now_us: u64,
        params: &impl ParamStore,
        tel: &mut impl Telemetry,
        frame: &mut [u8; FRAME_BUF],
    ) -> Option<usize> {
        let mut i = 0;
        while i < self.reply_count {
            let reply = (*self.replies.get(i)?)?;
            let requested = match reply {
                Reply::Param { index, value } => {
                    self.remove_reply(i);
                    if let Some(name) = params.name(index) {
                        let m = param_value(&name, value, index, params.count());
                        return self.emit(&m, frame);
                    }
                    continue;
                }
                Reply::Ack {
                    command,
                    result,
                    target,
                    then,
                } => {
                    match then {
                        Some(requested) => {
                            *self.replies.get_mut(i)? = Some(Reply::Requested(requested));
                        }
                        None => self.remove_reply(i),
                    }
                    let ack = COMMAND_ACK_DATA {
                        command,
                        result,
                        target_system: target.system_id,
                        target_component: target.component_id,
                        ..Default::default()
                    };
                    return self.emit(&ack, frame);
                }
                Reply::Requested(requested) => requested,
            };
            match requested {
                Requested::Version => {
                    self.remove_reply(i);
                    let v = self.config.version;
                    let m = AUTOPILOT_VERSION_DATA {
                        capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
                            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE,
                        uid: v.uid,
                        flight_sw_version: v.flight_sw_version,
                        middleware_sw_version: v.middleware_sw_version,
                        os_sw_version: v.os_sw_version,
                        board_version: v.board_version,
                        vendor_id: v.vendor_id,
                        product_id: v.product_id,
                        ..Default::default()
                    };
                    return self.emit(&m, frame);
                }
                Requested::Heartbeat => {
                    self.remove_reply(i);
                    return self.emit_heartbeat(frame);
                }
                Requested::Stream(id) => {
                    if self.batch.is_pending() {
                        i += 1;
                        continue;
                    }
                    self.remove_reply(i);
                    if sample_into(&mut self.batch, &self.config, tel, id, now_us)
                        && let Some(n) = self.send_batch_frame(frame)
                    {
                        return Some(n);
                    }
                }
            }
        }
        None
    }

    /// The next item of the parameter list, if one is due.
    pub(super) fn poll_list(
        &mut self,
        now_us: u64,
        params: &impl ParamStore,
        frame: &mut [u8; FRAME_BUF],
    ) -> Option<usize> {
        let list = self.list.filter(|l| l.due_us <= now_us)?;
        let count = params.count();
        if count == 0 {
            self.list = None;
            let m = PARAM_VALUE_DATA {
                param_value: 0.0,
                param_count: 0,
                param_index: u16::MAX,
                param_id: CharArray::new([0u8; 16]),
                param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
            };
            return self.emit(&m, frame);
        }
        let mut index = list.next;
        while index < count {
            if let (Some(name), Some(value)) = (params.name(index), params.get(index)) {
                let next = index.saturating_add(1);
                self.list = (next < count).then_some(List {
                    next,
                    due_us: now_us.saturating_add(u64::from(self.config.param_list_interval_us)),
                });
                let m = param_value(&name, value, index, count);
                return self.emit(&m, frame);
            }
            index = index.saturating_add(1);
        }
        self.list = None;
        None
    }
}
