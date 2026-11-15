//! Commands for the firmware's hook (`Telemetry::command`) and the later ACKs (`Telemetry::acks`).
//! Frames are made and read with the `marv` dialect types, so a command number of the project's
//! own dialect is a valid command.

use crate::marv::{COMMAND_ACK_DATA, COMMAND_LONG_DATA, MavCmd, MavResult};
use crate::tests::NoParams;
use crate::*;
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, MessageData};
use std::collections::VecDeque;

const SENDER: Peer = Peer {
    system_id: 200,
    component_id: 17,
};

/// A firmware that records the commands it is asked and answers `outcome`; its later ACKs wait in
/// `late` until `acks` is asked.
struct Firmware {
    calls: Vec<(CommandRequest, u64)>,
    outcome: CommandOutcome,
    late: VecDeque<LateAck>,
}

impl Firmware {
    fn new(outcome: CommandOutcome) -> Self {
        Self {
            calls: Vec::new(),
            outcome,
            late: VecDeque::new(),
        }
    }
}

impl Telemetry for Firmware {
    fn sample(&mut self, _: u32, _: u64, _: &mut Outgoing<'_>) -> bool {
        false
    }
    fn command(&mut self, req: &CommandRequest, now_us: u64) -> CommandOutcome {
        self.calls.push((*req, now_us));
        self.outcome
    }
    fn acks(&mut self, _: u64) -> Option<LateAck> {
        self.late.pop_front()
    }
}

/// Only the required method: every provided method is the default.
struct Plain;

impl Telemetry for Plain {
    fn sample(&mut self, _: u32, _: u64, _: &mut Outgoing<'_>) -> bool {
        false
    }
}

fn long(command: MavCmd, params: [f32; 7], target: (u8, u8), confirmation: u8) -> Vec<u8> {
    let m = COMMAND_LONG_DATA {
        param1: params[0],
        param2: params[1],
        param3: params[2],
        param4: params[3],
        param5: params[4],
        param6: params[5],
        param7: params[6],
        command,
        target_system: target.0,
        target_component: target.1,
        confirmation,
    };
    let mut r = MAVLinkV2MessageRaw::new();
    r.serialize_message_data(
        MavHeader {
            system_id: u32::from(SENDER.system_id),
            component_id: SENDER.component_id,
            sequence: 0,
        },
        &m,
    );
    r.raw_bytes().to_vec()
}

fn to_node(command: MavCmd, params: [f32; 7]) -> Vec<u8> {
    long(command, params, (1, 1), 0)
}

fn calibration() -> Vec<u8> {
    to_node(
        MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
}

/// Every frame the node has now, whole.
fn drain<T: Telemetry>(node: &mut Node<'_>, now: u64, tel: &mut T) -> Vec<Vec<u8>> {
    let mut frames = Vec::new();
    let mut buf = [0u8; FRAME_BUF];
    while let Some(n) = node.poll_transmit(now, &NoParams, tel, &mut buf) {
        frames.push(buf[..n].to_vec());
        assert!(frames.len() < 100);
    }
    frames
}

/// The COMMAND_ACKs among `frames`, in wire order, with the sender (system, component) of each.
fn acks_of(frames: &[Vec<u8>]) -> Vec<((u8, u8), COMMAND_ACK_DATA)> {
    frames
        .iter()
        .filter(|f| {
            let id = u32::from_le_bytes([f[7], f[8], f[9], 0]);
            id == COMMAND_ACK_DATA::ID
        })
        .map(|f| {
            let len = usize::from(f[1]);
            let ack = COMMAND_ACK_DATA::deser(MavlinkVersion::V2, &f[10..10 + len]).unwrap();
            ((f[5], f[6]), ack)
        })
        .collect()
}

fn heard_by_a_node(bytes: &[u8], tel: &mut Firmware) -> Vec<((u8, u8), COMMAND_ACK_DATA)> {
    let mut node = Node::new(Config::new(1, 1), &mut []);
    node.receive(bytes, 5_000, &mut NoParams);
    acks_of(&drain(&mut node, 6_000, tel))
}

#[test]
fn preflight_calibration_reaches_the_hook_once_and_the_ack_carries_its_outcome() {
    let mut fw = Firmware::new(CommandOutcome {
        result: MavResult::MAV_RESULT_DENIED,
        progress: 0,
        result_param2: 7,
    });
    let acks = heard_by_a_node(&calibration(), &mut fw);
    assert_eq!(fw.calls.len(), 1);
    let (req, at) = fw.calls[0];
    assert_eq!(at, 6_000, "called in poll_transmit, with its now_us");
    assert_eq!(req.command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
    assert_eq!(req.params, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    assert_eq!(req.sender, SENDER);
    assert_eq!(req.confirmation, 0);
    assert_eq!(acks.len(), 1);
    let (from, ack) = &acks[0];
    assert_eq!(*from, (1, 1));
    assert_eq!(ack.command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
    assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    assert_eq!(ack.result_param2, 7);
    assert_eq!(
        (ack.target_system, ack.target_component),
        (SENDER.system_id, SENDER.component_id)
    );
}

#[test]
fn the_default_hook_answers_unsupported() {
    let mut node = Node::new(Config::new(1, 1), &mut []);
    node.receive(&calibration(), 0, &mut NoParams);
    let acks = acks_of(&drain(&mut node, 1, &mut Plain));
    assert_eq!(acks.len(), 1);
    assert_eq!(acks[0].1.command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
    assert_eq!(acks[0].1.result, MavResult::MAV_RESULT_UNSUPPORTED);
    assert_eq!(acks[0].1.progress, 0);
}

/// Hook calls after `receive` alone, and after the first `poll_transmit` that follows it.
fn hook_calls(early: bool) -> (usize, usize) {
    let mut fw = Firmware::new(CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED));
    let mut node = Node::new(Config::new(1, 1), &mut []);
    if early {
        #[cfg(feature = "_test-hooks")]
        node.receive_calling_command_hook(&calibration(), 0, &mut NoParams, &mut fw);
        #[cfg(not(feature = "_test-hooks"))]
        unreachable!();
    } else {
        node.receive(&calibration(), 0, &mut NoParams);
    }
    let after_receive = fw.calls.len();
    drain(&mut node, 1, &mut fw);
    (after_receive, fw.calls.len())
}

#[test]
fn i5_1_the_hook_is_not_called_from_receive_only_from_poll_transmit() {
    assert_eq!(hook_calls(false), (0, 1));
}

#[cfg(feature = "_test-hooks")]
#[test]
fn negative_control_the_i5_1_test_fails_when_the_hook_is_called_from_receive() {
    let (after_receive, after_poll) = hook_calls(true);
    assert_ne!(after_receive, 0, "the I5.1 assertion (0) must fail here");
    assert_eq!(after_poll, 1, "still once per command frame");
}

#[test]
fn in_progress_then_the_final_ack_each_once_to_the_sender() {
    let mut fw = Firmware::new(CommandOutcome::in_progress(0));
    let mut node = Node::new(Config::new(1, 1), &mut []);
    node.receive(&calibration(), 0, &mut NoParams);
    let first = acks_of(&drain(&mut node, 1, &mut fw));
    assert_eq!(first.len(), 1);
    assert_eq!(first[0].1.result, MavResult::MAV_RESULT_IN_PROGRESS);
    for outcome in [
        CommandOutcome::in_progress(50),
        CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED),
    ] {
        fw.late.push_back(LateAck {
            command: MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            target: SENDER,
            outcome,
        });
    }
    let later = acks_of(&drain(&mut node, 500_000, &mut fw));
    assert_eq!(later.len(), 2);
    assert_eq!(later[0].1.result, MavResult::MAV_RESULT_IN_PROGRESS);
    assert_eq!(later[0].1.progress, 50);
    assert_eq!(later[1].1.result, MavResult::MAV_RESULT_ACCEPTED);
    for (from, ack) in &later {
        assert_eq!(*from, (1, 1));
        assert_eq!(ack.command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
        assert_eq!(
            (ack.target_system, ack.target_component),
            (SENDER.system_id, SENDER.component_id)
        );
    }
    assert!(acks_of(&drain(&mut node, 900_000, &mut fw)).is_empty());
    assert_eq!(
        fw.calls.len(),
        1,
        "the later acks did not call the command hook"
    );
}

#[test]
fn a_late_ack_does_not_starve_a_reply_or_a_due_heartbeat() {
    let mut fw = Firmware::new(CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED));
    fw.late.push_back(LateAck {
        command: MavCmd::MAV_CMD_PREFLIGHT_STORAGE,
        target: SENDER,
        outcome: CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED),
    });
    let mut node = Node::new(Config::new(1, 1), &mut []);
    node.receive(&calibration(), 0, &mut NoParams);
    let frames = drain(&mut node, 0, &mut fw);
    let ids: Vec<u32> = frames
        .iter()
        .map(|f| u32::from_le_bytes([f[7], f[8], f[9], 0]))
        .collect();
    // heartbeat, then the reply of the command (its ACK), then the late ACK
    assert_eq!(ids, [0, 77, 77]);
    let acks = acks_of(&frames);
    assert_eq!(acks[0].1.command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
    assert_eq!(acks[1].1.command, MavCmd::MAV_CMD_PREFLIGHT_STORAGE);
}

#[test]
fn set_message_interval_and_request_message_never_reach_the_hook() {
    let mut fw = Firmware::new(CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED));
    let mut slots = [StreamSlot::new(Stream {
        message_id: 29,
        default_interval_us: 100_000,
        min_interval_us: 20_000,
    })];
    let mut node = Node::new(Config::new(1, 1), &mut slots);
    node.receive(
        &to_node(
            MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            [29.0, 50_000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),
        0,
        &mut NoParams,
    );
    node.receive(
        &to_node(
            MavCmd::MAV_CMD_REQUEST_MESSAGE,
            [148.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ),
        0,
        &mut NoParams,
    );
    let frames = drain(&mut node, 1, &mut fw);
    assert_eq!(fw.calls.len(), 0);
    let acks = acks_of(&frames);
    assert_eq!(acks.len(), 2);
    assert_eq!(acks[0].1.command, MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL);
    assert_eq!(acks[0].1.result, MavResult::MAV_RESULT_ACCEPTED);
    assert_eq!(acks[1].1.command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
    assert_eq!(acks[1].1.result, MavResult::MAV_RESULT_ACCEPTED);
    let version = frames
        .iter()
        .any(|f| u32::from_le_bytes([f[7], f[8], f[9], 0]) == 148);
    assert!(version, "AUTOPILOT_VERSION still follows the ACK");
}

#[test]
fn the_projects_own_command_42430_decodes_reaches_the_hook_and_acks_with_its_number() {
    assert_eq!(MavCmd::MAV_CMD_MARV_DEVICE as u32, 42430);
    let mut fw = Firmware::new(CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED));
    let bytes = to_node(
        MavCmd::MAV_CMD_MARV_DEVICE,
        [3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    );
    let mut node = Node::new(Config::new(1, 1), &mut []);
    node.receive(&bytes, 0, &mut NoParams);
    assert_eq!(node.stats().dropped_malformed, 0);
    let frames = drain(&mut node, 1, &mut fw);
    assert_eq!(fw.calls.len(), 1);
    assert_eq!(fw.calls[0].0.command, MavCmd::MAV_CMD_MARV_DEVICE);
    assert_eq!(fw.calls[0].0.params[0], 3.0);
    assert_eq!(fw.calls[0].0.params[1], 1.0);
    let acks = acks_of(&frames);
    assert_eq!(acks.len(), 1);
    assert_eq!(acks[0].1.command, MavCmd::MAV_CMD_MARV_DEVICE);
    // the number on the wire: COMMAND_ACK payload starts with the u16 command
    let f = frames.iter().find(|f| f[7] == 77).unwrap();
    assert_eq!(u16::from_le_bytes([f[10], f[11]]), 42430);
}

#[test]
fn a_command_number_outside_the_marv_dialect_is_still_dropped_as_malformed() {
    let mut bytes = calibration();
    // overwrite the command (payload offset 28, u16) with a number no dialect has, fix the checksum
    bytes[10 + 28] = 0x0F;
    bytes[10 + 29] = 0x27;
    let len = bytes.len();
    let crc = marv_dialect::calculate_crc(&bytes[1..len - 2], COMMAND_LONG_DATA::EXTRA_CRC);
    bytes[len - 2..].copy_from_slice(&crc.to_le_bytes());
    let mut fw = Firmware::new(CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED));
    let mut node = Node::new(Config::new(1, 1), &mut []);
    node.receive(&bytes, 0, &mut NoParams);
    assert_eq!(node.stats().dropped_malformed, 1);
    assert!(acks_of(&drain(&mut node, 1, &mut fw)).is_empty());
    assert_eq!(fw.calls.len(), 0);
}

#[test]
fn a_command_for_another_system_or_component_does_not_reach_the_hook() {
    // The addressing rule of the node: target_system is the node's or 0, and target_component is
    // the node's or 0; otherwise Stats::dropped_not_for_us and no reply.
    let mut fw = Firmware::new(CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED));
    let mut node = Node::new(Config::new(1, 1), &mut []);
    let cmd = MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION;
    for target in [(2, 1), (1, 2), (2, 2)] {
        node.receive(&long(cmd, [1.0; 7], target, 0), 0, &mut NoParams);
    }
    assert_eq!(node.stats().dropped_not_for_us, 3);
    assert!(acks_of(&drain(&mut node, 1, &mut fw)).is_empty());
    assert_eq!(fw.calls.len(), 0);
    for target in [(0, 0), (1, 0), (0, 1), (1, 1)] {
        node.receive(&long(cmd, [1.0; 7], target, 0), 2, &mut NoParams);
        assert_eq!(drain(&mut node, 3, &mut fw).len(), 1);
    }
    assert_eq!(fw.calls.len(), 4);
}

#[test]
fn a_retransmission_reaches_the_hook_again_with_its_confirmation() {
    let mut fw = Firmware::new(CommandOutcome::in_progress(0));
    let mut node = Node::new(Config::new(1, 1), &mut []);
    let cmd = MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION;
    node.receive(&long(cmd, [1.0; 7], (1, 1), 0), 0, &mut NoParams);
    node.receive(&long(cmd, [1.0; 7], (1, 1), 1), 0, &mut NoParams);
    let acks = acks_of(&drain(&mut node, 1, &mut fw));
    assert_eq!(acks.len(), 2);
    let confirmations: Vec<u8> = fw.calls.iter().map(|c| c.0.confirmation).collect();
    assert_eq!(confirmations, [0, 1]);
}

#[test]
fn a_full_reply_queue_drops_the_command_without_calling_the_hook() {
    let mut fw = Firmware::new(CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED));
    let mut node = Node::new(Config::new(1, 1), &mut []);
    for _ in 0..5 {
        node.receive(&calibration(), 0, &mut NoParams);
    }
    assert_eq!(node.stats().dropped_reply_full, 1);
    assert_eq!(acks_of(&drain(&mut node, 1, &mut fw)).len(), 4);
    assert_eq!(fw.calls.len(), 4);
}
