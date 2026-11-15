//! Commands the node does not serve itself: what the firmware is asked and what it answers.

use crate::node::Peer;
use marv_dialect::marv::{MavCmd, MavResult};

/// A COMMAND_LONG addressed to this node that the node does not serve itself (it serves
/// SET_MESSAGE_INTERVAL and REQUEST_MESSAGE; those never reach the firmware). Passed to
/// [`Telemetry::command`](crate::Telemetry::command).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommandRequest {
    /// The command; a number of the `marv` dialect (the project's own included).
    pub command: MavCmd,
    /// `param1` to `param7`, in order.
    pub params: [f32; 7],
    /// The system and component that sent the command; the ACK is addressed to them.
    pub sender: Peer,
    /// The `confirmation` counter of the frame: 0 for the first transmission, one more for each
    /// retransmission. The firmware decides what a retransmission means.
    pub confirmation: u8,
}

/// What the COMMAND_ACK carries.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommandOutcome {
    /// The MAV_RESULT.
    pub result: MavResult,
    /// The `progress` field: a percentage for IN_PROGRESS, 0 otherwise by [`Self::new`].
    pub progress: u8,
    /// The `result_param2` field (extension): a command-specific code, 0 when unused.
    pub result_param2: i32,
}

impl CommandOutcome {
    /// What the default [`Telemetry::command`](crate::Telemetry::command) answers.
    pub const UNSUPPORTED: Self = Self::new(MavResult::MAV_RESULT_UNSUPPORTED);

    /// `result` with `progress` 0 and `result_param2` 0.
    pub const fn new(result: MavResult) -> Self {
        Self {
            result,
            progress: 0,
            result_param2: 0,
        }
    }

    /// IN_PROGRESS with `percent` (above 100 is cut to 100).
    pub const fn in_progress(percent: u8) -> Self {
        Self {
            result: MavResult::MAV_RESULT_IN_PROGRESS,
            progress: if percent > 100 { 100 } else { percent },
            result_param2: 0,
        }
    }
}

/// A COMMAND_ACK the firmware sends later, for a long command it answered IN_PROGRESS (or any
/// other command): returned by [`Telemetry::acks`](crate::Telemetry::acks).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LateAck {
    /// The command acknowledged.
    pub command: MavCmd,
    /// The original sender of the command (target system and component of the ACK).
    pub target: Peer,
    /// The result, progress and `result_param2`.
    pub outcome: CommandOutcome,
}
