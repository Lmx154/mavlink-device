//! Node configuration and the stream table.

use marv_dialect::common::{MavAutopilot, MavType};

/// Static configuration of a [`Node`](crate::Node).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The node's MAVLink system id. Frames carrying it are dropped.
    pub system_id: u8,
    /// The node's MAVLink component id.
    pub component_id: u8,
    /// `HEARTBEAT.type`.
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    pub vehicle_type: MavType,
    /// `HEARTBEAT.autopilot`.
    #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
    pub autopilot: MavAutopilot,
    /// The ground station is called lost this long after its last HEARTBEAT, in microseconds.
    pub gcs_timeout_us: u64,
    /// Spacing of the PARAM_VALUE frames of a parameter list, in microseconds.
    pub param_list_interval_us: u32,
    /// What AUTOPILOT_VERSION reports.
    pub version: VersionInfo,
}

/// The version numbers, identity and unique id that AUTOPILOT_VERSION reports. All zero by default.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct VersionInfo {
    /// `flight_sw_version`.
    pub flight_sw_version: u32,
    /// `middleware_sw_version`.
    pub middleware_sw_version: u32,
    /// `os_sw_version`.
    pub os_sw_version: u32,
    /// `board_version`.
    pub board_version: u32,
    /// `vendor_id`.
    pub vendor_id: u16,
    /// `product_id`.
    pub product_id: u16,
    /// `uid`.
    pub uid: u64,
}

impl VersionInfo {
    /// Everything zero.
    pub const fn new() -> Self {
        Self {
            flight_sw_version: 0,
            middleware_sw_version: 0,
            os_sw_version: 0,
            board_version: 0,
            vendor_id: 0,
            product_id: 0,
            uid: 0,
        }
    }

    /// Sets `flight_sw_version`.
    pub const fn with_flight_sw_version(mut self, v: u32) -> Self {
        self.flight_sw_version = v;
        self
    }

    /// Sets `middleware_sw_version`.
    pub const fn with_middleware_sw_version(mut self, v: u32) -> Self {
        self.middleware_sw_version = v;
        self
    }

    /// Sets `os_sw_version`.
    pub const fn with_os_sw_version(mut self, v: u32) -> Self {
        self.os_sw_version = v;
        self
    }

    /// Sets `board_version`.
    pub const fn with_board_version(mut self, v: u32) -> Self {
        self.board_version = v;
        self
    }

    /// Sets `vendor_id`.
    pub const fn with_vendor_id(mut self, v: u16) -> Self {
        self.vendor_id = v;
        self
    }

    /// Sets `product_id`.
    pub const fn with_product_id(mut self, v: u16) -> Self {
        self.product_id = v;
        self
    }

    /// Sets `uid`.
    pub const fn with_uid(mut self, v: u64) -> Self {
        self.uid = v;
        self
    }
}

impl Config {
    /// Vehicle type and autopilot GENERIC, ground-station timeout 5 s, parameter list interval 2 ms.
    pub const fn new(system_id: u8, component_id: u8) -> Self {
        Self {
            system_id,
            component_id,
            vehicle_type: MavType::MAV_TYPE_GENERIC,
            autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
            gcs_timeout_us: 5_000_000,
            param_list_interval_us: 2_000,
            version: VersionInfo::new(),
        }
    }

    /// Sets `HEARTBEAT.type`.
    pub const fn with_vehicle_type(mut self, vehicle_type: MavType) -> Self {
        self.vehicle_type = vehicle_type;
        self
    }

    /// Sets `HEARTBEAT.autopilot`.
    pub const fn with_autopilot(mut self, autopilot: MavAutopilot) -> Self {
        self.autopilot = autopilot;
        self
    }

    /// Sets how long the ground station may stay silent before it is called lost.
    pub const fn with_gcs_timeout_us(mut self, gcs_timeout_us: u64) -> Self {
        self.gcs_timeout_us = gcs_timeout_us;
        self
    }

    /// Sets the spacing of the frames of a parameter list.
    pub const fn with_param_list_interval_us(mut self, param_list_interval_us: u32) -> Self {
        self.param_list_interval_us = param_list_interval_us;
        self
    }

    /// Sets what AUTOPILOT_VERSION reports.
    pub const fn with_version(mut self, version: VersionInfo) -> Self {
        self.version = version;
        self
    }
}

/// One entry of the firmware's stream table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Stream {
    /// Id of the message the telemetry hook is asked to sample.
    pub message_id: u32,
    /// Interval at boot and after a link-down, in microseconds. Zero or negative: off.
    pub default_interval_us: i32,
    /// Shortest interval allowed, in microseconds. A default below it is raised to it.
    pub min_interval_us: u32,
}

/// A [`Stream`] plus its run-time state. The firmware owns the slice of slots and hands it to
/// [`Node::new`](crate::Node::new), which resets every slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StreamSlot {
    stream: Stream,
    pub(crate) interval_us: i32,
    due_us: Option<u64>,
}

impl StreamSlot {
    /// A slot at its default interval, not yet started.
    pub const fn new(stream: Stream) -> Self {
        Self {
            stream,
            interval_us: default_interval(&stream),
            due_us: None,
        }
    }

    /// The table entry.
    pub const fn stream(&self) -> &Stream {
        &self.stream
    }

    /// The interval in force, in microseconds; zero or negative: off.
    pub const fn interval_us(&self) -> i32 {
        self.interval_us
    }

    pub(crate) fn reset(&mut self) {
        *self = Self::new(self.stream);
    }

    /// A new interval (negative: off) with the schedule restarted: the next sample is due at once.
    pub(crate) fn set_interval(&mut self, interval_us: i32) {
        self.interval_us = interval_us;
        self.due_us = None;
    }

    fn period_us(&self) -> Option<u64> {
        u64::try_from(self.interval_us).ok().filter(|&p| p > 0)
    }

    pub(crate) fn is_due(&self, now_us: u64) -> bool {
        self.period_us().is_some() && self.due_us.is_none_or(|d| d <= now_us)
    }

    pub(crate) fn deadline_us(&self) -> Option<u64> {
        self.period_us().map(|_| self.due_us.unwrap_or(0))
    }

    /// Moves the schedule past `now_us`: the next period on the grid of the first one, so missed
    /// periods are skipped and the rate does not drift.
    pub(crate) fn advance(&mut self, now_us: u64) {
        let Some(period) = self.period_us() else {
            return;
        };
        self.due_us = Some(match self.due_us {
            None => now_us.saturating_add(period),
            Some(d) => {
                d.saturating_add(period.saturating_mul(now_us.saturating_sub(d) / period + 1))
            }
        });
    }
}

const fn default_interval(stream: &Stream) -> i32 {
    if stream.default_interval_us <= 0 {
        return stream.default_interval_us;
    }
    let min = stream.min_interval_us as i64;
    let v = stream.default_interval_us as i64;
    let v = if v < min { min } else { v };
    if v > i32::MAX as i64 {
        i32::MAX
    } else {
        v as i32
    }
}
