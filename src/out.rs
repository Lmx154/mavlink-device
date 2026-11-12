//! What the telemetry hook pushes, and the hook itself.

use crate::node::Stats;
use marv_dialect::common::{MavSeverity, MavState};
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MessageData};

/// Texts that can wait to be sent.
pub(crate) const TEXT_QUEUE: usize = 4;
/// Bytes of a STATUSTEXT text.
pub(crate) const TEXT_LEN: usize = 50;

/// Frames one [`Telemetry::sample`] call may push.
pub const MAX_BATCH: usize = 4;

/// Largest frame the node emits (MAVLink 2, unsigned, 16-bit system id field): 10 + 255 + 2 = 267
/// bytes, in the 280-byte frame buffer of [`Node::poll_transmit`](crate::Node::poll_transmit).
pub const FRAME_BUF: usize = 280;

/// A queue or batch is full.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Full;

#[derive(Clone, Copy)]
pub(crate) struct BatchFrame {
    bytes: [u8; FRAME_BUF],
    len: u16,
    extra_crc: u8,
}

impl BatchFrame {
    const EMPTY: Self = Self {
        bytes: [0; FRAME_BUF],
        len: 0,
        extra_crc: 0,
    };

    pub(crate) fn len(&self) -> usize {
        usize::from(self.len)
    }

    pub(crate) fn bytes(&self) -> &[u8] {
        self.bytes.get(..self.len()).unwrap_or(&[])
    }

    pub(crate) const fn extra_crc(&self) -> u8 {
        self.extra_crc
    }
}

/// The frames of one sample, serialised with sequence number 0 and sent in order; the node sets the
/// real sequence number and the checksum when it sends each one.
pub(crate) struct Batch {
    frames: [BatchFrame; MAX_BATCH],
    count: usize,
    next: usize,
}

impl Batch {
    pub(crate) const fn new() -> Self {
        Self {
            frames: [BatchFrame::EMPTY; MAX_BATCH],
            count: 0,
            next: 0,
        }
    }

    pub(crate) fn clear(&mut self) {
        self.count = 0;
        self.next = 0;
    }

    pub(crate) fn is_pending(&self) -> bool {
        self.next < self.count
    }

    /// The next frame to send, if any; it stays until [`Self::advance`].
    pub(crate) fn peek(&self) -> Option<&BatchFrame> {
        if self.is_pending() {
            self.frames.get(self.next)
        } else {
            None
        }
    }

    pub(crate) fn advance(&mut self) {
        self.next += 1;
        if self.next >= self.count {
            self.clear();
        }
    }
}

/// Where [`Telemetry::sample`] puts its frames: up to [`MAX_BATCH`] (4) per call. They are sent in
/// the order pushed, before the stream is sampled again.
pub struct Outgoing<'a> {
    pub(crate) batch: &'a mut Batch,
    pub(crate) system_id: u8,
    pub(crate) component_id: u8,
}

impl Outgoing<'_> {
    /// Adds one message. A fifth call for the same sample returns [`Full`] and adds nothing.
    pub fn push<M: MessageData>(&mut self, m: &M) -> Result<(), Full> {
        let slot = self.batch.frames.get_mut(self.batch.count).ok_or(Full)?;
        let mut raw = MAVLinkV2MessageRaw::new();
        raw.serialize_message_data(
            MavHeader {
                system_id: u32::from(self.system_id),
                component_id: self.component_id,
                sequence: 0,
            },
            m,
        );
        let src = raw.raw_bytes();
        let dst = slot.bytes.get_mut(..src.len()).ok_or(Full)?;
        dst.copy_from_slice(src);
        slot.len = src.len() as u16;
        slot.extra_crc = M::EXTRA_CRC;
        self.batch.count += 1;
        Ok(())
    }
}

#[derive(Clone, Copy)]
pub(crate) struct Text {
    pub(crate) severity: MavSeverity,
    pub(crate) len: usize,
    pub(crate) bytes: [u8; TEXT_LEN],
}

/// The queue of STATUSTEXTs waiting to be sent.
pub(crate) struct TextQueue {
    items: [Option<Text>; TEXT_QUEUE],
    head: usize,
    count: usize,
}

impl TextQueue {
    pub(crate) const fn new() -> Self {
        Self {
            items: [None; TEXT_QUEUE],
            head: 0,
            count: 0,
        }
    }

    pub(crate) fn clear(&mut self) {
        *self = Self::new();
    }

    pub(crate) const fn len(&self) -> usize {
        self.count
    }

    pub(crate) fn has_room(&self) -> bool {
        self.count < TEXT_QUEUE
    }

    pub(crate) fn push(
        &mut self,
        stats: &mut Stats,
        severity: MavSeverity,
        text: &str,
    ) -> Result<(), Full> {
        if !self.has_room() {
            stats.texts_dropped = stats.texts_dropped.saturating_add(1);
            return Err(Full);
        }
        let mut cut = text.len().min(TEXT_LEN);
        while !text.is_char_boundary(cut) {
            cut -= 1;
        }
        let mut bytes = [0u8; TEXT_LEN];
        for (dst, src) in bytes
            .iter_mut()
            .zip(text.as_bytes().get(..cut).unwrap_or(&[]))
        {
            *dst = *src;
        }
        let tail = (self.head + self.count) % TEXT_QUEUE;
        let slot = self.items.get_mut(tail).ok_or(Full)?;
        *slot = Some(Text {
            severity,
            len: cut,
            bytes,
        });
        self.count += 1;
        Ok(())
    }

    pub(crate) fn pop(&mut self) -> Option<Text> {
        if self.count == 0 {
            return None;
        }
        let text = self.items.get_mut(self.head)?.take();
        self.head = (self.head + 1) % TEXT_QUEUE;
        self.count -= 1;
        text.filter(|t| t.len <= TEXT_LEN)
    }
}

/// Where [`Telemetry::texts`] puts its STATUSTEXTs: the same queue as
/// [`Node::queue_text`](crate::Node::queue_text).
pub struct Texts<'a> {
    pub(crate) queue: &'a mut TextQueue,
    pub(crate) stats: &'a mut Stats,
}

impl Texts<'_> {
    /// Queues a STATUSTEXT under the rules of [`Node::queue_text`](crate::Node::queue_text): at
    /// most 4 wait, text beyond 50 bytes is cut on a character boundary. A `push` that does not
    /// fit returns [`Full`] and is counted in [`Stats::texts_dropped`](crate::Stats).
    pub fn push(&mut self, severity: MavSeverity, text: &str) -> Result<(), Full> {
        self.queue.push(self.stats, severity, text)
    }
}

/// The firmware's sensor hook. Values are sampled when due and never queued.
pub trait Telemetry {
    /// Called once per period of the stream with this `message_id`. Push the frames to send with
    /// [`Outgoing::push`] (for example four HIGHRES_IMU instances) and return `true`. Return `false`,
    /// or push nothing, to send nothing this period; the period is used up either way.
    fn sample(&mut self, message_id: u32, now_us: u64, out: &mut Outgoing<'_>) -> bool;

    /// Called once, right before each HEARTBEAT is built. `Some(state)` becomes the node's status
    /// from then on, exactly as [`Node::set_status`](crate::Node::set_status) would; `None` leaves
    /// it as it is. The default is `None`.
    fn status(&mut self, now_us: u64) -> Option<MavState> {
        let _ = now_us;
        None
    }

    /// Called at most once per [`Node::poll_transmit`](crate::Node::poll_transmit), when no
    /// HEARTBEAT is due and no reply waits, and only while the text queue has room. Queue texts
    /// with [`Texts::push`]. The default pushes nothing.
    fn texts(&mut self, now_us: u64, out: &mut Texts<'_>) {
        let _ = (now_us, out);
    }
}
