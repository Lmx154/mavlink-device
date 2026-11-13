//! The accept list and the hand-over of listed messages to the application.

use marv_dialect::{MavlinkVersion, MessageData};

/// Most entries an accept list holds.
pub const MAX_ACCEPT: usize = 16;

/// More than [`MAX_ACCEPT`] entries were given to [`Config::with_accept_list`](crate::Config::with_accept_list).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AcceptListFull;

/// A fixed list of (message id, CRC_EXTRA) pairs: the messages the application wants handed over.
/// Build it through [`Config::with_accept_list`](crate::Config::with_accept_list).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AcceptList {
    entries: [(u32, u8); MAX_ACCEPT],
    len: usize,
}

impl AcceptList {
    /// An empty list.
    pub const fn new() -> Self {
        Self {
            entries: [(0, 0); MAX_ACCEPT],
            len: 0,
        }
    }

    /// The list of `pairs`, or [`AcceptListFull`] when there are more than [`MAX_ACCEPT`]. Nothing
    /// is truncated.
    pub const fn from_pairs(pairs: &[(u32, u8)]) -> Result<Self, AcceptListFull> {
        if pairs.len() > MAX_ACCEPT {
            return Err(AcceptListFull);
        }
        let mut list = Self::new();
        let mut rest = pairs;
        while let [first, tail @ ..] = rest {
            if list.len < MAX_ACCEPT {
                // `get_mut` is not const yet; the index is checked on the line above.
                #[allow(clippy::indexing_slicing)]
                {
                    list.entries[list.len] = *first;
                }
                list.len += 1;
            }
            rest = tail;
        }
        Ok(list)
    }

    /// How many entries the list holds.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Whether the list is empty.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The CRC_EXTRA listed for `message_id` (the first entry with that id), if any.
    pub(crate) fn extra_crc(&self, message_id: u32) -> Option<u8> {
        self.entries
            .get(..self.len)?
            .iter()
            .find(|e| e.0 == message_id)
            .map(|e| e.1)
    }
}

impl Default for AcceptList {
    fn default() -> Self {
        Self::new()
    }
}

/// A listed message that passed its checks, handed to the application inside
/// [`Node::receive_with`](crate::Node::receive_with). It borrows the node's receive buffer and is
/// valid only for the duration of the call.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Incoming<'a> {
    /// The sender's MAVLink system id (never the node's own).
    pub system_id: u8,
    /// The sender's MAVLink component id.
    pub component_id: u8,
    /// The message id.
    pub message_id: u32,
    /// The payload exactly as on the wire. MAVLink 2 senders cut the trailing zero bytes of a
    /// payload, so it can be shorter than the message's encoded length (even empty). Decode it with
    /// [`decode`](Self::decode), which zero-extends it, never by indexing it.
    pub payload: &'a [u8],
    /// The `now_us` given to `receive_with`.
    pub now_us: u64,
}

impl Incoming<'_> {
    /// Decodes the payload as `M` (a `marv-dialect` message type, for example
    /// `RC_CHANNELS_OVERRIDE_DATA`), restoring the truncated zeros. `None` when `M::ID` is not this
    /// message's id or the payload does not decode.
    pub fn decode<M: MessageData>(&self) -> Option<M> {
        if M::ID != self.message_id {
            return None;
        }
        M::deser(MavlinkVersion::V2, self.payload).ok()
    }
}
