//! Parameter types and the storage trait the firmware implements.

use marv_dialect::common::MavParamType;

/// A parameter value. The firmware uses only these two types (a bool or an enum is an `I32`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParamValue {
    /// A signed 32-bit integer.
    I32(i32),
    /// A 32-bit float.
    F32(f32),
}

impl ParamValue {
    /// The value field and type of a PARAM_VALUE or PARAM_SET: byte-wise, an `I32` travels as its
    /// own little-endian bytes in the 4-byte float field.
    pub(crate) fn to_wire(self) -> (f32, MavParamType) {
        match self {
            Self::I32(v) => (f32::from_bits(v as u32), MavParamType::MAV_PARAM_TYPE_INT32),
            Self::F32(v) => (v, MavParamType::MAV_PARAM_TYPE_REAL32),
        }
    }

    /// The inverse of [`Self::to_wire`]; `None` for any type other than INT32 and REAL32.
    pub(crate) fn from_wire(value: f32, kind: MavParamType) -> Option<Self> {
        match kind {
            MavParamType::MAV_PARAM_TYPE_INT32 => Some(Self::I32(value.to_bits() as i32)),
            MavParamType::MAV_PARAM_TYPE_REAL32 => Some(Self::F32(value)),
            _ => None,
        }
    }

    pub(crate) fn same_type(self, other: Self) -> bool {
        matches!(
            (self, other),
            (Self::I32(_), Self::I32(_)) | (Self::F32(_), Self::F32(_))
        )
    }
}

/// Why a parameter name, index or value was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParamError {
    /// The name is empty, longer than 16 bytes, or has a byte outside `A-Z`, `0-9`, `_`.
    InvalidName,
    /// No parameter has this index.
    NotFound,
    /// The value has another type than the parameter.
    WrongType,
    /// The value is outside what the parameter accepts.
    OutOfRange,
    /// The firmware refuses the write for another reason.
    Rejected,
}

/// A validated parameter name: 1 to 16 bytes of `A-Z`, `0-9` and `_`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ParamName {
    bytes: [u8; 16],
    len: u8,
}

impl ParamName {
    /// Longest name, in bytes.
    pub const MAX_LEN: usize = 16;

    /// Validates `name`.
    pub fn new(name: &str) -> Result<Self, ParamError> {
        Self::from_bytes(name.as_bytes())
    }

    /// Validates `name`; the bytes of a PARAM message's name field, up to its first zero, go here.
    pub fn from_bytes(name: &[u8]) -> Result<Self, ParamError> {
        let valid = !name.is_empty()
            && name.len() <= Self::MAX_LEN
            && name
                .iter()
                .all(|&b| b.is_ascii_uppercase() || b.is_ascii_digit() || b == b'_');
        if !valid {
            return Err(ParamError::InvalidName);
        }
        let mut bytes = [0u8; 16];
        for (dst, src) in bytes.iter_mut().zip(name) {
            *dst = *src;
        }
        Ok(Self {
            bytes,
            len: name.len() as u8,
        })
    }

    /// The name's bytes, without padding.
    pub fn as_bytes(&self) -> &[u8] {
        self.bytes.get(..usize::from(self.len)).unwrap_or(&[])
    }

    /// The name as text.
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(self.as_bytes()).unwrap_or("")
    }
}

/// The firmware's parameter storage, validation and persistence. The set, its order and its count
/// are fixed for one boot; indices are positions in that order.
pub trait ParamStore {
    /// Number of parameters.
    fn count(&self) -> u16;
    /// Name of parameter `i`.
    fn name(&self, i: u16) -> Option<ParamName>;
    /// Value of parameter `i`.
    fn get(&self, i: u16) -> Option<ParamValue>;
    /// Index of the parameter called `n`.
    fn find(&self, n: &ParamName) -> Option<u16>;
    /// Writes parameter `i` and returns the value now in effect.
    fn set(&mut self, i: u16, v: ParamValue) -> Result<ParamValue, ParamError>;
}
