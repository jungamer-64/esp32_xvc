//! XVC wire-format decoding.

use crate::jtag::bytes_for_bits;

pub(super) const BUFFERED_SHIFT_MAX_BITS: usize = 32_768;
pub(super) const BUFFERED_SHIFT_MAX_BYTES: usize = bytes_for_bits(BUFFERED_SHIFT_MAX_BITS);
pub(super) const GETINFO_RESPONSE: &[u8] = b"xvcServer_v1.0:196608\n";

pub(super) enum Command<'data> {
    GetInfo,
    SetTck { requested_period_ns: u32 },
    Shift(BufferedShift<'data>),
}

pub(super) struct BufferedShift<'data> {
    pub(super) bit_count: usize,
    pub(super) tms: &'data [u8],
    pub(super) tdi: &'data [u8],
}

pub(super) enum DecodeOutcome<'data> {
    NeedMore,
    Command {
        command: Command<'data>,
        consumed: usize,
    },
    StreamingShift {
        bit_count: usize,
        consumed: usize,
    },
    ProtocolError,
}

pub(super) fn decode(data: &[u8]) -> DecodeOutcome<'_> {
    if data.len() >= 6 && data.starts_with(b"shift:") {
        if data.len() < 10 {
            return DecodeOutcome::NeedMore;
        }

        let bit_count = u32::from_le_bytes([data[6], data[7], data[8], data[9]]) as usize;
        if bit_count > BUFFERED_SHIFT_MAX_BITS {
            return DecodeOutcome::StreamingShift {
                bit_count,
                consumed: 10,
            };
        }

        let byte_count = bytes_for_bits(bit_count);
        let Some(payload_len) = byte_count.checked_mul(2) else {
            return DecodeOutcome::ProtocolError;
        };
        let Some(frame_len) = 10_usize.checked_add(payload_len) else {
            return DecodeOutcome::ProtocolError;
        };
        if data.len() < frame_len {
            return DecodeOutcome::NeedMore;
        }

        return DecodeOutcome::Command {
            command: Command::Shift(BufferedShift {
                bit_count,
                tms: &data[10..10 + byte_count],
                tdi: &data[10 + byte_count..frame_len],
            }),
            consumed: frame_len,
        };
    }

    if data.len() < 8 {
        return DecodeOutcome::NeedMore;
    }

    if data.starts_with(b"getinfo:") {
        return DecodeOutcome::Command {
            command: Command::GetInfo,
            consumed: 8,
        };
    }

    if data.starts_with(b"settck:") {
        if data.len() < 11 {
            return DecodeOutcome::NeedMore;
        }
        return DecodeOutcome::Command {
            command: Command::SetTck {
                requested_period_ns: u32::from_le_bytes([data[7], data[8], data[9], data[10]]),
            },
            consumed: 11,
        };
    }

    DecodeOutcome::ProtocolError
}
