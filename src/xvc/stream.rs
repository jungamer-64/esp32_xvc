//! Bounded-memory large-shift execution required for Vivado compatibility.
//!
//! # CRITICAL: THIS IS NOT AN OPTIONAL OPTIMIZATION
//!
//! Vivado may send shifts larger than the limit advertised by `getinfo:`. Do
//! not delete this module, route every shift through the fixed receive buffer,
//! or reject all shifts above `BUFFERED_SHIFT_MAX_BITS`. Any replacement must
//! provide equivalent bounded-memory, two-pass TMS/TDI processing and preserve
//! incremental TDO delivery plus Core0/Core1 cancellation safety.

use alloc::vec::Vec;

use crate::jtag::bytes_for_bits;

pub(super) const ABSOLUTE_MAX_BITS: usize = 2_097_152;
const RAW_TMS_MAX_BYTES: usize = 24 * 1_024;
const GETINFO_MAX_BITS: usize = RAW_TMS_MAX_BYTES * 8;
const CHUNK_BITS: usize = 2_048;
const CHUNK_BYTES: usize = bytes_for_bits(CHUNK_BITS);
const MAX_RLE_RUNS: usize = 1_024;

const _: () = assert!(GETINFO_MAX_BITS == 196_608);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum StreamError {
    Disconnected,
    JtagFailed,
    Timeout,
    ShiftTooLarge,
    TmsRleOverflow,
    TmsUnderflow,
    OutOfMemory,
}

struct TmsRle {
    runs: Vec<(u8, u32)>,
    run_index: usize,
    run_remaining: u32,
}

impl TmsRle {
    fn new() -> Self {
        let mut runs = Vec::new();
        let _ = runs.try_reserve(64);
        Self {
            runs,
            run_index: 0,
            run_remaining: 0,
        }
    }

    fn clear(&mut self) {
        self.runs.clear();
        self.run_index = 0;
        self.run_remaining = 0;
    }

    fn push(&mut self, byte: u8) -> Result<(), StreamError> {
        if let Some((last_byte, last_len)) = self.runs.last_mut()
            && *last_byte == byte
            && *last_len < u32::MAX
        {
            *last_len += 1;
            return Ok(());
        }
        if self.runs.len() >= MAX_RLE_RUNS {
            return Err(StreamError::TmsRleOverflow);
        }
        self.runs
            .try_reserve(1)
            .map_err(|_| StreamError::OutOfMemory)?;
        self.runs.push((byte, 1));
        Ok(())
    }

    fn rewind(&mut self) {
        self.run_index = 0;
        self.run_remaining = 0;
    }

    fn read_into(&mut self, output: &mut [u8]) -> Result<(), StreamError> {
        for output_byte in output {
            if self.run_remaining == 0 {
                let Some((_, run_len)) = self.runs.get(self.run_index) else {
                    return Err(StreamError::TmsUnderflow);
                };
                self.run_remaining = *run_len;
            }

            *output_byte = self.runs[self.run_index].0;
            self.run_remaining -= 1;
            if self.run_remaining == 0 {
                self.run_index += 1;
            }
        }
        Ok(())
    }
}

enum TmsMode {
    Raw { length: usize, position: usize },
    Rle,
}

struct TmsStorage {
    mode: TmsMode,
    rle: TmsRle,
}

impl TmsStorage {
    fn new() -> Self {
        Self {
            mode: TmsMode::Raw {
                length: 0,
                position: 0,
            },
            rle: TmsRle::new(),
        }
    }

    fn begin(&mut self, byte_count: usize) {
        self.rle.clear();
        self.mode = if byte_count <= RAW_TMS_MAX_BYTES {
            TmsMode::Raw {
                length: 0,
                position: 0,
            }
        } else {
            TmsMode::Rle
        };
    }

    fn store(&mut self, raw: &mut [u8], input: &[u8]) -> Result<(), StreamError> {
        match &mut self.mode {
            TmsMode::Raw { length, .. } => {
                let end = length
                    .checked_add(input.len())
                    .ok_or(StreamError::ShiftTooLarge)?;
                if end > raw.len() {
                    return Err(StreamError::ShiftTooLarge);
                }
                raw[*length..end].copy_from_slice(input);
                *length = end;
                Ok(())
            }
            TmsMode::Rle => {
                for &byte in input {
                    self.rle.push(byte)?;
                }
                Ok(())
            }
        }
    }

    fn rewind(&mut self) {
        match &mut self.mode {
            TmsMode::Raw { position, .. } => *position = 0,
            TmsMode::Rle => self.rle.rewind(),
        }
    }

    fn read_into(&mut self, raw: &[u8], output: &mut [u8]) -> Result<(), StreamError> {
        match &mut self.mode {
            TmsMode::Raw { length, position } => {
                let end = position
                    .checked_add(output.len())
                    .ok_or(StreamError::TmsUnderflow)?;
                if end > *length {
                    return Err(StreamError::TmsUnderflow);
                }
                output.copy_from_slice(&raw[*position..end]);
                *position = end;
                Ok(())
            }
            TmsMode::Rle => self.rle.read_into(output),
        }
    }

    fn description(&self) -> StorageDescription {
        match self.mode {
            TmsMode::Raw { length, .. } => StorageDescription::Raw { length },
            TmsMode::Rle => StorageDescription::Rle {
                runs: self.rle.runs.len(),
            },
        }
    }
}

enum StorageDescription {
    Raw { length: usize },
    Rle { runs: usize },
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Slot {
    First,
    Second,
}

impl Slot {
    const fn index(self) -> usize {
        match self {
            Self::First => 0,
            Self::Second => 1,
        }
    }

    const fn other(self) -> Self {
        match self {
            Self::First => Self::Second,
            Self::Second => Self::First,
        }
    }
}

pub(super) struct StreamWorkspace {
    tms_storage: TmsStorage,
    raw_tms: [u8; RAW_TMS_MAX_BYTES],
    tms_chunk: [u8; CHUNK_BYTES],
    tdi_chunk: [u8; CHUNK_BYTES],
    tdo_buffers: [[u8; CHUNK_BYTES]; 2],
}

impl StreamWorkspace {
    pub(super) fn new() -> Self {
        Self {
            tms_storage: TmsStorage::new(),
            raw_tms: [0; RAW_TMS_MAX_BYTES],
            tms_chunk: [0; CHUNK_BYTES],
            tdi_chunk: [0; CHUNK_BYTES],
            tdo_buffers: [[0; CHUNK_BYTES]; 2],
        }
    }
}

fn split_tdo_buffers(
    buffers: &mut [[u8; CHUNK_BYTES]; 2],
    fill: Slot,
) -> (&mut [u8; CHUNK_BYTES], &[u8; CHUNK_BYTES]) {
    let (first, second) = buffers.split_at_mut(1);
    match fill {
        Slot::First => (&mut first[0], &second[0]),
        Slot::Second => (&mut second[0], &first[0]),
    }
}
