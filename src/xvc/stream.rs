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
use core::cell::Cell;

use embassy_futures::join::join;
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Instant};
use esp_println::println;

use crate::{
    jtag::{JtagService, JtagShift, ShiftExecution, bytes_for_bits},
    logging::xvc_log,
};

use super::{ReceiveWindow, TransportError, read_progress, write_all};

pub(super) const ABSOLUTE_MAX_BITS: usize = 2_097_152;
const RAW_TMS_MAX_BYTES: usize = 24 * 1_024;
const GETINFO_MAX_BITS: usize = RAW_TMS_MAX_BYTES * 8;
const CHUNK_BITS: usize = 2_048;
const CHUNK_BYTES: usize = bytes_for_bits(CHUNK_BITS);
const MAX_RLE_RUNS: usize = 1_024;
const STREAM_PROGRESS_TIMEOUT: Duration = Duration::from_secs(30);

const _: () = assert!(GETINFO_MAX_BITS == 196_608);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum StreamError {
    Transport(TransportError),
    JtagFailed,
    ShiftTooLarge,
    TmsRleOverflow,
    TmsUnderflow,
    OutOfMemory,
}

impl From<TransportError> for StreamError {
    fn from(error: TransportError) -> Self {
        Self::Transport(error)
    }
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

#[derive(Clone, Copy)]
struct PendingTdo {
    slot: Slot,
    length: usize,
}

pub(super) async fn execute(
    bit_count: usize,
    receive: &mut ReceiveWindow,
    socket: &mut TcpSocket<'_>,
    jtag: &mut JtagService,
    workspace: &mut StreamWorkspace,
) -> Result<(), StreamError> {
    if bit_count > ABSOLUTE_MAX_BITS {
        println!("Shift too large: {bit_count} bits (max {ABSOLUTE_MAX_BITS})");
        return Err(StreamError::ShiftTooLarge);
    }

    let byte_count = bytes_for_bits(bit_count);
    let started_at = Instant::now();
    xvc_log!("Stream start: {bit_count} bits ({byte_count} bytes)");
    workspace.tms_storage.begin(byte_count);

    let mut tms_tail = [0_u8; 4];
    let mut tms_seen = 0_usize;
    let mut remaining = byte_count;
    while remaining > 0 {
        let chunk_len = remaining.min(CHUNK_BYTES);
        read_exact_mix(&mut workspace.tms_chunk[..chunk_len], receive, socket).await?;

        for &byte in &workspace.tms_chunk[..chunk_len] {
            if tms_seen < tms_tail.len() {
                tms_tail[tms_seen] = byte;
            } else {
                tms_tail.rotate_left(1);
                tms_tail[3] = byte;
            }
            tms_seen += 1;
        }
        workspace
            .tms_storage
            .store(&mut workspace.raw_tms, &workspace.tms_chunk[..chunk_len])?;
        remaining -= chunk_len;
    }
    workspace.tms_storage.rewind();

    let tail_len = byte_count.min(tms_tail.len());
    match workspace.tms_storage.description() {
        StorageDescription::Raw { length } => xvc_log!(
            "Stream: {bit_count} bits, static RAW mode ({length} bytes), TMS tail={:02x?}",
            &tms_tail[..tail_len]
        ),
        StorageDescription::Rle { runs } => xvc_log!(
            "Stream: {bit_count} bits, {runs} RLE runs, TMS tail={:02x?}",
            &tms_tail[..tail_len]
        ),
    }

    let mut bits_remaining = bit_count;
    let mut total_processed = 0_usize;
    let mut pending: Option<PendingTdo> = None;
    let mut fill_slot = Slot::First;

    while bits_remaining > 0 {
        let chunk_bits = bits_remaining.min(CHUNK_BITS);
        let chunk_bytes = bytes_for_bits(chunk_bits);
        workspace
            .tms_storage
            .read_into(&workspace.raw_tms, &mut workspace.tms_chunk[..chunk_bytes])?;
        read_exact_mix(&mut workspace.tdi_chunk[..chunk_bytes], receive, socket).await?;

        let (current_tdo, previous_tdo) = split_tdo_buffers(&mut workspace.tdo_buffers, fill_slot);
        let shift = JtagShift::new(
            ShiftExecution::StreamChunk,
            chunk_bits,
            &workspace.tms_chunk[..chunk_bytes],
            &workspace.tdi_chunk[..chunk_bytes],
            &mut current_tdo[..chunk_bytes],
        )
        .map_err(|_| StreamError::JtagFailed)?;

        if let Some(previous) = pending {
            debug_assert!(previous.slot == fill_slot.other());
            let alive = Cell::new(socket.may_recv() && socket.may_send());
            let shift_future = jtag.shift(shift, || alive.get());
            let send_future = send_with_liveness(socket, &previous_tdo[..previous.length], &alive);
            let (shift_result, send_result) = join(shift_future, send_future).await;
            send_result?;
            shift_result.map_err(|_| StreamError::JtagFailed)?;
        } else {
            jtag.shift(shift, || socket.may_recv() && socket.may_send())
                .await
                .map_err(|_| StreamError::JtagFailed)?;
        }

        pending = Some(PendingTdo {
            slot: fill_slot,
            length: chunk_bytes,
        });
        fill_slot = fill_slot.other();
        bits_remaining -= chunk_bits;
        total_processed += chunk_bits;
        if total_processed % 65_536 < CHUNK_BITS {
            xvc_log!("Stream: {total_processed}/{bit_count} bits");
        }
    }

    if let Some(final_tdo) = pending {
        write_all(
            socket,
            &workspace.tdo_buffers[final_tdo.slot.index()][..final_tdo.length],
            STREAM_PROGRESS_TIMEOUT,
        )
        .await?;
    }

    let elapsed_ms = started_at.elapsed().as_millis();
    let bits_per_second = if elapsed_ms > 0 {
        (bit_count as u64).saturating_mul(1_000) / elapsed_ms
    } else {
        0
    };
    xvc_log!(
        "Stream complete: {bit_count} bits in {elapsed_ms}ms ({} kbit/s, TCK={}ns)",
        bits_per_second / 1_000,
        jtag.period_ns()
    );
    Ok(())
}

async fn read_exact_mix(
    output: &mut [u8],
    receive: &mut ReceiveWindow,
    socket: &mut TcpSocket<'_>,
) -> Result<(), StreamError> {
    let mut received = receive.take_into(output);
    while received < output.len() {
        received += read_progress(socket, &mut output[received..]).await?;
    }
    Ok(())
}

async fn send_with_liveness(
    socket: &mut TcpSocket<'_>,
    data: &[u8],
    alive: &Cell<bool>,
) -> Result<(), StreamError> {
    let result = write_all(socket, data, STREAM_PROGRESS_TIMEOUT).await;
    if result.is_err() {
        alive.set(false);
    }
    result.map_err(StreamError::from)
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
