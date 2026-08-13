//! Bounded-memory large-shift execution required for Vivado compatibility.

use alloc::vec::Vec;

use esp_println::println;

use crate::{
    jtag::{JtagService, JtagShift, ShiftExecution, bytes_for_bits},
    logging::xvc_log,
    network::Network,
    runtime::{self, Clock},
};

use super::ReceiveWindow;

pub(super) const ABSOLUTE_MAX_BITS: usize = 2_097_152;
const RAW_TMS_MAX_BYTES: usize = 24 * 1_024;
const GETINFO_MAX_BITS: usize = RAW_TMS_MAX_BYTES * 8;
const CHUNK_BITS: usize = 2_048;
const CHUNK_BYTES: usize = bytes_for_bits(CHUNK_BITS);
const MAX_RLE_RUNS: usize = 1_024;
const SEND_TIMEOUT_MS: i64 = 30_000;
const RECEIVE_TIMEOUT_MS: i64 = 30_000;

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

#[derive(Clone, Copy)]
struct PendingTdo {
    slot: Slot,
    length: usize,
    sent: usize,
}

impl PendingTdo {
    fn remaining(self) -> core::ops::Range<usize> {
        self.sent..self.length
    }
}

enum TdoQueue {
    Empty {
        fill: Slot,
    },
    One {
        pending: PendingTdo,
        fill: Slot,
    },
    Two {
        oldest: PendingTdo,
        newest: PendingTdo,
    },
}

impl TdoQueue {
    const fn new() -> Self {
        Self::Empty { fill: Slot::First }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn pending_count(&self) -> usize {
        match self {
            Self::Empty { .. } => 0,
            Self::One { .. } => 1,
            Self::Two { .. } => 2,
        }
    }

    fn fill_slot(&self) -> Option<Slot> {
        match self {
            Self::Empty { fill } | Self::One { fill, .. } => Some(*fill),
            Self::Two { .. } => None,
        }
    }

    fn enqueue(&mut self, slot: Slot, length: usize) {
        let new_pending = PendingTdo {
            slot,
            length,
            sent: 0,
        };
        *self = match *self {
            Self::Empty { fill } => {
                debug_assert!(fill == slot);
                Self::One {
                    pending: new_pending,
                    fill: slot.other(),
                }
            }
            Self::One { pending, fill } => {
                debug_assert!(fill == slot);
                Self::Two {
                    oldest: pending,
                    newest: new_pending,
                }
            }
            Self::Two { .. } => unreachable!("both TDO slots are already pending"),
        };
    }

    fn oldest(&self) -> Option<PendingTdo> {
        match self {
            Self::Empty { .. } => None,
            Self::One { pending, .. } => Some(*pending),
            Self::Two { oldest, .. } => Some(*oldest),
        }
    }

    fn advance(&mut self, sent: usize) {
        *self = match *self {
            Self::Empty { fill } => Self::Empty { fill },
            Self::One { mut pending, fill } => {
                pending.sent = pending.sent.saturating_add(sent).min(pending.length);
                if pending.sent == pending.length {
                    Self::Empty { fill }
                } else {
                    Self::One { pending, fill }
                }
            }
            Self::Two { mut oldest, newest } => {
                oldest.sent = oldest.sent.saturating_add(sent).min(oldest.length);
                if oldest.sent == oldest.length {
                    Self::One {
                        pending: newest,
                        fill: oldest.slot,
                    }
                } else {
                    Self::Two { oldest, newest }
                }
            }
        };
    }
}

pub(super) struct StreamWorkspace {
    tms_storage: TmsStorage,
    raw_tms: [u8; RAW_TMS_MAX_BYTES],
    tms_chunk: [u8; CHUNK_BYTES],
    tdi_chunk: [u8; CHUNK_BYTES],
    tdo_buffers: [[u8; CHUNK_BYTES]; 2],
    tdo_queue: TdoQueue,
}

impl StreamWorkspace {
    pub(super) fn new() -> Self {
        Self {
            tms_storage: TmsStorage::new(),
            raw_tms: [0; RAW_TMS_MAX_BYTES],
            tms_chunk: [0; CHUNK_BYTES],
            tdi_chunk: [0; CHUNK_BYTES],
            tdo_buffers: [[0; CHUNK_BYTES]; 2],
            tdo_queue: TdoQueue::new(),
        }
    }
}

enum DrainResult {
    Progress,
    Blocked,
}

pub(super) fn execute(
    bit_count: usize,
    receive: &mut ReceiveWindow,
    network: &mut Network<'_>,
    jtag: &mut JtagService,
    clock: &Clock,
    workspace: &mut StreamWorkspace,
) -> Result<(), StreamError> {
    if bit_count > ABSOLUTE_MAX_BITS {
        println!(
            "Shift too large: {} bits (max {})",
            bit_count, ABSOLUTE_MAX_BITS
        );
        return Err(StreamError::ShiftTooLarge);
    }

    let byte_count = bytes_for_bits(bit_count);
    let started_at = clock.now_ms();
    xvc_log!("Stream start: {} bits ({} bytes)", bit_count, byte_count);
    workspace.tms_storage.begin(byte_count);
    workspace.tdo_queue.reset();

    let mut temporary = [0_u8; CHUNK_BYTES];
    let mut tms_tail = [0_u8; 4];
    let mut tms_seen = 0_usize;
    let mut remaining = byte_count;
    while remaining > 0 {
        let chunk_len = remaining.min(CHUNK_BYTES);
        read_exact_mix(&mut temporary[..chunk_len], receive, network, clock)?;

        for &byte in &temporary[..chunk_len] {
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
            .store(&mut workspace.raw_tms, &temporary[..chunk_len])?;
        remaining -= chunk_len;
    }
    workspace.tms_storage.rewind();

    let tail_len = byte_count.min(tms_tail.len());
    match workspace.tms_storage.description() {
        StorageDescription::Raw { length } => xvc_log!(
            "Stream: {} bits, static RAW mode ({} bytes), TMS tail={:02x?}",
            bit_count,
            length,
            &tms_tail[..tail_len]
        ),
        StorageDescription::Rle { runs } => xvc_log!(
            "Stream: {} bits, {} RLE runs, TMS tail={:02x?}",
            bit_count,
            runs,
            &tms_tail[..tail_len]
        ),
    }

    let mut bits_remaining = bit_count;
    let mut total_processed = 0_usize;
    let mut last_send_progress_ms = clock.now_ms();
    while bits_remaining > 0 {
        while workspace.tdo_queue.pending_count() >= 2 {
            network.poll(clock);
            match drain_tdo(network, &mut workspace.tdo_queue, &workspace.tdo_buffers)? {
                DrainResult::Progress => last_send_progress_ms = clock.now_ms(),
                DrainResult::Blocked => {}
            }
            if workspace.tdo_queue.pending_count() >= 2 {
                ensure_send_progress(clock, last_send_progress_ms)?;
                runtime::yield_now();
            }
        }

        if let DrainResult::Progress =
            drain_tdo(network, &mut workspace.tdo_queue, &workspace.tdo_buffers)?
        {
            last_send_progress_ms = clock.now_ms();
        }

        let chunk_bits = bits_remaining.min(CHUNK_BITS);
        let chunk_bytes = bytes_for_bits(chunk_bits);
        workspace
            .tms_storage
            .read_into(&workspace.raw_tms, &mut workspace.tms_chunk[..chunk_bytes])?;
        read_exact_mix(
            &mut workspace.tdi_chunk[..chunk_bytes],
            receive,
            network,
            clock,
        )?;

        let fill_slot = workspace
            .tdo_queue
            .fill_slot()
            .expect("a TDO slot was drained before starting JTAG");
        let (current_tdo, other_tdo) = split_tdo_buffers(&mut workspace.tdo_buffers, fill_slot);
        let mut polling_error = None;
        let shift = JtagShift::new(
            ShiftExecution::StreamChunk,
            chunk_bits,
            &workspace.tms_chunk[..chunk_bytes],
            &workspace.tdi_chunk[..chunk_bytes],
            &mut current_tdo[..chunk_bytes],
        )
        .map_err(|_| StreamError::JtagFailed)?;
        let shift_result = jtag.shift(shift, || {
            network.poll(clock);
            if let Err(error) = drain_other_tdo(
                network,
                &mut workspace.tdo_queue,
                fill_slot.other(),
                other_tdo,
            ) {
                polling_error = Some(error);
                return false;
            }
            network.connection_alive()
        });
        if let Some(error) = polling_error {
            return Err(error);
        }
        shift_result.map_err(|_| StreamError::JtagFailed)?;
        workspace.tdo_queue.enqueue(fill_slot, chunk_bytes);

        bits_remaining -= chunk_bits;
        total_processed += chunk_bits;
        if total_processed % 65_536 < CHUNK_BITS {
            xvc_log!("Stream: {}/{} bits", total_processed, bit_count);
        }
    }

    flush_tdo(
        network,
        &mut workspace.tdo_queue,
        &workspace.tdo_buffers,
        clock,
    )?;
    let elapsed_ms = clock.now_ms().saturating_sub(started_at);
    let bits_per_second = if elapsed_ms > 0 {
        (bit_count as u64).saturating_mul(1_000) / elapsed_ms as u64
    } else {
        0
    };
    xvc_log!(
        "Stream complete: {} bits in {}ms ({} kbit/s, TCK={}ns)",
        bit_count,
        elapsed_ms,
        bits_per_second / 1_000,
        jtag.period_ns()
    );
    Ok(())
}

fn read_exact_mix(
    output: &mut [u8],
    receive: &mut ReceiveWindow,
    network: &mut Network<'_>,
    clock: &Clock,
) -> Result<(), StreamError> {
    let copied = receive.take_into(output);
    if copied < output.len() {
        receive_exact(network, &mut output[copied..], clock)?;
    }
    Ok(())
}

fn receive_exact(
    network: &mut Network<'_>,
    output: &mut [u8],
    clock: &Clock,
) -> Result<(), StreamError> {
    let mut received = 0_usize;
    let mut last_progress_ms = clock.now_ms();
    while received < output.len() {
        network.poll(clock);
        if !network.may_receive() {
            return Err(StreamError::Disconnected);
        }
        if !network.can_receive() {
            if clock.now_ms().saturating_sub(last_progress_ms) > RECEIVE_TIMEOUT_MS {
                return Err(StreamError::Timeout);
            }
            runtime::yield_now();
            continue;
        }

        let count = network
            .receive(&mut output[received..])
            .map_err(|_| StreamError::Disconnected)?;
        if count == 0 {
            if !network.may_receive() {
                return Err(StreamError::Disconnected);
            }
            runtime::yield_now();
        } else {
            received += count;
            last_progress_ms = clock.now_ms();
        }
    }
    Ok(())
}

fn drain_tdo(
    network: &mut Network<'_>,
    queue: &mut TdoQueue,
    buffers: &[[u8; CHUNK_BYTES]; 2],
) -> Result<DrainResult, StreamError> {
    let Some(pending) = queue.oldest() else {
        return Ok(DrainResult::Blocked);
    };
    drain_pending(network, queue, pending, &buffers[pending.slot.index()])
}

fn drain_other_tdo(
    network: &mut Network<'_>,
    queue: &mut TdoQueue,
    available_slot: Slot,
    available_buffer: &[u8; CHUNK_BYTES],
) -> Result<DrainResult, StreamError> {
    let Some(pending) = queue.oldest() else {
        return Ok(DrainResult::Blocked);
    };
    debug_assert!(pending.slot == available_slot);
    drain_pending(network, queue, pending, available_buffer)
}

fn drain_pending(
    network: &mut Network<'_>,
    queue: &mut TdoQueue,
    pending: PendingTdo,
    buffer: &[u8; CHUNK_BYTES],
) -> Result<DrainResult, StreamError> {
    if !network.may_send() {
        return Err(StreamError::Disconnected);
    }
    if !network.can_send() {
        return Ok(DrainResult::Blocked);
    }

    let sent = network
        .send(&buffer[pending.remaining()])
        .map_err(|_| StreamError::Disconnected)?;
    if sent == 0 {
        Ok(DrainResult::Blocked)
    } else {
        queue.advance(sent);
        Ok(DrainResult::Progress)
    }
}

fn flush_tdo(
    network: &mut Network<'_>,
    queue: &mut TdoQueue,
    buffers: &[[u8; CHUNK_BYTES]; 2],
    clock: &Clock,
) -> Result<(), StreamError> {
    let mut last_progress_ms = clock.now_ms();
    while queue.pending_count() > 0 {
        network.poll(clock);
        if let DrainResult::Progress = drain_tdo(network, queue, buffers)? {
            last_progress_ms = clock.now_ms();
        }
        if queue.pending_count() > 0 {
            ensure_send_progress(clock, last_progress_ms)?;
            runtime::yield_now();
        }
    }
    Ok(())
}

fn ensure_send_progress(clock: &Clock, last_progress_ms: i64) -> Result<(), StreamError> {
    if clock.now_ms().saturating_sub(last_progress_ms) > SEND_TIMEOUT_MS {
        println!("Stream: TDO send stalled, client not reading");
        Err(StreamError::Timeout)
    } else {
        Ok(())
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
