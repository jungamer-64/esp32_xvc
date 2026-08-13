//! XVC TCP session, wire protocol, and shift orchestration.
//!
//! # CRITICAL: LARGE-SHIFT STREAMING MUST NOT BE REMOVED
//!
//! Vivado can ignore the `getinfo:` shift-size advertisement and send a shift
//! larger than the normal receive buffer. A fixed-buffer-only implementation
//! is therefore not Vivado compatible. Every oversized shift must remain on
//! `stream` or a functionally equivalent bounded-memory path.
//!
//! Any replacement must preserve TMS retention or reconstruction, incremental
//! JTAG execution, incremental TDO delivery, timeouts, disconnect aborts, and
//! Core0/Core1 buffer-lifetime synchronization.

mod stream;
mod wire;

use esp_println::println;
use static_cell::{ConstStaticCell, StaticCell};

use crate::{
    config::NetworkConfig,
    jtag::{
        JtagService, JtagShift, MAX_TCK_PERIOD_US, MIN_TCK_PERIOD_US, ShiftExecution,
        bytes_for_bits,
    },
    logging::xvc_log,
    network::{ConnectionState, LinkEvent, Network},
    runtime::{self, Clock},
};

const XVC_PORT: u16 = 2_542;
const RECEIVE_BUFFER_SIZE: usize = 10 * 1_024;
const RESPONSE_BUFFER_SIZE: usize = wire::BUFFERED_SHIFT_MAX_BYTES;
const RESPONSE_TIMEOUT_MS: i64 = 10_000;

pub(crate) struct XvcServer {
    receive: ReceiveWindow,
    response: ResponseBuffer,
    stream: &'static mut stream::StreamWorkspace,
    previous_connection_state: ConnectionState,
}

impl XvcServer {
    pub(crate) fn new() -> Self {
        static RECEIVE: ConstStaticCell<[u8; RECEIVE_BUFFER_SIZE]> =
            ConstStaticCell::new([0; RECEIVE_BUFFER_SIZE]);
        static RESPONSE: ConstStaticCell<[u8; RESPONSE_BUFFER_SIZE]> =
            ConstStaticCell::new([0; RESPONSE_BUFFER_SIZE]);
        static STREAM: StaticCell<stream::StreamWorkspace> = StaticCell::new();

        Self {
            receive: ReceiveWindow::new(RECEIVE.take()),
            response: ResponseBuffer::new(RESPONSE.take()),
            stream: STREAM.init_with(stream::StreamWorkspace::new),
            previous_connection_state: ConnectionState::Closed,
        }
    }

    pub(crate) fn run(
        mut self,
        network_config: NetworkConfig,
        network: &mut Network<'_>,
        jtag: &mut JtagService,
        clock: &Clock,
    ) -> ! {
        let address = network_config.address().octets();
        println!("========================================");
        println!("XVC Server Ready!");
        println!(
            "IP: {}.{}.{}.{}:{}",
            address[0], address[1], address[2], address[3], XVC_PORT
        );
        println!("TCK: {}us-{}us", MIN_TCK_PERIOD_US, MAX_TCK_PERIOD_US);
        println!("========================================");

        loop {
            if network.maintain_link(clock) == LinkEvent::SessionInvalidated {
                self.abort_session(network, jtag, clock);
            }
            network.poll(clock);

            if !network.is_open() {
                if !matches!(
                    self.previous_connection_state,
                    ConnectionState::Closed | ConnectionState::Listening
                ) {
                    xvc_log!("Re-listening...");
                }
                self.clear_session();
                if let Err(error) = network.listen(XVC_PORT) {
                    println!("Listen error: {:?}", error);
                }
            }

            let connection_state = network.state();
            let newly_connected = connection_state == ConnectionState::Established
                && self.previous_connection_state != ConnectionState::Established;
            if newly_connected {
                network.disable_nagle();
                self.clear_session();
                jtag.restore_min_period();
                xvc_log!("New client, TAP reset");

                let reset_result = jtag.reset(clock, || {
                    network.poll(clock);
                    network.connection_alive()
                });
                if let Err(error) = reset_result {
                    println!("Reset failed: {:?}", error);
                    self.abort_session(network, jtag, clock);
                } else {
                    xvc_log!("New connection initialized");
                }
            }
            self.previous_connection_state = network.state();

            if self.flush_response(network, clock).is_err() {
                self.abort_session(network, jtag, clock);
            }
            if self.response.timed_out(clock) {
                println!("Pending timeout, aborting connection");
                self.abort_session(network, jtag, clock);
            }

            if !self.response.is_pending() && network.can_receive() {
                let Some(space) = self.receive.writable() else {
                    println!("Error: RX buffer overflow, aborting connection");
                    self.abort_session(network, jtag, clock);
                    continue;
                };

                match network.receive(space) {
                    Ok(0) if !network.may_receive() => {
                        xvc_log!("Connection closed by peer");
                        network.close();
                    }
                    Ok(received) => self.receive.commit(received),
                    Err(_) => self.abort_session(network, jtag, clock),
                }
            }
            self.previous_connection_state = network.state();

            if self.process_commands(network, jtag, clock) == ProcessResult::Abort {
                println!("Aborting connection due to protocol or execution error");
                self.abort_session(network, jtag, clock);
            }

            let has_work = self.previous_connection_state == ConnectionState::Established
                && (!self.receive.is_empty() || self.response.is_pending());
            if has_work {
                runtime::yield_now();
            } else {
                runtime::delay_ms(1);
            }
        }
    }

    fn process_commands(
        &mut self,
        network: &mut Network<'_>,
        jtag: &mut JtagService,
        clock: &Clock,
    ) -> ProcessResult {
        while !self.receive.is_empty() && !self.response.is_pending() {
            match wire::decode(self.receive.data()) {
                wire::DecodeOutcome::NeedMore => return ProcessResult::Continue,
                wire::DecodeOutcome::ProtocolError => {
                    self.log_protocol_error();
                    return ProcessResult::Abort;
                }
                wire::DecodeOutcome::StreamingShift {
                    bit_count,
                    consumed,
                } => {
                    // COMPATIBILITY INVARIANT: Vivado may ignore `getinfo:`.
                    // Never reject this solely because it exceeds the buffered
                    // path; dispatch it to the bounded streaming implementation.
                    self.receive.consume(consumed);
                    if let Err(error) = stream::execute(
                        bit_count,
                        &mut self.receive,
                        network,
                        jtag,
                        clock,
                        self.stream,
                    ) {
                        println!("Stream failed: {:?}", error);
                        return ProcessResult::Abort;
                    }
                }
                wire::DecodeOutcome::Command { command, consumed } => match command {
                    wire::Command::GetInfo => {
                        self.response.write(wire::GETINFO_RESPONSE, clock);
                        self.receive.consume(consumed);
                    }
                    wire::Command::SetTck {
                        requested_period_ns,
                    } => {
                        let applied = jtag.set_period_ns(requested_period_ns).to_le_bytes();
                        self.response.write(&applied, clock);
                        self.receive.consume(consumed);
                    }
                    wire::Command::Shift(shift) => {
                        let byte_count = bytes_for_bits(shift.bit_count);
                        let output = self.response.output(byte_count);
                        let request = match JtagShift::new(
                            ShiftExecution::BufferedCommand,
                            shift.bit_count,
                            shift.tms,
                            shift.tdi,
                            output,
                        ) {
                            Ok(request) => request,
                            Err(error) => {
                                println!("Invalid JTAG shift: {:?}", error);
                                return ProcessResult::Abort;
                            }
                        };
                        if let Err(error) = jtag.shift(request, || {
                            network.poll(clock);
                            network.connection_alive()
                        }) {
                            println!("Shift failed: {:?}", error);
                            return ProcessResult::Abort;
                        }

                        #[cfg(feature = "xvc-log")]
                        xvc_log!(
                            "shift: {} bits TMS={:02x?} TDI={:02x?} -> TDO={:02x?}",
                            shift.bit_count,
                            &shift.tms[..byte_count.min(4)],
                            &shift.tdi[..byte_count.min(4)],
                            &self.response.storage[..byte_count.min(8)]
                        );

                        self.response.queue(byte_count, clock);
                        self.receive.consume(consumed);
                    }
                },
            }
        }
        ProcessResult::Continue
    }

    fn flush_response(&mut self, network: &mut Network<'_>, clock: &Clock) -> Result<(), ()> {
        while self.response.is_pending() && network.can_send() {
            let sent = network
                .send(self.response.pending_slice())
                .map_err(|_| ())?;
            if sent == 0 {
                break;
            }
            self.response.advance(sent, clock);
        }
        Ok(())
    }

    fn abort_session(&mut self, network: &mut Network<'_>, jtag: &JtagService, clock: &Clock) {
        jtag.abort_and_wait(clock);
        network.abort();
        self.clear_session();
    }

    fn clear_session(&mut self) {
        self.receive.clear();
        self.response.clear();
    }

    fn log_protocol_error(&self) {
        let data = self.receive.data();
        let log_len = data.len().min(16);
        println!(
            "Protocol Error: Unknown command. First {} bytes: {:02x?} ({:?})",
            log_len,
            &data[..log_len],
            core::str::from_utf8(&data[..log_len]).unwrap_or(".")
        );
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ProcessResult {
    Continue,
    Abort,
}

pub(super) struct ReceiveWindow {
    storage: &'static mut [u8; RECEIVE_BUFFER_SIZE],
    head: usize,
    length: usize,
}

impl ReceiveWindow {
    fn new(storage: &'static mut [u8; RECEIVE_BUFFER_SIZE]) -> Self {
        Self {
            storage,
            head: 0,
            length: 0,
        }
    }

    fn data(&self) -> &[u8] {
        &self.storage[self.head..self.head + self.length]
    }

    fn is_empty(&self) -> bool {
        self.length == 0
    }

    fn writable(&mut self) -> Option<&mut [u8]> {
        if self.length == 0 {
            self.head = 0;
        }

        let mut write_position = self.head + self.length;
        if write_position == self.storage.len() && self.head > 0 {
            self.storage
                .copy_within(self.head..self.head + self.length, 0);
            self.head = 0;
            write_position = self.length;
        }

        (write_position < self.storage.len()).then(|| &mut self.storage[write_position..])
    }

    fn commit(&mut self, received: usize) {
        debug_assert!(self.head + self.length + received <= self.storage.len());
        self.length += received;
    }

    fn consume(&mut self, consumed: usize) {
        debug_assert!(consumed <= self.length);
        self.head += consumed;
        self.length -= consumed;
        if self.length == 0 {
            self.head = 0;
        }
    }

    pub(super) fn take_into(&mut self, output: &mut [u8]) -> usize {
        let copied = output.len().min(self.length);
        output[..copied].copy_from_slice(&self.data()[..copied]);
        self.consume(copied);
        copied
    }

    fn clear(&mut self) {
        self.head = 0;
        self.length = 0;
    }
}

struct PendingResponse {
    length: usize,
    sent: usize,
    last_progress_ms: i64,
}

struct ResponseBuffer {
    storage: &'static mut [u8; RESPONSE_BUFFER_SIZE],
    pending: Option<PendingResponse>,
}

impl ResponseBuffer {
    fn new(storage: &'static mut [u8; RESPONSE_BUFFER_SIZE]) -> Self {
        Self {
            storage,
            pending: None,
        }
    }

    fn output(&mut self, length: usize) -> &mut [u8] {
        debug_assert!(length <= self.storage.len());
        &mut self.storage[..length]
    }

    fn write(&mut self, data: &[u8], clock: &Clock) {
        debug_assert!(data.len() <= self.storage.len());
        self.storage[..data.len()].copy_from_slice(data);
        self.queue(data.len(), clock);
    }

    fn queue(&mut self, length: usize, clock: &Clock) {
        self.pending = (length > 0).then_some(PendingResponse {
            length,
            sent: 0,
            last_progress_ms: clock.now_ms(),
        });
    }

    fn is_pending(&self) -> bool {
        self.pending.is_some()
    }

    fn pending_slice(&self) -> &[u8] {
        match self.pending {
            Some(ref pending) => &self.storage[pending.sent..pending.length],
            None => &[],
        }
    }

    fn advance(&mut self, sent: usize, clock: &Clock) {
        let Some(pending) = &mut self.pending else {
            return;
        };
        pending.sent = pending.sent.saturating_add(sent).min(pending.length);
        pending.last_progress_ms = clock.now_ms();
        if pending.sent == pending.length {
            self.pending = None;
        }
    }

    fn timed_out(&self, clock: &Clock) -> bool {
        self.pending.as_ref().is_some_and(|pending| {
            clock.now_ms().saturating_sub(pending.last_progress_ms) > RESPONSE_TIMEOUT_MS
        })
    }

    fn clear(&mut self) {
        self.pending = None;
    }
}
