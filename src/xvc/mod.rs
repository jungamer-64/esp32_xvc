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

use embassy_net::{Stack, tcp::TcpSocket};
use embassy_time::{Duration, Timer, with_timeout};
use esp_println::println;
use static_cell::{ConstStaticCell, StaticCell};

use crate::{
    jtag::{
        JtagService, JtagShift, MAX_TCK_PERIOD_US, MIN_TCK_PERIOD_US, ShiftExecution,
        bytes_for_bits,
    },
    logging::xvc_log,
};

const XVC_PORT: u16 = 2_542;
const RECEIVE_BUFFER_SIZE: usize = 10 * 1_024;
const RESPONSE_BUFFER_SIZE: usize = wire::BUFFERED_SHIFT_MAX_BYTES;
const TCP_BUFFER_SIZE: usize = 4 * 1_024;
const RECEIVE_PROGRESS_TIMEOUT: Duration = Duration::from_secs(30);
const SEND_PROGRESS_TIMEOUT: Duration = Duration::from_secs(10);
const ABORT_FLUSH_TIMEOUT: Duration = Duration::from_secs(1);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum TransportError {
    Disconnected,
    Timeout,
}

pub(crate) struct XvcServer {
    receive: ReceiveWindow,
    response: &'static mut [u8; RESPONSE_BUFFER_SIZE],
    stream: &'static mut stream::StreamWorkspace,
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
            response: RESPONSE.take(),
            stream: STREAM.init_with(stream::StreamWorkspace::new),
        }
    }

    pub(crate) async fn run(mut self, stack: Stack<'static>, mut jtag: JtagService) -> ! {
        static TCP_RX: ConstStaticCell<[u8; TCP_BUFFER_SIZE]> =
            ConstStaticCell::new([0; TCP_BUFFER_SIZE]);
        static TCP_TX: ConstStaticCell<[u8; TCP_BUFFER_SIZE]> =
            ConstStaticCell::new([0; TCP_BUFFER_SIZE]);
        let mut socket = TcpSocket::new(stack, TCP_RX.take(), TCP_TX.take());
        socket.set_nagle_enabled(false);

        println!("========================================");
        println!("XVC Server Ready on TCP {XVC_PORT}");
        println!("TCK: {MIN_TCK_PERIOD_US}us-{MAX_TCK_PERIOD_US}us");
        println!("========================================");

        loop {
            self.receive.clear();
            jtag.restore_min_period();
            if let Err(error) = socket.accept(XVC_PORT).await {
                println!("XVC accept failed: {error:?}");
                abort_socket(&mut socket).await;
                Timer::after_millis(100).await;
                continue;
            }

            xvc_log!("New client, TAP reset");
            let reset_result = jtag.reset(|| socket.may_recv() && socket.may_send()).await;
            if let Err(error) = reset_result {
                println!("TAP reset failed: {error:?}");
                abort_socket(&mut socket).await;
                continue;
            }

            if let Err(error) = self.run_session(&mut socket, &mut jtag).await {
                println!("XVC session aborted: {error:?}");
            }
            abort_socket(&mut socket).await;
        }
    }

    async fn run_session(
        &mut self,
        socket: &mut TcpSocket<'static>,
        jtag: &mut JtagService,
    ) -> Result<(), SessionError> {
        loop {
            match wire::decode(self.receive.data()) {
                wire::DecodeOutcome::NeedMore => {
                    let Some(space) = self.receive.writable() else {
                        return Err(SessionError::ReceiveOverflow);
                    };
                    let received = read_progress(socket, space).await?;
                    self.receive.commit(received);
                }
                wire::DecodeOutcome::ProtocolError => {
                    self.log_protocol_error();
                    return Err(SessionError::Protocol);
                }
                wire::DecodeOutcome::StreamingShift {
                    bit_count,
                    consumed,
                } => {
                    // Vivado may exceed the advertised limit. Keep every
                    // oversized request on the bounded streaming path.
                    self.receive.consume(consumed);
                    stream::execute(bit_count, &mut self.receive, socket, jtag, self.stream)
                        .await?;
                }
                wire::DecodeOutcome::Command { command, consumed } => match command {
                    wire::Command::GetInfo => {
                        write_all(socket, wire::GETINFO_RESPONSE, SEND_PROGRESS_TIMEOUT).await?;
                        self.receive.consume(consumed);
                    }
                    wire::Command::SetTck {
                        requested_period_ns,
                    } => {
                        let applied = jtag.set_period_ns(requested_period_ns).to_le_bytes();
                        write_all(socket, &applied, SEND_PROGRESS_TIMEOUT).await?;
                        self.receive.consume(consumed);
                    }
                    wire::Command::Shift(shift) => {
                        let byte_count = bytes_for_bits(shift.bit_count);
                        let output = &mut self.response[..byte_count];
                        let request = JtagShift::new(
                            ShiftExecution::BufferedCommand,
                            shift.bit_count,
                            shift.tms,
                            shift.tdi,
                            output,
                        )?;
                        jtag.shift(request, || socket.may_recv() && socket.may_send())
                            .await?;

                        #[cfg(feature = "xvc-log")]
                        xvc_log!(
                            "shift: {} bits TMS={:02x?} TDI={:02x?} -> TDO={:02x?}",
                            shift.bit_count,
                            &shift.tms[..byte_count.min(4)],
                            &shift.tdi[..byte_count.min(4)],
                            &output[..byte_count.min(8)]
                        );

                        write_all(socket, output, SEND_PROGRESS_TIMEOUT).await?;
                        self.receive.consume(consumed);
                    }
                },
            }
        }
    }

    fn log_protocol_error(&self) {
        let data = self.receive.data();
        let log_len = data.len().min(16);
        println!(
            "Protocol Error: first {log_len} bytes: {:02x?} ({:?})",
            &data[..log_len],
            core::str::from_utf8(&data[..log_len]).unwrap_or(".")
        );
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SessionError {
    Transport(TransportError),
    Stream(stream::StreamError),
    Jtag(crate::jtag::JtagError),
    Protocol,
    ReceiveOverflow,
}

impl From<TransportError> for SessionError {
    fn from(error: TransportError) -> Self {
        Self::Transport(error)
    }
}

impl From<stream::StreamError> for SessionError {
    fn from(error: stream::StreamError) -> Self {
        Self::Stream(error)
    }
}

impl From<crate::jtag::JtagError> for SessionError {
    fn from(error: crate::jtag::JtagError) -> Self {
        Self::Jtag(error)
    }
}

pub(super) async fn read_progress(
    socket: &mut TcpSocket<'_>,
    output: &mut [u8],
) -> Result<usize, TransportError> {
    match with_timeout(RECEIVE_PROGRESS_TIMEOUT, socket.read(output)).await {
        Ok(Ok(0)) | Ok(Err(_)) => Err(TransportError::Disconnected),
        Ok(Ok(received)) => Ok(received),
        Err(_) => Err(TransportError::Timeout),
    }
}

pub(super) async fn write_all(
    socket: &mut TcpSocket<'_>,
    data: &[u8],
    progress_timeout: Duration,
) -> Result<(), TransportError> {
    let mut sent = 0;
    while sent < data.len() {
        match with_timeout(progress_timeout, socket.write(&data[sent..])).await {
            Ok(Ok(0)) | Ok(Err(_)) => return Err(TransportError::Disconnected),
            Ok(Ok(count)) => sent += count,
            Err(_) => return Err(TransportError::Timeout),
        }
    }
    Ok(())
}

async fn abort_socket(socket: &mut TcpSocket<'_>) {
    socket.abort();
    let _ = with_timeout(ABORT_FLUSH_TIMEOUT, socket.flush()).await;
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
