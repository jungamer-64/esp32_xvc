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

use static_cell::{ConstStaticCell, StaticCell};

const XVC_PORT: u16 = 2_542;
const RECEIVE_BUFFER_SIZE: usize = 10 * 1_024;
const RESPONSE_BUFFER_SIZE: usize = wire::BUFFERED_SHIFT_MAX_BYTES;

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
