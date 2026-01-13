//! Writer that integrates tracing output with indicatif progress bars
//!
//! This module provides a custom writer that ensures tracing logs
//! are printed above progress bars without interfering with them.

use indicatif::MultiProgress;
use std::io::{self, Write};
use tracing_subscriber::fmt::MakeWriter;

/// Writer that suspends progress bars while writing logs
pub struct ProgressWriter {
    multi: MultiProgress,
}

impl ProgressWriter {
    /// Create a new progress writer
    pub fn new(multi: MultiProgress) -> Self {
        Self { multi }
    }
}

impl MakeWriter<'_> for ProgressWriter {
    type Writer = Self;

    fn make_writer(&'_ self) -> Self::Writer {
        Self {
            multi: self.multi.clone(),
        }
    }
}

impl Write for ProgressWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        // Suspend progress bars and write directly to stderr
        self.multi.suspend(|| io::stderr().lock().write(buf))
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }

    // fn write_vectored(&mut self, bufs: &[io::IoSlice<'_>]) -> io::Result<usize> {
    //     self.multi.suspend(|| io::stderr().write_vectored(bufs))
    // }

    // fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
    //     self.multi.suspend(|| io::stderr().write_all(buf))
    // }

    // fn write_fmt(&mut self, args: std::fmt::Arguments<'_>) -> io::Result<()> {
    //     self.multi.suspend(|| io::stderr().write_fmt(args))
    // }
}
