//! Writer that integrates tracing output with indicatif progress bars
//!
//! This module provides a custom writer that ensures tracing logs
//! are printed above progress bars without interfering with them.

use indicatif::MultiProgress;
use std::io::{self, Write};
use std::sync::Arc;
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

impl Write for ProgressWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        // Suspend progress bars and write directly to stderr
        self.multi.suspend(|| io::stderr().write(buf))
    }

    fn flush(&mut self) -> io::Result<()> {
        self.multi.suspend(|| io::stderr().flush())
    }
}

/// MakeWriter implementation for ProgressWriter
#[derive(Clone)]
pub struct ProgressMakeWriter {
    multi: Arc<MultiProgress>,
}

impl ProgressMakeWriter {
    pub fn new(multi: MultiProgress) -> Self {
        Self {
            multi: Arc::new(multi),
        }
    }
}

impl<'a> MakeWriter<'a> for ProgressMakeWriter {
    type Writer = ProgressWriter;

    fn make_writer(&'a self) -> Self::Writer {
        ProgressWriter::new((*self.multi).clone())
    }
}

/// Make a writer factory for tracing that works with progress bars
pub fn make_writer(multi: MultiProgress) -> ProgressMakeWriter {
    ProgressMakeWriter::new(multi)
}
