//! Command execution with log capture and progress display
//!
//! This module provides utilities for running build commands while capturing
//! their output and displaying it through indicatif progress bars and tracing.

use std::io::{BufRead, BufReader};
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread;

use crate::Result;

/// Callback for log lines
pub type LogCallback = Arc<dyn Fn(&str) + Send + Sync>;

/// Run a command with log capture
///
/// This function runs a command and captures its stdout and stderr.
/// Each line is:
/// 1. Logged via tracing
/// 2. Passed to the callback for progress display
///
/// # Arguments
/// * `command` - The command to execute
/// * `package_name` - Name of the package being built (for error messages)
/// * `operation` - Description of the operation (e.g., "CMake configure")
/// * `log_callback` - Optional callback to receive log lines
pub fn run_command_with_logging(
    command: &mut Command,
    package_name: &str,
    operation: &str,
    log_callback: Option<LogCallback>,
) -> Result<()> {
    // Configure command to capture output
    command.stdout(Stdio::piped()).stderr(Stdio::piped());

    tracing::debug!("Running command: {:?}", command);

    // Spawn the process
    let mut child = command.spawn().map_err(|e| {
        crate::Error::build(
            format!("{} failed to start for {}", operation, package_name),
            &e.to_string(),
        )
    })?;

    // Take ownership of stdout and stderr
    let stdout = child.stdout.take().expect("Failed to capture stdout");
    let stderr = child.stderr.take().expect("Failed to capture stderr");

    // Shared state for the last log line
    let last_line = Arc::new(Mutex::new(String::new()));

    // Spawn thread to read stdout
    let last_line_stdout = Arc::clone(&last_line);
    let log_callback_stdout = log_callback.clone();
    let package_name_stdout = package_name.to_string();
    let stdout_thread = thread::spawn(move || {
        let reader = BufReader::new(stdout);
        for line in reader.lines() {
            if let Ok(line) = line {
                // Log via tracing
                tracing::debug!(target: "build_output", package = %package_name_stdout, "{}", line);

                // Update last line
                if let Ok(mut last) = last_line_stdout.lock() {
                    *last = line.clone();
                }

                // Call callback
                if let Some(ref callback) = log_callback_stdout {
                    callback(&line);
                }
            }
        }
    });

    // Spawn thread to read stderr
    let last_line_stderr = Arc::clone(&last_line);
    let log_callback_stderr = log_callback;
    let package_name_stderr = package_name.to_string();
    let stderr_thread = thread::spawn(move || {
        let reader = BufReader::new(stderr);
        for line in reader.lines() {
            if let Ok(line) = line {
                // Log via tracing (use warn level for stderr)
                tracing::warn!(target: "build_output", package = %package_name_stderr, "{}", line);

                // Update last line
                if let Ok(mut last) = last_line_stderr.lock() {
                    *last = line.clone();
                }

                // Call callback
                if let Some(ref callback) = log_callback_stderr {
                    callback(&line);
                }
            }
        }
    });

    // Wait for the process to complete
    let status = child.wait().map_err(|e| {
        crate::Error::build(
            format!("{} failed for {}", operation, package_name),
            &e.to_string(),
        )
    })?;

    // Wait for output threads to finish
    let _ = stdout_thread.join();
    let _ = stderr_thread.join();

    // Check if the command succeeded
    if !status.success() {
        return Err(crate::Error::build(
            format!("{} failed for {}", operation, package_name),
            "Check the build output for errors",
        ));
    }

    Ok(())
}
