//! Command execution with log capture and progress display
//!
//! This module provides utilities for running build commands while capturing
//! their output and displaying it through indicatif progress bars and tracing.

use std::process::Stdio;
use std::sync::Arc;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command;

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
pub async fn run_command_with_logging(
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
            e.to_string(),
        )
    })?;

    // Take ownership of stdout and stderr
    let stdout = child.stdout.take().expect("Failed to capture stdout");
    let stderr = child.stderr.take().expect("Failed to capture stderr");

    // Spawn task to read stdout
    let log_callback_stdout = log_callback.clone();
    let package_name_stdout = package_name.to_string();

    let stdout_handle = tokio::spawn(async move {
        let reader = BufReader::new(stdout);
        let mut lines = reader.lines();
        while let Ok(Some(line)) = lines.next_line().await {
            // Log via tracing
            tracing::debug!(target: "build_output", package = %package_name_stdout, "{}", line);

            // Call callback
            if let Some(ref callback) = log_callback_stdout {
                callback(&line);
            }
        }
    });

    // Spawn task to read stderr
    let log_callback_stderr = log_callback;
    let package_name_stderr = package_name.to_string();

    let stderr_handle = tokio::spawn(async move {
        let reader = BufReader::new(stderr);
        let mut lines = reader.lines();
        while let Ok(Some(line)) = lines.next_line().await {
            // Log via tracing (use warn level for stderr)
            tracing::warn!(target: "build_output", package = %package_name_stderr, "{}", line);

            // Call callback
            if let Some(ref callback) = log_callback_stderr {
                callback(&line);
            }
        }
    });

    // Wait for the process to complete
    let status = child.wait().await.map_err(|e| {
        crate::Error::build(
            format!("{} failed for {}", operation, package_name),
            e.to_string(),
        )
    })?;

    // Wait for output tasks to finish
    let _ = stdout_handle.await;
    let _ = stderr_handle.await;

    // Check if the command succeeded
    if !status.success() {
        return Err(crate::Error::build(
            format!("{} failed for {}", operation, package_name),
            "Check the build output for errors",
        ));
    }

    Ok(())
}
