//! Progress display for parallel builds
//!
//! This module provides progress bar functionality using indicatif,
//! integrated with tracing for clean log output.

use indicatif::{MultiProgress, ProgressBar, ProgressStyle};
use std::sync::{Arc, Mutex};
use std::{collections::HashMap, time::Duration};

/// Progress manager for parallel package builds
#[derive(Clone)]
pub struct BuildProgress {
    /// Multi-progress container for all progress bars
    multi: MultiProgress,
    /// Active progress bars indexed by package name (shared across threads)
    bars: Arc<Mutex<HashMap<String, ProgressBar>>>,
    /// Completed packages count (shared across threads)
    completed: Arc<Mutex<usize>>,
    /// Total packages count
    total: usize,
    /// Main progress bar showing overall progress
    main_bar: ProgressBar,
}

impl BuildProgress {
    /// Create a new build progress manager with a provided MultiProgress
    ///
    /// This is useful for integrating with tracing-indicatif
    pub fn new_with_multi_progress(total: usize, multi: MultiProgress) -> Self {
        // Create main progress bar
        let main_bar = multi.add(ProgressBar::new(total as u64));
        main_bar.set_style(
            ProgressStyle::default_bar()
                .template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} packages ({eta})")
                .expect("Invalid progress template")
                .progress_chars("#>-"),
        );

        main_bar.enable_steady_tick(Duration::from_millis(100));

        Self {
            multi,
            bars: Arc::new(Mutex::new(HashMap::new())),
            completed: Arc::new(Mutex::new(0)),
            total,
            main_bar,
        }
    }

    /// Start building a package
    pub fn start_package(&self, package_name: &str, build_type: &str) {
        let bar = self.multi.add(ProgressBar::new_spinner());
        bar.set_style(
            ProgressStyle::default_spinner()
                .template("  {spinner:.yellow} {msg}")
                .expect("Invalid spinner template"),
        );
        bar.set_message(format!("{} ({})", package_name, build_type));

        bar.enable_steady_tick(Duration::from_millis(100));

        let mut bars = self.bars.lock().unwrap();
        bars.insert(package_name.to_string(), bar);
    }

    /// Update the message for a package's progress bar
    pub fn update_package(&self, package_name: &str, message: &str) {
        if let Ok(bars) = self.bars.lock() {
            if let Some(bar) = bars.get(package_name) {
                bar.set_message(format!("{}: {}", package_name, message));
            }
        }
    }

    /// Update the log line for a package's progress bar
    ///
    /// Displays the latest log output from the build process
    pub fn update_package_log(&self, package_name: &str, log_line: &str) {
        let bars = self.bars.lock().unwrap();
        if let Some(bar) = bars.get(package_name) {
            // Truncate long lines to fit in the terminal
            let max_len = 80;
            let truncated = if log_line.len() > max_len {
                format!("{}...", &log_line[..max_len])
            } else {
                log_line.to_string()
            };
            bar.set_message(format!("{}: {}", package_name, truncated));
        }
    }

    /// Mark a package as completed successfully
    pub fn finish_package(&self, package_name: &str) {
        if let Ok(mut bars) = self.bars.lock() {
            if let Some(bar) = bars.remove(package_name) {
                bar.finish_and_clear();
            }
        }
        if let Ok(mut completed) = self.completed.lock() {
            *completed += 1;
            self.main_bar.set_position(*completed as u64);
        }
    }

    /// Mark a package as skipped (cache hit)
    pub fn skip_package(&self, _package_name: &str) {
        // Don't create a bar for skipped packages, just increment the counter
        if let Ok(mut completed) = self.completed.lock() {
            *completed += 1;
            self.main_bar.set_position(*completed as u64);
        }
    }

    /// Mark a package as failed
    pub fn fail_package(&self, package_name: &str, error: &str) {
        if let Ok(mut bars) = self.bars.lock() {
            if let Some(bar) = bars.remove(package_name) {
                bar.abandon_with_message(format!("{}: FAILED - {}", package_name, error));
            }
        }
    }

    /// Finish all progress bars
    pub fn finish(&self) {
        let completed = self.completed.lock().map(|c| *c).unwrap_or(0);
        self.main_bar
            .finish_with_message(format!("Built {}/{} packages", completed, self.total));
    }

    /// Get the multi-progress for integration with tracing
    pub fn multi_progress(&self) -> &MultiProgress {
        &self.multi
    }

    /// Suspend progress bars during a closure (for clean output)
    pub fn suspend<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        self.multi.suspend(f)
    }
}
