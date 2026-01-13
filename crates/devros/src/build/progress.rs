//! Progress display for parallel builds
//!
//! This module provides progress bar functionality using indicatif,
//! integrated with tracing for clean log output.

use indicatif::{MultiProgress, ProgressBar, ProgressStyle};
use std::collections::HashMap;
use std::time::Duration;

/// Progress manager for parallel package builds
pub struct BuildProgress {
    /// Multi-progress container for all progress bars
    multi: MultiProgress,
    /// Active progress bars indexed by package name
    bars: HashMap<String, ProgressBar>,
    /// Completed packages count
    completed: usize,
    /// Total packages count
    total: usize,
    /// Main progress bar showing overall progress
    main_bar: ProgressBar,
}

impl BuildProgress {
    /// Create a new build progress manager
    pub fn new(total: usize) -> Self {
        let multi = MultiProgress::new();

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
            bars: HashMap::new(),
            completed: 0,
            total,
            main_bar,
        }
    }

    /// Start building a package
    pub fn start_package(&mut self, package_name: &str, build_type: &str) {
        let bar = self.multi.add(ProgressBar::new_spinner());
        bar.set_style(
            ProgressStyle::default_spinner()
                .template("  {spinner:.yellow} {msg}")
                .expect("Invalid spinner template"),
        );
        bar.set_message(format!("{} ({})", package_name, build_type));
        bar.enable_steady_tick(Duration::from_millis(100));

        self.bars.insert(package_name.to_string(), bar);
    }

    /// Update the message for a package's progress bar
    pub fn update_package(&self, package_name: &str, message: &str) {
        if let Some(bar) = self.bars.get(package_name) {
            bar.set_message(format!("{}: {}", package_name, message));
        }
    }

    /// Mark a package as completed successfully
    pub fn finish_package(&mut self, package_name: &str) {
        if let Some(bar) = self.bars.remove(package_name) {
            bar.finish_and_clear();
        }
        self.completed += 1;
        self.main_bar.set_position(self.completed as u64);
    }

    /// Mark a package as skipped (cache hit)
    pub fn skip_package(&mut self, _package_name: &str) {
        // Don't create a bar for skipped packages, just increment the counter
        self.completed += 1;
        self.main_bar.set_position(self.completed as u64);
    }

    /// Mark a package as failed
    pub fn fail_package(&mut self, package_name: &str, error: &str) {
        if let Some(bar) = self.bars.remove(package_name) {
            bar.abandon_with_message(format!("{}: FAILED - {}", package_name, error));
        }
    }

    /// Finish all progress bars
    pub fn finish(&self) {
        self.main_bar
            .finish_with_message(format!("Built {}/{} packages", self.completed, self.total));
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

impl Drop for BuildProgress {
    fn drop(&mut self) {
        // Clear any remaining progress bars
        for (_, bar) in self.bars.drain() {
            bar.finish_and_clear();
        }
    }
}
