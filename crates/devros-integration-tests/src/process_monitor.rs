use std::process::Command;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// Statistics about observed process counts
#[derive(Debug)]
pub struct ProcessStats {
    /// Maximum number of processes observed
    pub max_processes: usize,
    /// Average number of processes across all samples
    pub avg_processes: f64,
    /// All samples collected
    pub samples: Vec<usize>,
}

/// Monitor for tracking process counts during test execution
pub struct ProcessMonitor {
    stop_flag: Arc<AtomicBool>,
    handle: Option<JoinHandle<Vec<usize>>>,
}

impl ProcessMonitor {
    /// Start monitoring processes matching the given pattern
    ///
    /// # Arguments
    /// * `pattern` - Pattern to match against process names (e.g., "sleep", "cmake|make")
    ///
    /// # Example
    /// ```no_run
    /// use devros_integration_tests::process_monitor::ProcessMonitor;
    ///
    /// let monitor = ProcessMonitor::start("sleep");
    /// // ... run some code that spawns processes ...
    /// let stats = monitor.stop();
    /// println!("Max processes: {}", stats.max_processes);
    /// ```
    pub fn start(pattern: &str) -> Self {
        let stop_flag = Arc::new(AtomicBool::new(false));
        let stop_flag_clone = Arc::clone(&stop_flag);
        let pattern = pattern.to_string();

        let handle = thread::spawn(move || {
            let mut samples = Vec::new();

            while !stop_flag_clone.load(Ordering::Relaxed) {
                if let Ok(count) = count_processes_matching(&pattern) {
                    samples.push(count);
                }

                thread::sleep(Duration::from_millis(100));
            }

            samples
        });

        Self {
            stop_flag,
            handle: Some(handle),
        }
    }

    /// Stop monitoring and return statistics
    pub fn stop(mut self) -> ProcessStats {
        self.stop_flag.store(true, Ordering::Relaxed);

        let samples = self
            .handle
            .take()
            .expect("Monitor handle should exist")
            .join()
            .expect("Monitor thread should finish cleanly");

        let max_processes = samples.iter().copied().max().unwrap_or(0);
        let avg_processes = if !samples.is_empty() {
            samples.iter().sum::<usize>() as f64 / samples.len() as f64
        } else {
            0.0
        };

        ProcessStats {
            max_processes,
            avg_processes,
            samples,
        }
    }
}

/// Count the number of processes matching the given pattern
///
/// Uses `pgrep` to count processes. Returns 0 if pgrep fails or finds no matches.
///
/// # Arguments
/// * `pattern` - Pattern to match against process names
///
/// # Returns
/// Number of matching processes, or 0 if none found or error occurred
pub fn count_processes_matching(pattern: &str) -> Result<usize, String> {
    let output = Command::new("pgrep")
        .args(["-c", "-f", pattern])
        .output()
        .map_err(|e| format!("Failed to run pgrep: {}", e))?;

    // pgrep -c returns the count in stdout
    // If no processes match, it returns exit code 1 with "0" in stderr
    if output.status.success() {
        let stdout = String::from_utf8_lossy(&output.stdout);
        stdout
            .trim()
            .parse::<usize>()
            .map_err(|e| format!("Failed to parse pgrep output: {}", e))
    } else {
        // No processes found
        Ok(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_count_processes_matching() {
        // This should always find at least one process (the test itself runs in a process)
        let result = count_processes_matching(".");
        assert!(result.is_ok());
        let count = result.unwrap();
        assert!(count > 0, "Should find at least one process");
    }

    #[test]
    fn test_count_nonexistent_processes() {
        // This should not find any processes with this very specific pattern
        let result = count_processes_matching("nonexistent_process_xyz_12345");
        assert!(result.is_ok());
        let count = result.unwrap();
        assert_eq!(count, 0, "Should not find any processes");
    }

    #[test]
    fn test_process_monitor() {
        let monitor = ProcessMonitor::start("sleep");

        // Spawn some sleep processes
        let mut children = vec![];
        for _ in 0..3 {
            let child = Command::new("sleep")
                .arg("2")
                .spawn()
                .expect("Failed to spawn sleep");
            children.push(child);
        }

        // Wait a bit for monitoring
        thread::sleep(Duration::from_millis(500));

        let stats = monitor.stop();

        // Clean up
        for mut child in children {
            let _ = child.kill();
            let _ = child.wait();
        }

        // We should have observed at least 3 sleep processes
        assert!(
            stats.max_processes >= 3,
            "Should have observed at least 3 sleep processes, got {}",
            stats.max_processes
        );
        assert!(stats.samples.len() > 0, "Should have collected samples");
        assert!(
            stats.avg_processes > 0.0,
            "Average should be greater than 0"
        );
    }
}
