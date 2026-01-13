//! Build system for ROS 2 packages
//!
//! This module provides the build functionality for devros, including:
//! - Build orchestration with dependency ordering
//! - Build type specific implementations (ament_cmake, ament_python)
//! - Environment computation for builds
//! - Cache integration for incremental builds
//! - Parallel execution with progress display

mod ament_cmake;
mod ament_python;
mod builder;
mod command_logger;
mod environment;
mod parallel;
mod progress;
mod progress_writer;

pub use ament_cmake::AmentCmakeBuilder;
pub use ament_python::AmentPythonBuilder;
pub use builder::{BuildArgs, BuildResult, Builder};
pub use command_logger::run_command_with_logging;
pub use environment::compute_build_environment;
pub use parallel::ParallelExecutor;
pub use progress::BuildProgress;
pub use progress_writer::make_writer;
