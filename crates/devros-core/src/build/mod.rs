//! Build system for ROS 2 packages
//!
//! This module provides the build functionality for devros, including:
//! - Build orchestration with dependency ordering
//! - Build type specific implementations (ament_cmake, ament_python)
//! - Environment computation for builds
//! - Cache integration for incremental builds

mod ament_cmake;
mod ament_python;
mod builder;
mod environment;

pub use ament_cmake::AmentCmakeBuilder;
pub use ament_python::AmentPythonBuilder;
pub use builder::{BuildArgs, BuildResult, Builder};
pub use environment::compute_build_environment;
