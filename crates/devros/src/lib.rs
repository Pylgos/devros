//! devros - ROS 2 workflow management tool
//!
//! This crate provides both a library and CLI for devros, including:
//! - Configuration file parsing and merging
//! - Workspace analysis (package discovery, package.xml parsing)
//! - Dependency graph construction and topological sorting
//! - Environment variable management (.dsv file handling)
//! - Build system integration (ament_cmake, ament_python)
//! - Build cache management
//! - Deployment and materialization

use indicatif::MultiProgress;
use std::sync::OnceLock;

/// Global MultiProgress for coordinating progress bars and log output
pub static MULTI_PROGRESS: OnceLock<MultiProgress> = OnceLock::new();

/// Get or initialize the global MultiProgress
pub fn get_multi_progress() -> &'static MultiProgress {
    MULTI_PROGRESS.get_or_init(MultiProgress::new)
}

pub mod build;
pub mod cache;
pub mod commands;
pub mod config;
pub mod deploy;
pub mod dsv;
pub mod error;
pub mod package;
pub mod workspace;

pub use error::{Error, Result};
