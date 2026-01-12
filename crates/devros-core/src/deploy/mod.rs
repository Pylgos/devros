//! Deployment functionality including materialization and transfer
//!
//! This module provides:
//! - Materialization: converting symlink-based install to portable artifacts
//! - Local deployment: copying materialized artifacts to local directories
//! - Remote deployment: transferring artifacts via rsync over SSH
//!
//! **Note**: This module uses Unix-specific functionality (symlinks, rsync)
//! and is designed for use on Linux systems as is standard for ROS 2 development.

mod manager;
mod materialize;
mod state;
mod utils;

pub use manager::DeployManager;
pub use state::DeployState;
pub use utils::copy_dir_recursive;
