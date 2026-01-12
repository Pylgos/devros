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
