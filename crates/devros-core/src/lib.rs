//! devros-core - Core library for devros
//!
//! This crate provides the core functionality for devros, including:
//! - Configuration file parsing and merging
//! - Workspace analysis (package discovery, package.xml parsing)
//! - Dependency graph construction and topological sorting
//! - Environment variable management (.dsv file handling)
//! - Build system integration (ament_cmake, ament_python)

pub mod config;
pub mod dsv;
pub mod error;
pub mod package;
pub mod workspace;

pub use error::{Error, Result};
