//! DSV (Dynamic Source Value) file parsing and environment variable management
//!
//! This module handles parsing of .dsv files and computing environment variables
//! according to the specification in specs/05_environment_management.md.

mod calculator;
mod generators;
mod operation;
mod parser;
mod shell;

pub use calculator::EnvCalculator;
pub use generators::{
    generate_local_setup_sh, generate_package_dsv, generate_workspace_setup_bash,
};
pub use operation::DsvOperation;
pub use parser::DsvFile;
pub use shell::ShellType;
