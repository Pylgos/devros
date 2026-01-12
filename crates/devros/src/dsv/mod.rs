//! DSV (Dynamic Source Value) file parsing and environment variable management
//!
//! This module handles parsing of .dsv files and computing environment variables
//! according to the specification in specs/05_environment_management.md.
//! The generated files are colcon-compatible to ensure interoperability with
//! existing ROS 2 tools.

mod calculator;
mod generators;
mod operation;
mod parser;
mod shell;

pub use calculator::EnvCalculator;
pub use generators::{
    filter_workspace_dependencies, generate_ament_cmake_package_dsv,
    generate_ament_python_package_dsv, generate_colcon_install_layout, generate_colcon_marker_file,
    generate_local_setup_util_py, generate_workspace_local_setup_sh, generate_workspace_setup_bash,
    generate_workspace_setup_sh, generate_workspace_setup_zsh, write_colcon_marker_file,
};
pub use operation::DsvOperation;
pub use parser::DsvFile;
pub use shell::ShellType;
