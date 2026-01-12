//! Environment command implementation

use camino::Utf8Path;
use clap::{Args, Subcommand};
use devros_core::dsv::{DsvFile, EnvCalculator, ShellType};
use devros_core::workspace::Workspace;
use miette::{IntoDiagnostic, Result};

/// Environment subcommands
#[derive(Debug, Subcommand)]
pub enum EnvCommand {
    /// Output shell commands to set up the environment
    Shell(ShellArgs),
}

/// Arguments for the shell subcommand
#[derive(Debug, Args)]
pub struct ShellArgs {
    /// Shell type (bash, zsh, fish)
    #[arg(long, default_value = "bash")]
    pub shell: String,

    /// Set up environment for a specific package only
    #[arg(long)]
    pub package: Option<String>,
}

/// Run the env command
pub fn run(workspace_root: &Utf8Path, command: EnvCommand) -> Result<()> {
    match command {
        EnvCommand::Shell(args) => shell_command(workspace_root, args),
    }
}

/// Generate shell commands for environment setup
fn shell_command(workspace_root: &Utf8Path, args: ShellArgs) -> Result<()> {
    let shell_type: ShellType = args.shell.parse().into_diagnostic()?;
    let workspace = Workspace::discover(workspace_root).into_diagnostic()?;

    let mut calc = EnvCalculator::from_current_env();

    if let Some(ref package_name) = args.package {
        // Set up environment for a specific package
        let install_dir = workspace.package_install_dir(package_name);
        let dsv_path = install_dir.join("share").join(package_name).join("package.dsv");

        if dsv_path.exists() {
            let dsv = DsvFile::parse(&dsv_path).into_diagnostic()?;
            calc.apply_dsv(&dsv, &install_dir).into_diagnostic()?;
        } else {
            return Err(miette::miette!(
                "Package '{}' not found or not built. DSV file not found at {}",
                package_name,
                dsv_path
            ));
        }
    } else {
        // Set up environment for entire workspace
        for package_name in &workspace.build_order {
            let install_dir = workspace.package_install_dir(package_name);
            let dsv_path = install_dir.join("share").join(package_name).join("package.dsv");

            if dsv_path.exists() {
                if let Ok(dsv) = DsvFile::parse(&dsv_path) {
                    calc.apply_dsv(&dsv, &install_dir).into_diagnostic()?;
                }
            }
        }
    }

    // Output shell commands
    println!("{}", calc.generate_shell_commands(shell_type));

    Ok(())
}
