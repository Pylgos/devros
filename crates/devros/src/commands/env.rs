//! Environment command implementation

use crate::dsv::{DsvFile, EnvCalculator, ShellType};
use crate::workspace::Workspace;
use camino::{Utf8Path, Utf8PathBuf};
use clap::{Args, Subcommand};
use miette::{IntoDiagnostic, Result};
use walkdir::WalkDir;

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

    /// Use a specific prefix directory instead of workspace install directory
    /// (used for deployed/materialized environments)
    #[arg(long)]
    pub prefix: Option<String>,
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

    let mut calc = EnvCalculator::from_current_env();

    if let Some(ref prefix_path) = args.prefix {
        // Use a specific prefix directory (for deployed environments)
        let prefix = Utf8PathBuf::from(prefix_path);

        if !prefix.exists() {
            return Err(miette::miette!(
                "Prefix directory does not exist: {}",
                prefix
            ));
        }

        // In a merged/deployed environment, packages are merged into a single directory
        // We need to scan the share directory for package.dsv files
        let share_dir = prefix.join("share");
        if share_dir.exists() {
            setup_merged_environment(&mut calc, &prefix, &share_dir)?;
        }
    } else if let Some(ref package_name) = args.package {
        // Set up environment for a specific package
        let workspace = Workspace::discover(workspace_root).into_diagnostic()?;
        let install_dir = workspace.package_install_dir(package_name);
        let dsv_path = install_dir
            .join("share")
            .join(package_name)
            .join("package.dsv");

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
        let workspace = Workspace::discover(workspace_root).into_diagnostic()?;

        tracing::debug!("Build order: {:?}", workspace.build_order);
        tracing::debug!(
            "Processing {} packages in build order",
            workspace.build_order.len()
        );
        for package_name in &workspace.build_order {
            let install_dir = workspace.package_install_dir(package_name);
            let dsv_path = install_dir
                .join("share")
                .join(package_name)
                .join("package.dsv");

            if dsv_path.exists() {
                tracing::debug!("Processing package: {}", package_name);
                tracing::debug!(
                    "  AMENT_PREFIX_PATH before: {:?}",
                    calc.get("AMENT_PREFIX_PATH")
                );
                if let Ok(dsv) = DsvFile::parse(&dsv_path) {
                    calc.apply_dsv(&dsv, &install_dir).into_diagnostic()?;
                }
                tracing::debug!(
                    "  AMENT_PREFIX_PATH after: {:?}",
                    calc.get("AMENT_PREFIX_PATH")
                );
            }
        }
    }

    // Output shell commands
    println!("{}", calc.generate_shell_commands(shell_type));

    Ok(())
}

/// Set up environment for a merged/deployed directory
fn setup_merged_environment(
    calc: &mut EnvCalculator,
    prefix: &Utf8Path,
    share_dir: &Utf8Path,
) -> Result<()> {
    // Collect all package.dsv files in the share directory
    let mut dsv_files: Vec<Utf8PathBuf> = Vec::new();

    for entry in WalkDir::new(share_dir).max_depth(2) {
        let entry = entry.into_diagnostic()?;
        let path = entry.path();

        if path.is_file() && path.file_name() == Some(std::ffi::OsStr::new("package.dsv")) {
            if let Some(utf8_path) = Utf8Path::from_path(path) {
                dsv_files.push(utf8_path.to_path_buf());
            }
        }
    }

    // Process each DSV file
    // Note: In a merged environment, all packages share the same prefix
    for dsv_path in dsv_files {
        if let Ok(dsv) = DsvFile::parse(&dsv_path) {
            calc.apply_dsv(&dsv, prefix).into_diagnostic()?;
        }
    }

    Ok(())
}
