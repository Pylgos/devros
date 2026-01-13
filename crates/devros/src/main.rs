//! devros CLI - ROS 2 workflow management tool

use clap::{Parser, Subcommand};
use miette::Result;
use tracing_indicatif::IndicatifLayer;
use tracing_subscriber::{EnvFilter, fmt, prelude::*};

use devros::commands;

/// devros - ROS 2 workflow management tool
#[derive(Debug, Parser)]
#[command(name = "devros")]
#[command(version, about, long_about = None)]
pub struct Cli {
    /// Enable verbose output
    #[arg(short, long, global = true)]
    verbose: bool,

    /// Workspace root directory
    #[arg(short = 'w', long, global = true)]
    workspace: Option<String>,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Debug, Subcommand)]
enum Commands {
    /// Build packages in the workspace
    Build(commands::build::BuildArgs),

    /// Deploy packages to a target
    Deploy(commands::deploy::DeployArgs),

    /// Environment variable management
    Env {
        #[command(subcommand)]
        command: commands::env::EnvCommand,
    },
}

fn main() -> Result<()> {
    // Initialize tracing with indicatif layer for progress bar support
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));

    // Create indicatif layer for progress bars
    let indicatif_layer = IndicatifLayer::new();

    tracing_subscriber::registry()
        .with(fmt::layer().with_writer(indicatif_layer.get_stderr_writer()))
        .with(indicatif_layer)
        .with(filter)
        .init();

    let cli = Cli::parse();

    // Determine workspace root
    let workspace_root = if let Some(ref path) = cli.workspace {
        camino::Utf8PathBuf::from(path)
    } else {
        std::env::current_dir()
            .ok()
            .and_then(|p| camino::Utf8PathBuf::try_from(p).ok())
            .unwrap_or_else(|| camino::Utf8PathBuf::from("."))
    };

    match cli.command {
        Commands::Build(args) => commands::build::run(&workspace_root, args),
        Commands::Deploy(args) => commands::deploy::run(&workspace_root, args),
        Commands::Env { command } => commands::env::run(&workspace_root, command),
    }
}
