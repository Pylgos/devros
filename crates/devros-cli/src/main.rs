//! devros CLI - ROS 2 workflow management tool

use clap::{Parser, Subcommand};
use miette::Result;
use tracing_subscriber::{fmt, prelude::*, EnvFilter};

mod commands;

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

    /// Environment variable management
    Env {
        #[command(subcommand)]
        command: commands::env::EnvCommand,
    },
}

fn main() -> Result<()> {
    // Initialize tracing
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));

    tracing_subscriber::registry()
        .with(fmt::layer())
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
        Commands::Env { command } => commands::env::run(&workspace_root, command),
    }
}
