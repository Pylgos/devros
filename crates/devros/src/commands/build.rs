//! Build command implementation
//!
//! This module provides the CLI interface for building packages.

use crate::build::{BuildArgs as CoreBuildArgs, Builder};
use crate::workspace::Workspace;
use camino::Utf8Path;
use clap::Args;
use jobserver::Client;
use miette::{IntoDiagnostic, Result};

/// Arguments for the build command
#[derive(Debug, Args)]
pub struct BuildArgs {
    /// Build specific packages only
    #[arg(short, long)]
    pub packages: Option<Vec<String>>,

    /// Number of parallel jobs
    #[arg(short, long)]
    pub jobs: Option<usize>,

    /// Use symlink install (default: true)
    #[arg(long, default_value = "true")]
    pub symlink_install: bool,

    /// Dry run - show what would be built
    #[arg(long)]
    pub dry_run: bool,

    /// Force rebuild even if cache is valid
    #[arg(long)]
    pub force_rebuild: bool,
}

impl From<BuildArgs> for CoreBuildArgs {
    fn from(args: BuildArgs) -> Self {
        CoreBuildArgs {
            packages: args.packages,
            jobs: args.jobs,
            symlink_install: args.symlink_install,
            dry_run: args.dry_run,
            force_rebuild: args.force_rebuild,
        }
    }
}

/// Run the build command
pub fn run(workspace_root: &Utf8Path, args: BuildArgs) -> Result<()> {
    let workspace = Workspace::discover(workspace_root).into_diagnostic()?;

    // Get effective jobs for jobserver initialization
    let jobs = args
        .jobs
        .unwrap_or_else(|| workspace.config.effective_jobs());

    // Initialize jobserver for parallel build control
    // Try to inherit from parent process first, otherwise create a new one
    //
    // SAFETY: `Client::from_env()` is unsafe because it:
    // 1. Reads MAKEFLAGS/CARGO_MAKEFLAGS environment variables
    // 2. May open file descriptors based on those values
    // This is safe here because:
    // - We are the top-level process controlling the build
    // - If the env vars are set but invalid, `from_env()` returns None
    // - We handle the None case by creating a new jobserver
    // - The jobserver is only used for coordinating parallel cmake builds
    let jobserver = unsafe { Client::from_env() }.or_else(|| {
        tracing::debug!("Creating new jobserver with {} jobs", jobs);
        Client::new(jobs).ok()
    });

    // Convert CLI args to core args
    let core_args: CoreBuildArgs = args.into();

    // Create builder with optional jobserver
    let mut builder = Builder::new(&workspace);
    if let Some(js) = jobserver {
        builder = builder.with_jobserver(js);
    }

    // Execute build
    builder.build(&core_args).into_diagnostic()?;

    Ok(())
}
