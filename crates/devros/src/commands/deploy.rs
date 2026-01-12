//! Deploy command implementation

use crate::deploy::DeployManager;
use crate::workspace::Workspace;
use camino::Utf8Path;
use clap::Args;
use miette::{IntoDiagnostic, Result};

/// Arguments for the deploy command
#[derive(Debug, Args)]
pub struct DeployArgs {
    /// Deploy target name (as defined in devros.toml)
    pub target: String,

    /// Deploy specific packages only
    #[arg(short, long)]
    pub packages: Option<Vec<String>>,

    /// Dry run - show what would be deployed
    #[arg(long)]
    pub dry_run: bool,
}

/// Run the deploy command
pub fn run(workspace_root: &Utf8Path, args: DeployArgs) -> Result<()> {
    tracing::info!("Discovering workspace at {}", workspace_root);

    let workspace = Workspace::discover(workspace_root).into_diagnostic()?;

    // Get deploy target configuration
    let target_config = workspace.config.deploy.get(&args.target).ok_or_else(|| {
        miette::miette!(
            "Deploy target '{}' not found in devros.toml\n\nAvailable targets: {}",
            args.target,
            if workspace.config.deploy.is_empty() {
                "(none configured)".to_string()
            } else {
                workspace
                    .config
                    .deploy
                    .keys()
                    .cloned()
                    .collect::<Vec<_>>()
                    .join(", ")
            }
        )
    })?;

    // Determine which packages to deploy
    let packages_to_deploy: Vec<String> = if let Some(ref pkgs) = args.packages {
        // Validate that all requested packages exist
        for pkg in pkgs {
            if !workspace.packages.contains_key(pkg) {
                return Err(miette::miette!("Package '{}' not found in workspace", pkg));
            }
        }
        pkgs.clone()
    } else {
        workspace.build_order.clone()
    };

    if packages_to_deploy.is_empty() {
        tracing::warn!("No packages to deploy");
        return Ok(());
    }

    tracing::info!(
        "Deploying {} packages to target '{}'",
        packages_to_deploy.len(),
        args.target
    );

    if args.dry_run {
        println!("Would deploy the following packages:");
        for name in &packages_to_deploy {
            println!("  - {}", name);
        }

        match target_config {
            crate::config::DeployTarget::Local(config) => {
                println!("\nTarget: local");
                println!("Directory: {}", config.target_dir);
            }
            crate::config::DeployTarget::Remote(config) => {
                println!("\nTarget: remote");
                println!("Host: {}", config.target);
                println!("Directory: {}", config.target_dir);
                if let Some(ref post) = config.post_deploy {
                    println!("Post-deploy: {}", post);
                }
            }
        }

        return Ok(());
    }

    // Create deploy manager and execute deployment
    let manager = DeployManager::new(&workspace);
    let changed_packages = manager
        .deploy(&args.target, target_config, Some(&packages_to_deploy))
        .into_diagnostic()?;

    if changed_packages.is_empty() {
        tracing::info!("No packages changed since last deployment");
    } else {
        tracing::info!("Changed packages: {}", changed_packages.join(", "));
    }

    tracing::info!("Deployment complete!");
    Ok(())
}
