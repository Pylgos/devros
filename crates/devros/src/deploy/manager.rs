//! Deployment manager
//!
//! This module provides the main deployment orchestration logic.

use camino::Utf8PathBuf;
use std::collections::HashMap;
use std::process::Command;

use crate::cache::CacheManager;
use crate::config::{DeployTarget, LocalDeployConfig, RemoteDeployConfig};
use crate::workspace::Workspace;
use crate::{Error, Result};

use super::materialize::Materializer;
use super::state::DeployState;
use super::utils::{copy_dir_recursive, parse_ssh_target};

/// Manager for deployment operations
pub struct DeployManager<'a> {
    workspace: &'a Workspace,
    cache_manager: CacheManager,
}

impl<'a> DeployManager<'a> {
    /// Create a new deploy manager
    pub fn new(workspace: &'a Workspace) -> Self {
        let cache_manager = CacheManager::new(&workspace.state_dir());
        Self {
            workspace,
            cache_manager,
        }
    }

    /// Get the staging directory for a target
    pub fn staging_dir(&self, target_name: &str) -> Utf8PathBuf {
        self.workspace.state_dir().join("staging").join(target_name)
    }

    /// Get the deploy state file path for a target
    pub fn deploy_state_path(&self, target_name: &str) -> Utf8PathBuf {
        self.workspace
            .state_dir()
            .join("deploy")
            .join(target_name)
            .join("last_deploy.json")
    }

    /// Detect changed packages since last deployment
    pub fn detect_changed_packages(
        &self,
        target_name: &str,
        packages: &[String],
    ) -> Result<Vec<String>> {
        let state_path = self.deploy_state_path(target_name);
        let last_state = DeployState::load(&state_path)?;

        let mut changed = Vec::new();

        for package_name in packages {
            // Get current hash from cache
            let current_hash = self.cache_manager.load(package_name)?.map(|e| e.hash);

            let is_changed = match (&last_state, &current_hash) {
                // No previous state means all packages are new
                (None, _) => true,
                // No current hash (not built) - skip
                (_, None) => {
                    tracing::warn!(
                        package = package_name,
                        "Package has no build hash, skipping"
                    );
                    continue;
                }
                // Compare hashes
                (Some(state), Some(hash)) => state.packages.get(package_name) != Some(hash),
            };

            if is_changed {
                changed.push(package_name.clone());
            }
        }

        Ok(changed)
    }

    /// Materialize packages to a staging directory
    ///
    /// This creates a portable deployment by:
    /// 1. Merging package install directories
    /// 2. Resolving symlinks (dereferencing external links)
    /// 3. Freezing Python packages
    /// 4. Generating environment setup scripts
    pub fn materialize(&self, target_name: &str, packages: &[String]) -> Result<Utf8PathBuf> {
        let staging_dir = self.staging_dir(target_name);
        tracing::info!("Materializing to {}", staging_dir);

        // Create staging directory
        std::fs::create_dir_all(&staging_dir)?;

        let install_base = self
            .workspace
            .root
            .join(&self.workspace.config.workspace.install_dir);

        // Merge packages in dependency order
        for package_name in packages {
            let package_install = install_base.join(package_name);
            if !package_install.exists() {
                tracing::warn!(
                    package = package_name,
                    "Install directory not found, skipping"
                );
                continue;
            }

            tracing::debug!(package = package_name, "Merging package");
            Materializer::merge_directory(&package_install, &staging_dir, &install_base)?;
        }

        // Freeze Python packages
        Materializer::freeze_python_packages(&staging_dir)?;

        // Generate setup scripts
        Materializer::generate_setup_scripts(&staging_dir)?;

        Ok(staging_dir)
    }

    /// Deploy to a local target
    pub fn deploy_local(
        &self,
        target_name: &str,
        config: &LocalDeployConfig,
        packages: &[String],
    ) -> Result<Vec<String>> {
        // Materialize
        let staging_dir = self.materialize(target_name, packages)?;

        // Determine target directory
        let target_dir = if config.target_dir.is_absolute() {
            config.target_dir.clone()
        } else {
            self.workspace.root.join(&config.target_dir)
        };

        tracing::info!("Deploying to {}", target_dir);

        // Detect changed packages
        let changed = self.detect_changed_packages(target_name, packages)?;

        // Copy from staging to target
        std::fs::create_dir_all(&target_dir)?;
        copy_dir_recursive(&staging_dir, &target_dir)?;

        // Save deploy state
        let state = self.build_deploy_state(packages)?;
        state.save(&self.deploy_state_path(target_name))?;

        tracing::info!("Local deployment complete");
        Ok(changed)
    }

    /// Deploy to a remote target
    pub fn deploy_remote(
        &self,
        target_name: &str,
        config: &RemoteDeployConfig,
        packages: &[String],
    ) -> Result<Vec<String>> {
        // Materialize
        let staging_dir = self.materialize(target_name, packages)?;

        // Detect changed packages
        let changed = self.detect_changed_packages(target_name, packages)?;

        tracing::info!("Deploying to {}:{}", config.target, config.target_dir);

        // Parse SSH target - handle optional port in format "user@host:port"
        let (ssh_target, ssh_port) = parse_ssh_target(&config.target);

        // Build rsync command
        let mut cmd = Command::new("rsync");
        cmd.args(["-avz", "--delete", "--checksum"]);

        // Add SSH port option if non-standard port is specified
        if let Some(port) = ssh_port {
            cmd.args(["-e", &format!("ssh -p {}", port)]);
        }

        // Add source and destination
        cmd.arg(format!("{}/", staging_dir));
        cmd.arg(format!("{}:{}/", ssh_target, config.target_dir));

        tracing::debug!("Running: {:?}", cmd);

        let status = cmd.status()?;

        if !status.success() {
            return Err(Error::deploy(
                format!("rsync failed with exit code: {:?}", status.code()),
                "Check SSH connectivity and permissions",
            ));
        }

        // Save deploy state
        let state = self.build_deploy_state(packages)?;
        state.save(&self.deploy_state_path(target_name))?;

        // Execute post_deploy hook if configured
        if let Some(ref post_deploy) = config.post_deploy {
            self.run_post_deploy(post_deploy, config, &changed)?;
        }

        tracing::info!("Remote deployment complete");
        Ok(changed)
    }

    /// Run post-deployment hook
    fn run_post_deploy(
        &self,
        command: &str,
        config: &RemoteDeployConfig,
        changed_packages: &[String],
    ) -> Result<()> {
        // Replace placeholders
        let command = command
            .replace("{target}", &config.target)
            .replace("{changed_packages}", &changed_packages.join(" "));

        tracing::info!("Running post-deploy: {}", command);

        let status = Command::new("sh").args(["-c", &command]).status()?;

        if !status.success() {
            return Err(Error::deploy(
                format!(
                    "post_deploy command failed with exit code: {:?}",
                    status.code()
                ),
                "Check the post_deploy command in your devros.toml",
            ));
        }

        Ok(())
    }

    /// Build deploy state from current cache
    fn build_deploy_state(&self, packages: &[String]) -> Result<DeployState> {
        let mut pkg_hashes = HashMap::new();

        for package_name in packages {
            if let Some(entry) = self.cache_manager.load(package_name)? {
                pkg_hashes.insert(package_name.clone(), entry.hash);
            }
        }

        Ok(DeployState::new(pkg_hashes))
    }

    /// Execute deployment to a target
    pub fn deploy(
        &self,
        target_name: &str,
        target: &DeployTarget,
        packages: Option<&[String]>,
    ) -> Result<Vec<String>> {
        // Determine which packages to deploy
        let packages_to_deploy: Vec<String> = if let Some(pkgs) = packages {
            pkgs.to_vec()
        } else {
            self.workspace.build_order.clone()
        };

        if packages_to_deploy.is_empty() {
            tracing::warn!("No packages to deploy");
            return Ok(Vec::new());
        }

        match target {
            DeployTarget::Local(config) => {
                self.deploy_local(target_name, config, &packages_to_deploy)
            }
            DeployTarget::Remote(config) => {
                self.deploy_remote(target_name, config, &packages_to_deploy)
            }
        }
    }
}
