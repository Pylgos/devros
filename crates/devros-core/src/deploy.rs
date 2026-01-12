//! Deployment functionality including materialization and transfer
//!
//! This module provides:
//! - Materialization: converting symlink-based install to portable artifacts
//! - Local deployment: copying materialized artifacts to local directories
//! - Remote deployment: transferring artifacts via rsync over SSH
//!
//! **Note**: This module uses Unix-specific functionality (symlinks, rsync)
//! and is designed for use on Linux systems as is standard for ROS 2 development.

use camino::{Utf8Path, Utf8PathBuf};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::process::Command;
use walkdir::WalkDir;

use crate::cache::CacheManager;
use crate::config::{DeployTarget, LocalDeployConfig, RemoteDeployConfig};
use crate::workspace::Workspace;
use crate::{Error, Result};

/// Python path patterns that indicate a build-specific absolute path
/// that should be replaced with a portable shebang.
/// These are matched at the start of the path after "#!" prefix.
const PYTHON_BUILD_PATH_PATTERNS: &[&str] = &[
    "#!/usr/bin/python",
    "#!/bin/python",
    "#!/usr/local/bin/python",
];

/// Python path patterns (anywhere in shebang) that indicate
/// a virtual environment that should be replaced
const PYTHON_VENV_PATTERNS: &[&str] = &["/.venv/", "/venv/", "/.virtualenv/", "/virtualenv/"];

/// Deploy state for tracking changes between deployments
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeployState {
    /// Timestamp of the last deployment
    pub timestamp: DateTime<Utc>,
    /// Package hashes at the time of deployment
    pub packages: HashMap<String, String>,
}

impl DeployState {
    /// Create a new deploy state
    pub fn new(packages: HashMap<String, String>) -> Self {
        Self {
            timestamp: Utc::now(),
            packages,
        }
    }

    /// Load deploy state from a file
    pub fn load(path: &Utf8Path) -> Result<Option<Self>> {
        if !path.exists() {
            return Ok(None);
        }

        let content = std::fs::read_to_string(path)?;
        let state: Self = serde_json::from_str(&content).map_err(|e| {
            Error::deploy(
                format!("Failed to parse deploy state: {}", e),
                "The deploy state file may be corrupted. Try deleting it.",
            )
        })?;

        Ok(Some(state))
    }

    /// Save deploy state to a file
    pub fn save(&self, path: &Utf8Path) -> Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }

        let content = serde_json::to_string_pretty(self).map_err(|e| {
            Error::deploy(
                format!("Failed to serialize deploy state: {}", e),
                "This is likely a bug in devros",
            )
        })?;

        std::fs::write(path, content)?;
        Ok(())
    }
}

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
            self.merge_directory(&package_install, &staging_dir, &install_base)?;
        }

        // Freeze Python packages
        self.freeze_python_packages(&staging_dir)?;

        // Generate setup scripts
        self.generate_setup_scripts(&staging_dir)?;

        Ok(staging_dir)
    }

    /// Merge a single package's install directory into the staging directory
    fn merge_directory(
        &self,
        source: &Utf8Path,
        target: &Utf8Path,
        install_base: &Utf8Path,
    ) -> Result<()> {
        let walker = WalkDir::new(source).follow_links(false);

        for entry in walker {
            let entry = entry.map_err(|e| {
                Error::deploy(
                    format!("Failed to read directory entry: {}", e),
                    "Check directory permissions",
                )
            })?;

            let src_path = Utf8Path::from_path(entry.path()).ok_or_else(|| {
                Error::deploy(
                    format!("Path is not valid UTF-8: {:?}", entry.path()),
                    "Ensure all paths are valid UTF-8",
                )
            })?;

            // Calculate relative path from source
            let rel_path = src_path.strip_prefix(source).map_err(|_| {
                Error::deploy(
                    format!("Failed to strip prefix from {}", src_path),
                    "This is likely a bug in devros",
                )
            })?;

            let dst_path = target.join(rel_path);

            if entry.file_type().is_dir() {
                std::fs::create_dir_all(&dst_path)?;
            } else if entry.file_type().is_symlink() {
                self.handle_symlink(src_path, &dst_path, target, install_base)?;
            } else {
                // Regular file - copy
                if let Some(parent) = dst_path.parent() {
                    std::fs::create_dir_all(parent)?;
                }
                if dst_path.exists() {
                    tracing::debug!(path = %dst_path, "Overwriting existing file");
                }
                std::fs::copy(src_path, &dst_path)?;
            }
        }

        Ok(())
    }

    /// Handle a symlink during materialization
    fn handle_symlink(
        &self,
        src_path: &Utf8Path,
        dst_path: &Utf8Path,
        staging_dir: &Utf8Path,
        install_base: &Utf8Path,
    ) -> Result<()> {
        // Read the symlink target
        let link_target = std::fs::read_link(src_path)?;
        let link_target_utf8 = Utf8PathBuf::try_from(link_target.clone()).map_err(|e| {
            Error::deploy(
                format!("Symlink target is not valid UTF-8: {:?}", e),
                "Ensure all paths are valid UTF-8",
            )
        })?;

        // Resolve to absolute path
        let absolute_target = if link_target_utf8.is_absolute() {
            link_target_utf8.clone()
        } else {
            // Get the parent directory of the symlink
            let parent = src_path.parent().ok_or_else(|| {
                Error::deploy(
                    format!(
                        "Cannot resolve relative symlink without parent directory: {}",
                        src_path
                    ),
                    "This is an unexpected path structure",
                )
            })?;

            // Join with the relative target and try to canonicalize
            match parent.join(&link_target_utf8).canonicalize_utf8() {
                Ok(abs) => abs,
                Err(e) => {
                    // Canonicalization failed - log warning but continue with joined path
                    tracing::warn!(
                        src = %src_path,
                        target = %link_target_utf8,
                        error = %e,
                        "Failed to canonicalize symlink target, using joined path"
                    );
                    parent.join(&link_target_utf8)
                }
            }
        };

        // Check if target is internal (within install_base) or external
        let is_internal = absolute_target.starts_with(install_base);

        if is_internal {
            // Internal link - create relative symlink
            let rel_from_staging = absolute_target.strip_prefix(install_base).map_err(|_| {
                Error::deploy(
                    format!(
                        "Failed to strip install base prefix from {}",
                        absolute_target
                    ),
                    "This is likely a bug in devros",
                )
            })?;

            let final_target = staging_dir.join(rel_from_staging);

            // Calculate relative path from dst_path to final_target
            if let (Some(dst_parent), Some(_)) = (dst_path.parent(), final_target.parent()) {
                let relative = make_relative_path(dst_parent, &final_target);

                // Ensure parent exists
                if let Some(parent) = dst_path.parent() {
                    std::fs::create_dir_all(parent)?;
                }

                // Remove existing symlink/file if exists
                if dst_path.exists() || dst_path.is_symlink() {
                    std::fs::remove_file(dst_path)?;
                }

                std::os::unix::fs::symlink(&relative, dst_path)?;
            }
        } else {
            // External link - dereference (copy the actual content)
            if let Some(parent) = dst_path.parent() {
                std::fs::create_dir_all(parent)?;
            }

            // Remove existing symlink/file if exists
            if dst_path.exists() || dst_path.is_symlink() {
                if dst_path.is_dir() {
                    std::fs::remove_dir_all(dst_path)?;
                } else {
                    std::fs::remove_file(dst_path)?;
                }
            }

            // Copy the target (could be file or directory)
            if absolute_target.is_dir() {
                copy_dir_recursive(&absolute_target, dst_path)?;
            } else if absolute_target.is_file() {
                std::fs::copy(&absolute_target, dst_path)?;
            } else {
                tracing::warn!(
                    src = %src_path,
                    target = %absolute_target,
                    "Symlink target does not exist, skipping"
                );
            }
        }

        Ok(())
    }

    /// Freeze Python packages (resolve .egg-link files)
    fn freeze_python_packages(&self, staging_dir: &Utf8Path) -> Result<()> {
        // Find all site-packages directories
        let lib_dir = staging_dir.join("lib");
        if !lib_dir.exists() {
            return Ok(());
        }

        for entry in std::fs::read_dir(&lib_dir)? {
            let entry = entry?;
            let path = entry.path();

            // Look for pythonX.Y directories
            if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                if name.starts_with("python") {
                    let site_packages =
                        Utf8PathBuf::try_from(path.join("site-packages")).map_err(|e| {
                            Error::deploy(
                                format!("Path is not valid UTF-8: {:?}", e),
                                "Ensure all file paths contain only valid UTF-8 characters",
                            )
                        })?;

                    if site_packages.exists() {
                        self.freeze_site_packages(&site_packages)?;
                    }
                }
            }
        }

        // Fix shebangs in bin directory
        let bin_dir = staging_dir.join("bin");
        if bin_dir.exists() {
            self.fix_shebangs(&bin_dir)?;
        }

        Ok(())
    }

    /// Freeze .egg-link files in a site-packages directory
    fn freeze_site_packages(&self, site_packages: &Utf8Path) -> Result<()> {
        let mut easy_install_entries: HashSet<String> = HashSet::new();
        let mut egg_links_to_remove: Vec<Utf8PathBuf> = Vec::new();

        // Find and process .egg-link files
        for entry in std::fs::read_dir(site_packages)? {
            let entry = entry?;
            let path = Utf8PathBuf::try_from(entry.path()).map_err(|e| {
                Error::deploy(
                    format!("Path is not valid UTF-8: {:?}", e),
                    "Ensure all file paths contain only valid UTF-8 characters",
                )
            })?;

            if path.extension() == Some("egg-link") {
                self.process_egg_link(&path, site_packages, &mut easy_install_entries)?;
                egg_links_to_remove.push(path);
            }
        }

        // Remove .egg-link files
        for path in egg_links_to_remove {
            std::fs::remove_file(&path)?;
            tracing::debug!(path = %path, "Removed .egg-link file");
        }

        // Clean up easy-install.pth
        let easy_install_pth = site_packages.join("easy-install.pth");
        if easy_install_pth.exists() {
            self.clean_easy_install_pth(&easy_install_pth, &easy_install_entries)?;
        }

        // Compile Python files
        self.compile_python_files(site_packages)?;

        Ok(())
    }

    /// Process a single .egg-link file
    fn process_egg_link(
        &self,
        egg_link_path: &Utf8Path,
        site_packages: &Utf8Path,
        easy_install_entries: &mut HashSet<String>,
    ) -> Result<()> {
        let content = std::fs::read_to_string(egg_link_path)?;
        let lines: Vec<&str> = content.lines().collect();

        if lines.is_empty() {
            return Ok(());
        }

        // First line is the source directory
        let source_dir = Utf8PathBuf::from(lines[0].trim());
        easy_install_entries.insert(source_dir.to_string());

        if !source_dir.exists() {
            tracing::warn!(path = %source_dir, "Source directory in .egg-link does not exist");
            return Ok(());
        }

        // Get package name from egg-link filename
        let package_name = egg_link_path.file_stem().ok_or_else(|| {
            Error::deploy(
                format!("Invalid .egg-link filename: {}", egg_link_path),
                "Ensure .egg-link files have valid filenames with package names",
            )
        })?;

        // Copy source directory contents to site-packages
        // For Python packages, we need to copy the actual package directories
        for entry in std::fs::read_dir(&source_dir)? {
            let entry = entry?;
            let entry_path = Utf8PathBuf::try_from(entry.path()).map_err(|e| {
                Error::deploy(
                    format!("Path is not valid UTF-8: {:?}", e),
                    "Ensure all file paths contain only valid UTF-8 characters",
                )
            })?;

            let name = entry_path.file_name().ok_or_else(|| {
                Error::deploy(
                    format!("Path has no filename component: {}", entry_path),
                    "This is an unexpected filesystem state",
                )
            })?;

            // Skip special directories and files
            if name.starts_with('.') || name == "build" || name.ends_with(".egg-info") {
                continue;
            }

            // Copy Python packages (directories with __init__.py)
            if entry_path.is_dir() && entry_path.join("__init__.py").exists() {
                let target = site_packages.join(name);
                if target.exists() {
                    std::fs::remove_dir_all(&target)?;
                }
                copy_dir_recursive(&entry_path, &target)?;
                tracing::debug!(src = %entry_path, dst = %target, "Copied Python package");
            }
        }

        // Find and copy .egg-info directory
        for entry in std::fs::read_dir(&source_dir)? {
            let entry = entry?;
            let entry_path = Utf8PathBuf::try_from(entry.path()).map_err(|e| {
                Error::deploy(
                    format!("Path is not valid UTF-8: {:?}", e),
                    "Ensure all file paths contain only valid UTF-8 characters",
                )
            })?;

            if let Some(name) = entry_path.file_name() {
                if name.ends_with(".egg-info") && entry_path.is_dir() {
                    let target = site_packages.join(name);
                    if target.exists() {
                        std::fs::remove_dir_all(&target)?;
                    }
                    copy_dir_recursive(&entry_path, &target)?;
                    tracing::debug!(src = %entry_path, dst = %target, "Copied .egg-info");
                }
            }
        }

        tracing::info!(package = package_name, "Frozen Python package");
        Ok(())
    }

    /// Clean easy-install.pth file by removing egg-link entries
    fn clean_easy_install_pth(
        &self,
        pth_path: &Utf8Path,
        entries_to_remove: &HashSet<String>,
    ) -> Result<()> {
        let content = std::fs::read_to_string(pth_path)?;
        let filtered_lines: Vec<&str> = content
            .lines()
            .filter(|line| {
                let trimmed = line.trim();
                !entries_to_remove.contains(trimmed)
            })
            .collect();

        std::fs::write(pth_path, filtered_lines.join("\n"))?;
        Ok(())
    }

    /// Compile Python files to .pyc
    fn compile_python_files(&self, directory: &Utf8Path) -> Result<()> {
        let status = Command::new("python3")
            .args(["-m", "compileall", "-q", "-f", directory.as_str()])
            .status();

        match status {
            Ok(s) if s.success() => {
                tracing::debug!(dir = %directory, "Compiled Python files");
            }
            Ok(s) => {
                tracing::warn!(
                    dir = %directory,
                    exit_code = ?s.code(),
                    "Python compilation had warnings"
                );
            }
            Err(e) => {
                tracing::warn!(
                    dir = %directory,
                    error = %e,
                    "Failed to compile Python files"
                );
            }
        }

        Ok(())
    }

    /// Fix shebangs in Python scripts
    fn fix_shebangs(&self, bin_dir: &Utf8Path) -> Result<()> {
        for entry in std::fs::read_dir(bin_dir)? {
            let entry = entry?;
            let path = Utf8PathBuf::try_from(entry.path()).map_err(|e| {
                Error::deploy(
                    format!("Path is not valid UTF-8: {:?}", e),
                    "Ensure all file paths contain only valid UTF-8 characters",
                )
            })?;

            if path.is_file() {
                self.fix_shebang(&path)?;
            }
        }

        Ok(())
    }

    /// Fix shebang in a single file
    fn fix_shebang(&self, path: &Utf8Path) -> Result<()> {
        let content = match std::fs::read_to_string(path) {
            Ok(c) => c,
            Err(_) => return Ok(()), // Skip binary files
        };

        // Check if it's a Python script with absolute shebang
        if let Some(first_line) = content.lines().next() {
            if first_line.starts_with("#!") && first_line.contains("python") {
                // Skip if it's already using env (portable)
                if first_line.contains("/usr/bin/env") {
                    return Ok(());
                }

                // Check if it starts with a known system Python path
                let is_system_python = PYTHON_BUILD_PATH_PATTERNS
                    .iter()
                    .any(|pattern| first_line.starts_with(pattern));

                // Check if it's a virtual environment path
                let is_venv_python = PYTHON_VENV_PATTERNS
                    .iter()
                    .any(|pattern| first_line.contains(pattern));

                if is_system_python || is_venv_python {
                    // Replace with portable shebang
                    let new_content = format!(
                        "#!/usr/bin/env python3\n{}",
                        content.lines().skip(1).collect::<Vec<_>>().join("\n")
                    );

                    std::fs::write(path, new_content)?;
                    tracing::debug!(path = %path, "Fixed shebang");
                }
            }
        }

        Ok(())
    }

    /// Generate setup.bash and setup.sh scripts
    fn generate_setup_scripts(&self, staging_dir: &Utf8Path) -> Result<()> {
        // Generate setup.bash
        // Note: This assumes 'devros' is available in PATH on the target system
        let setup_bash_content = r#"#!/bin/bash
# Generated by devros
# Source this file to set up the environment
# Requires: devros command to be installed and available in PATH

_DEVROS_SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
eval "$(devros env shell --prefix "$_DEVROS_SETUP_DIR" --shell bash)"
"#;

        let setup_bash = staging_dir.join("setup.bash");
        std::fs::write(&setup_bash, setup_bash_content)?;

        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mut perms = std::fs::metadata(&setup_bash)?.permissions();
            perms.set_mode(0o755);
            std::fs::set_permissions(&setup_bash, perms)?;
        }

        // Generate setup.sh (POSIX compatible, but still uses bash for devros output)
        // Note: While the script itself is POSIX sh, devros env shell generates bash-compatible output
        let setup_sh_content = r#"#!/bin/sh
# Generated by devros
# Source this file to set up the environment
# Requires: devros command to be installed and available in PATH

_DEVROS_SETUP_DIR="$(cd "$(dirname "$0")" && pwd)"
eval "$(devros env shell --prefix "$_DEVROS_SETUP_DIR" --shell bash)"
"#;

        let setup_sh = staging_dir.join("setup.sh");
        std::fs::write(&setup_sh, setup_sh_content)?;

        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mut perms = std::fs::metadata(&setup_sh)?.permissions();
            perms.set_mode(0o755);
            std::fs::set_permissions(&setup_sh, perms)?;
        }

        tracing::info!("Generated setup scripts");
        Ok(())
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

/// Parse SSH target string to extract host and optional port
///
/// Supports formats:
/// - "user@host" -> ("user@host", None)
/// - "user@host:port" -> ("user@host", Some(port))
fn parse_ssh_target(target: &str) -> (&str, Option<u16>) {
    // Check if there's a colon that could indicate a port
    // Format: user@host:port
    if let Some((host_part, port_str)) = target.rsplit_once(':') {
        // Only treat it as a port if it parses as a valid u16
        if let Ok(port) = port_str.parse::<u16>() {
            return (host_part, Some(port));
        }
    }
    (target, None)
}

/// Calculate a relative path from `from` to `to`
fn make_relative_path(from: &Utf8Path, to: &Utf8Path) -> Utf8PathBuf {
    // Simple implementation - find common prefix and build relative path
    let from_parts: Vec<_> = from.components().collect();
    let to_parts: Vec<_> = to.components().collect();

    // Find common prefix length
    let mut common_len = 0;
    for (a, b) in from_parts.iter().zip(to_parts.iter()) {
        if a == b {
            common_len += 1;
        } else {
            break;
        }
    }

    // Build relative path
    let mut result = Utf8PathBuf::new();

    // Go up from `from` to common ancestor
    for _ in common_len..from_parts.len() {
        result.push("..");
    }

    // Go down from common ancestor to `to`
    for part in &to_parts[common_len..] {
        result.push(part.as_str());
    }

    result
}

/// Recursively copy a directory
fn copy_dir_recursive(src: &Utf8Path, dst: &Utf8Path) -> Result<()> {
    std::fs::create_dir_all(dst)?;

    for entry in WalkDir::new(src).follow_links(false) {
        let entry = entry.map_err(|e| {
            Error::deploy(
                format!("Failed to read directory entry: {}", e),
                "Check directory permissions",
            )
        })?;

        let src_path = Utf8Path::from_path(entry.path()).ok_or_else(|| {
            Error::deploy(
                format!("Path is not valid UTF-8: {:?}", entry.path()),
                "Ensure all file paths contain only valid UTF-8 characters",
            )
        })?;

        let rel_path = src_path.strip_prefix(src).map_err(|_| {
            Error::deploy(
                format!("Failed to strip source prefix from {}", src_path),
                "This is an unexpected internal error",
            )
        })?;

        let dst_path = dst.join(rel_path);

        if entry.file_type().is_dir() {
            std::fs::create_dir_all(&dst_path)?;
        } else if entry.file_type().is_symlink() {
            // Preserve symlinks
            let target = std::fs::read_link(entry.path())?;
            if dst_path.exists() || dst_path.is_symlink() {
                std::fs::remove_file(&dst_path)?;
            }
            std::os::unix::fs::symlink(target, &dst_path)?;
        } else {
            if let Some(parent) = dst_path.parent() {
                std::fs::create_dir_all(parent)?;
            }
            std::fs::copy(src_path, &dst_path)?;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    #[test]
    fn test_deploy_state_save_load() {
        let temp_dir = TempDir::new().unwrap();
        let state_path = Utf8Path::from_path(temp_dir.path())
            .unwrap()
            .join("state.json");

        let mut packages = HashMap::new();
        packages.insert("pkg_a".to_string(), "hash_a".to_string());
        packages.insert("pkg_b".to_string(), "hash_b".to_string());

        let state = DeployState::new(packages.clone());
        state.save(&state_path).unwrap();

        let loaded = DeployState::load(&state_path).unwrap().unwrap();

        assert_eq!(loaded.packages, packages);
    }

    #[test]
    fn test_deploy_state_missing_file() {
        let temp_dir = TempDir::new().unwrap();
        let state_path = Utf8Path::from_path(temp_dir.path())
            .unwrap()
            .join("nonexistent.json");

        let result = DeployState::load(&state_path).unwrap();
        assert!(result.is_none());
    }

    #[test]
    fn test_make_relative_path() {
        // Same level
        assert_eq!(
            make_relative_path(Utf8Path::new("/a/b/c"), Utf8Path::new("/a/b/d")),
            Utf8PathBuf::from("../d")
        );

        // Deeper path
        assert_eq!(
            make_relative_path(Utf8Path::new("/a/b"), Utf8Path::new("/a/b/c/d")),
            Utf8PathBuf::from("c/d")
        );

        // Shallower path
        assert_eq!(
            make_relative_path(Utf8Path::new("/a/b/c/d"), Utf8Path::new("/a/b")),
            Utf8PathBuf::from("../..")
        );
    }

    #[test]
    fn test_copy_dir_recursive() {
        let temp_dir = TempDir::new().unwrap();
        let src = temp_dir.path().join("src");
        let dst = temp_dir.path().join("dst");

        // Create source structure
        fs::create_dir_all(src.join("subdir")).unwrap();
        fs::write(src.join("file1.txt"), "content1").unwrap();
        fs::write(src.join("subdir/file2.txt"), "content2").unwrap();

        let src_path = Utf8Path::from_path(&src).unwrap();
        let dst_path = Utf8Path::from_path(&dst).unwrap();

        copy_dir_recursive(src_path, dst_path).unwrap();

        // Verify
        assert!(dst.join("file1.txt").exists());
        assert!(dst.join("subdir/file2.txt").exists());
        assert_eq!(
            fs::read_to_string(dst.join("file1.txt")).unwrap(),
            "content1"
        );
        assert_eq!(
            fs::read_to_string(dst.join("subdir/file2.txt")).unwrap(),
            "content2"
        );
    }

    #[test]
    fn test_parse_ssh_target() {
        // Standard format without port
        assert_eq!(parse_ssh_target("user@host"), ("user@host", None));

        // With port
        assert_eq!(parse_ssh_target("user@host:22"), ("user@host", Some(22)));
        assert_eq!(
            parse_ssh_target("admin@192.168.1.100:2222"),
            ("admin@192.168.1.100", Some(2222))
        );

        // Edge cases
        assert_eq!(
            parse_ssh_target("user@host:invalid"),
            ("user@host:invalid", None)
        );
        assert_eq!(
            parse_ssh_target("user@host:99999"),
            ("user@host:99999", None)
        ); // Port out of range
    }
}
