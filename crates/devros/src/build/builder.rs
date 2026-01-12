//! Build orchestration
//!
//! This module provides the main build orchestration logic that:
//! - Determines build order from dependencies
//! - Dispatches to appropriate build type handlers
//! - Manages build cache for incremental builds
//! - Generates workspace-level setup files

use std::collections::HashMap;

use crate::Result;
use crate::cache::{CacheManager, compute_package_hash};
use crate::dsv::generate_workspace_setup_bash;
use crate::package::BuildType;
use crate::workspace::Workspace;

use super::ament_cmake::{AmentCmakeBuildOptions, AmentCmakeBuilder};
use super::ament_python::{AmentPythonBuildOptions, AmentPythonBuilder};

/// Arguments for the build operation
#[derive(Debug, Clone)]
pub struct BuildArgs {
    /// Build specific packages only
    pub packages: Option<Vec<String>>,
    /// Number of parallel jobs
    pub jobs: Option<usize>,
    /// Use symlink install (default: true)
    pub symlink_install: bool,
    /// Dry run - show what would be built
    pub dry_run: bool,
    /// Force rebuild even if cache is valid
    pub force_rebuild: bool,
}

impl Default for BuildArgs {
    fn default() -> Self {
        Self {
            packages: None,
            jobs: None,
            symlink_install: true,
            dry_run: false,
            force_rebuild: false,
        }
    }
}

/// Result of a build operation
#[derive(Debug)]
pub struct BuildResult {
    /// Packages that were built
    pub built_packages: Vec<String>,
    /// Packages that were skipped (cache hit)
    pub skipped_packages: Vec<String>,
}

/// Main builder for ROS 2 workspaces
pub struct Builder<'a> {
    workspace: &'a Workspace,
    cache_manager: CacheManager,
    /// Optional jobserver client for coordinating parallel builds
    jobserver: Option<jobserver::Client>,
}

impl<'a> Builder<'a> {
    /// Create a new builder for the given workspace
    pub fn new(workspace: &'a Workspace) -> Self {
        Self {
            workspace,
            cache_manager: CacheManager::new(&workspace.state_dir()),
            jobserver: None,
        }
    }

    /// Set the jobserver client for parallel build coordination
    pub fn with_jobserver(mut self, client: jobserver::Client) -> Self {
        self.jobserver = Some(client);
        self
    }

    /// Execute the build
    pub fn build(&mut self, args: &BuildArgs) -> Result<BuildResult> {
        tracing::info!("Discovering workspace at {}", self.workspace.root);
        tracing::info!("Found {} packages", self.workspace.packages.len());

        // Determine which packages to build
        let packages_to_build: Vec<_> = if let Some(ref names) = args.packages {
            self.workspace
                .build_order
                .iter()
                .filter(|name| names.contains(name))
                .cloned()
                .collect()
        } else {
            self.workspace.build_order.clone()
        };

        if packages_to_build.is_empty() {
            tracing::warn!("No packages to build");
            return Ok(BuildResult {
                built_packages: Vec::new(),
                skipped_packages: Vec::new(),
            });
        }

        tracing::info!("Build order: {:?}", packages_to_build);

        if args.dry_run {
            println!("Would build the following packages in order:");
            for name in &packages_to_build {
                if let Some(pkg) = self.workspace.packages.get(name) {
                    println!("  - {} ({}) at {}", pkg.name, pkg.build_type, pkg.path);
                }
            }
            return Ok(BuildResult {
                built_packages: Vec::new(),
                skipped_packages: packages_to_build,
            });
        }

        // Create COLCON_IGNORE in workspace build directory
        let build_dir = self
            .workspace
            .root
            .join(&self.workspace.config.workspace.build_dir);
        std::fs::create_dir_all(&build_dir)?;
        std::fs::File::create(build_dir.join("COLCON_IGNORE"))?;

        // Get effective jobs
        let jobs = args
            .jobs
            .unwrap_or_else(|| self.workspace.config.effective_jobs());

        tracing::info!("Using {} parallel jobs", jobs);

        // Store computed hashes for dependency calculation
        let mut package_hashes: HashMap<String, String> = HashMap::new();
        let mut built_packages = Vec::new();
        let mut skipped_packages = Vec::new();

        // Build each package
        for package_name in packages_to_build {
            let package = self
                .workspace
                .packages
                .get(&package_name)
                .ok_or_else(|| crate::Error::package("Package not found", &package_name))?;

            // Compute hash for this package
            let dep_hashes: Vec<&str> = package
                .build_order_dependencies()
                .filter_map(|dep| package_hashes.get(dep).map(|s| s.as_str()))
                .collect();

            let current_hash = compute_package_hash(&package.path, &dep_hashes)?;

            // Check cache (unless force rebuild)
            if !args.force_rebuild
                && !self
                    .cache_manager
                    .needs_rebuild(&package.name, &current_hash)?
            {
                tracing::info!("Skipping {} (up to date)", package.name);
                package_hashes.insert(package.name.clone(), current_hash);
                skipped_packages.push(package_name);
                continue;
            }

            tracing::info!("Building {} ({})", package.name, package.build_type);

            let build_result = match package.build_type {
                BuildType::AmentCmake => {
                    let options = AmentCmakeBuildOptions {
                        jobs,
                        symlink_install: args.symlink_install,
                        jobserver: self.jobserver.as_ref(),
                    };
                    AmentCmakeBuilder::build(self.workspace, package, &options)
                }
                BuildType::AmentPython => {
                    let options = AmentPythonBuildOptions {
                        symlink_install: args.symlink_install,
                    };
                    AmentPythonBuilder::build(self.workspace, package, &options)
                }
                BuildType::Cmake => {
                    tracing::warn!("cmake build not yet implemented for {}", package.name);
                    Ok(())
                }
                BuildType::Other(ref t) => {
                    tracing::warn!("Unknown build type '{}' for {}, skipping", t, package.name);
                    Ok(())
                }
            };

            // Update cache based on result
            match build_result {
                Ok(()) => {
                    self.cache_manager
                        .mark_success(&package.name, current_hash.clone())?;
                    package_hashes.insert(package.name.clone(), current_hash);
                    built_packages.push(package_name);
                }
                Err(e) => {
                    self.cache_manager
                        .mark_failed(&package.name, current_hash)?;
                    return Err(e);
                }
            }
        }

        // Generate workspace setup.bash
        self.generate_workspace_setup()?;

        // Create COLCON_IGNORE in workspace install directory
        let install_dir = self
            .workspace
            .root
            .join(&self.workspace.config.workspace.install_dir);
        std::fs::create_dir_all(&install_dir)?;
        std::fs::File::create(install_dir.join("COLCON_IGNORE"))?;

        tracing::info!("Build complete!");

        Ok(BuildResult {
            built_packages,
            skipped_packages,
        })
    }

    /// Generate workspace-level setup.bash
    fn generate_workspace_setup(&self) -> Result<()> {
        let install_dir = self
            .workspace
            .root
            .join(&self.workspace.config.workspace.install_dir);
        std::fs::create_dir_all(&install_dir)?;

        let content = generate_workspace_setup_bash();
        let setup_path = install_dir.join("setup.bash");
        std::fs::write(&setup_path, content)?;

        // Make executable
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mut perms = std::fs::metadata(&setup_path)?.permissions();
            perms.set_mode(0o755);
            std::fs::set_permissions(&setup_path, perms)?;
        }

        tracing::info!("Generated {}", setup_path);
        Ok(())
    }
}
