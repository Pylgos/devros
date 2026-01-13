//! Parallel execution for package builds
//!
//! This module provides parallel build execution that respects
//! package dependencies, building independent packages concurrently.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use tokio::sync::{Mutex, Semaphore};
use tokio::task::JoinSet;

use crate::Result;
use crate::cache::{CacheManager, compute_package_hash};
use crate::package::{BuildType, Package};
use crate::workspace::Workspace;

use super::ament_cmake::{AmentCmakeBuildOptions, AmentCmakeBuilder};
use super::ament_python::{AmentPythonBuildOptions, AmentPythonBuilder};
use super::progress::BuildProgress;

/// State for tracking parallel build execution
struct BuildState {
    /// Packages that have completed building
    completed: HashSet<String>,
    /// Packages that have failed
    failed: HashSet<String>,
    /// Computed hashes for completed packages
    hashes: HashMap<String, String>,
}

impl BuildState {
    fn new() -> Self {
        Self {
            completed: HashSet::new(),
            failed: HashSet::new(),
            hashes: HashMap::new(),
        }
    }

    /// Check if all dependencies of a package are completed
    fn are_dependencies_ready(
        &self,
        package: &Package,
        workspace_packages: &HashSet<String>,
    ) -> bool {
        for dep in package.build_order_dependencies() {
            // Only check dependencies that are in the workspace
            if workspace_packages.contains(dep) && !self.completed.contains(dep) {
                return false;
            }
        }
        true
    }
}

/// Build result for a single package
#[derive(Debug)]
pub enum PackageBuildResult {
    /// Package was built successfully
    Built { hash: String },
    /// Package build failed
    Failed { error: String },
}

/// Execute parallel builds for packages
pub struct ParallelExecutor<'a> {
    workspace: &'a Workspace,
    cache_manager: CacheManager,
    jobserver: Option<jobserver::Client>,
    jobs: usize,
    symlink_install: bool,
    force_rebuild: bool,
}

impl<'a> ParallelExecutor<'a> {
    /// Create a new parallel executor
    pub fn new(
        workspace: &'a Workspace,
        jobs: usize,
        symlink_install: bool,
        force_rebuild: bool,
        jobserver: Option<jobserver::Client>,
    ) -> Self {
        Self {
            workspace,
            cache_manager: CacheManager::new(&workspace.state_dir()),
            jobserver,
            jobs,
            symlink_install,
            force_rebuild,
        }
    }

    /// Execute builds for the given packages in parallel
    ///
    /// Returns a tuple of (built_packages, skipped_packages)
    pub fn execute(
        &mut self,
        packages_to_build: Vec<String>,
        progress: &mut BuildProgress,
    ) -> Result<(Vec<String>, Vec<String>)> {
        // Create tokio runtime for async execution
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(self.jobs.min(4)) // Limit worker threads
            .enable_all()
            .build()
            .map_err(|e| {
                crate::Error::build(format!("Failed to create async runtime: {}", e), "")
            })?;

        rt.block_on(self.execute_async(packages_to_build, progress))
    }

    async fn execute_async(
        &mut self,
        packages_to_build: Vec<String>,
        progress: &mut BuildProgress,
    ) -> Result<(Vec<String>, Vec<String>)> {
        let workspace_packages: HashSet<String> = self.workspace.packages.keys().cloned().collect();
        let state = Arc::new(Mutex::new(BuildState::new()));
        let semaphore = Arc::new(Semaphore::new(self.jobs));

        let mut pending: HashSet<String> = packages_to_build.iter().cloned().collect();
        let mut built_packages = Vec::new();
        let mut skipped_packages = Vec::new();
        let mut join_set: JoinSet<(String, PackageBuildResult)> = JoinSet::new();

        loop {
            // Check if we're done
            if pending.is_empty() && join_set.is_empty() {
                break;
            }

            // Find packages that are ready to build
            let ready_packages: Vec<String> = {
                let state_guard = state.lock().await;
                pending
                    .iter()
                    .filter(|name| {
                        if let Some(pkg) = self.workspace.packages.get(*name) {
                            state_guard.are_dependencies_ready(pkg, &workspace_packages)
                        } else {
                            false
                        }
                    })
                    .cloned()
                    .collect()
            };

            // Start builds for ready packages
            for package_name in ready_packages {
                pending.remove(&package_name);

                let package = match self.workspace.packages.get(&package_name) {
                    Some(p) => p.clone(),
                    None => continue,
                };

                // Compute hash for cache check
                let dep_hashes: Vec<String> = {
                    let state_guard = state.lock().await;
                    package
                        .build_order_dependencies()
                        .filter_map(|dep| state_guard.hashes.get(dep).cloned())
                        .collect()
                };
                let dep_hash_refs: Vec<&str> = dep_hashes.iter().map(|s| s.as_str()).collect();
                let current_hash = compute_package_hash(&package.path, &dep_hash_refs)?;

                // Check cache
                if !self.force_rebuild
                    && !self
                        .cache_manager
                        .needs_rebuild(&package.name, &current_hash)?
                {
                    tracing::info!("Skipping {} (up to date)", package.name);
                    progress.skip_package(&package.name);

                    let mut state_guard = state.lock().await;
                    state_guard.completed.insert(package.name.clone());
                    state_guard
                        .hashes
                        .insert(package.name.clone(), current_hash.clone());
                    skipped_packages.push(package.name);
                    continue;
                }

                // Start building
                progress.start_package(&package.name, &package.build_type.to_string());
                tracing::info!("Building {} ({})", package.name, package.build_type);

                // Create build context with all necessary data
                let ctx = BuildContext {
                    workspace_root: self.workspace.root.clone(),
                    workspace_config: self.workspace.config.clone(),
                    all_packages: self.workspace.packages.clone(),
                    build_order: self.workspace.build_order.clone(),
                    symlink_install: self.symlink_install,
                    jobs: self.jobs,
                    jobserver: self.jobserver.clone(),
                };

                let state_clone = Arc::clone(&state);
                let sem_clone = Arc::clone(&semaphore);

                join_set.spawn(async move {
                    // Acquire semaphore permit
                    let _permit = sem_clone.acquire().await.expect("Semaphore closed");

                    // Build the package
                    let result = build_package_sync(&ctx, &package, current_hash.clone());

                    // Update state
                    let mut state_guard = state_clone.lock().await;
                    match &result {
                        PackageBuildResult::Built { hash } => {
                            state_guard.completed.insert(package.name.clone());
                            state_guard
                                .hashes
                                .insert(package.name.clone(), hash.clone());
                        }
                        PackageBuildResult::Failed { .. } => {
                            state_guard.failed.insert(package.name.clone());
                        }
                    }

                    (package.name, result)
                });
            }

            // Wait for at least one task to complete
            if let Some(result) = join_set.join_next().await {
                match result {
                    Ok((package_name, PackageBuildResult::Built { hash })) => {
                        progress.finish_package(&package_name);
                        self.cache_manager.mark_success(&package_name, hash)?;
                        built_packages.push(package_name);
                    }
                    Ok((package_name, PackageBuildResult::Failed { error })) => {
                        progress.fail_package(&package_name, &error);
                        // Cancel remaining builds by clearing pending
                        pending.clear();
                        return Err(crate::Error::build(
                            format!("Build failed for {}: {}", package_name, error),
                            "Check the build output for details",
                        ));
                    }
                    Err(e) => {
                        return Err(crate::Error::build(
                            format!("Task panicked: {}", e),
                            "This is likely a bug in devros",
                        ));
                    }
                }
            } else if !pending.is_empty() {
                // No tasks completed but we have pending packages
                // This means we're waiting for dependencies - yield to allow tasks to progress
                tokio::task::yield_now().await;
            }
        }

        progress.finish();
        Ok((built_packages, skipped_packages))
    }
}

/// Context for building a single package
struct BuildContext {
    workspace_root: camino::Utf8PathBuf,
    workspace_config: crate::config::Config,
    all_packages: HashMap<String, Package>,
    build_order: Vec<String>,
    symlink_install: bool,
    jobs: usize,
    jobserver: Option<jobserver::Client>,
}

/// Build a single package synchronously (called from async context)
#[allow(clippy::too_many_arguments)]
fn build_package_sync(
    ctx: &BuildContext,
    package: &Package,
    current_hash: String,
) -> PackageBuildResult {
    // Create a full workspace with all packages for environment computation
    let workspace = Workspace {
        root: ctx.workspace_root.clone(),
        config: ctx.workspace_config.clone(),
        packages: ctx.all_packages.clone(),
        build_order: ctx.build_order.clone(),
    };

    let result = match package.build_type {
        BuildType::AmentCmake => {
            let options = AmentCmakeBuildOptions {
                jobs: ctx.jobs,
                symlink_install: ctx.symlink_install,
                jobserver: ctx.jobserver.as_ref(),
            };
            AmentCmakeBuilder::build(&workspace, package, &options)
        }
        BuildType::AmentPython => {
            let options = AmentPythonBuildOptions {
                symlink_install: ctx.symlink_install,
            };
            AmentPythonBuilder::build(&workspace, package, &options)
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

    match result {
        Ok(()) => PackageBuildResult::Built { hash: current_hash },
        Err(e) => PackageBuildResult::Failed {
            error: e.to_string(),
        },
    }
}
