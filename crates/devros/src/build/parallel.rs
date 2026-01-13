//! Parallel execution for package builds
//!
//! This module provides parallel build execution that respects
//! package dependencies, building independent packages concurrently.

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::sync::mpsc;

use crate::Result;
use crate::cache::{CacheManager, compute_package_hash};
use crate::package::{BuildType, Package};
use crate::workspace::Workspace;

use super::ament_cmake::{AmentCmakeBuildOptions, AmentCmakeBuilder};
use super::ament_python::{AmentPythonBuildOptions, AmentPythonBuilder};
use super::progress::BuildProgress;

/// Result of a package build task
#[derive(Debug)]
enum TaskResult {
    /// Package was skipped (cache hit)
    Skipped { name: String, hash: String },
    /// Package was built successfully
    Built { name: String, hash: String },
    /// Package build failed
    Failed { name: String, error: String },
}

/// Execute parallel builds for packages
pub struct ParallelExecutor<'a> {
    workspace: &'a Workspace,
    cache_manager: Arc<Mutex<CacheManager>>,
    jobserver: Option<jobserver::Client>,
    jobs: usize,
    symlink_install: bool,
    force_rebuild: bool,

    // Shared read-only data
    config: Arc<crate::config::Config>,
    packages: Arc<HashMap<String, Package>>,
    build_order: Arc<Vec<String>>,
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
            cache_manager: Arc::new(Mutex::new(CacheManager::new(&workspace.state_dir()))),
            jobserver,
            jobs,
            symlink_install,
            force_rebuild,
            config: Arc::new(workspace.config.clone()),
            packages: Arc::new(workspace.packages.clone()),
            build_order: Arc::new(workspace.build_order.clone()),
        }
    }

    /// Execute builds for the given packages in parallel
    ///
    /// Returns a tuple of (built_packages, skipped_packages)
    pub fn execute(
        &mut self,
        packages_to_build: Vec<String>,
        progress: &BuildProgress,
    ) -> Result<(Vec<String>, Vec<String>)> {
        // Create tokio runtime for async execution
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(self.jobs)
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
        progress: &BuildProgress,
    ) -> Result<(Vec<String>, Vec<String>)> {
        let mut built_packages = Vec::new();
        let mut skipped_packages = Vec::new();
        // Hashes of packages completed in this run (built or skipped)
        let mut package_hashes: HashMap<String, String> = HashMap::new();

        // 1. Build Dependency Graph
        let mut dependents: HashMap<String, Vec<String>> = HashMap::new();
        let mut in_degree: HashMap<String, usize> = HashMap::new();
        let package_set: HashSet<String> = packages_to_build.iter().cloned().collect();

        for pkg_name in &packages_to_build {
            in_degree.insert(pkg_name.clone(), 0);
        }

        for pkg_name in &packages_to_build {
            let pkg = self.packages.get(pkg_name).unwrap();
            for dep_name in pkg.build_order_dependencies() {
                if package_set.contains(dep_name) {
                    in_degree.entry(pkg_name.clone()).and_modify(|d| *d += 1);
                    dependents
                        .entry(dep_name.to_string())
                        .or_default()
                        .push(pkg_name.clone());
                }
            }
        }

        // 2. Initialize Queue with packages that have 0 dependencies in the set
        let mut queue: VecDeque<String> = VecDeque::new();
        for pkg_name in &packages_to_build {
            if *in_degree.get(pkg_name).unwrap() == 0 {
                queue.push_back(pkg_name.clone());
            }
        }

        // 3. Execution Loop
        let (tx, mut rx) = mpsc::channel(self.jobs + 1);
        let mut active_jobs = 0;
        let mut remaining_packages = packages_to_build.len();

        while remaining_packages > 0 {
            // Schedule new jobs up to limit
            while active_jobs < self.jobs && !queue.is_empty() {
                let pkg_name = queue.pop_front().unwrap();
                let pkg = self.packages.get(&pkg_name).unwrap().clone();

                // Collect dependency hashes available from this run
                let dep_hashes: Vec<String> = pkg
                    .build_order_dependencies()
                    .filter_map(|dep| package_hashes.get(dep).cloned())
                    .collect();

                let ctx = Arc::new(BuildContext {
                    workspace_root: self.workspace.root.clone(),
                    workspace_config: Arc::clone(&self.config),
                    all_packages: Arc::clone(&self.packages),
                    build_order: Arc::clone(&self.build_order),
                    symlink_install: self.symlink_install,
                    jobs: self.jobs,
                    jobserver: self.jobserver.clone(),
                    progress: progress.clone(),
                    cache_manager: Arc::clone(&self.cache_manager),
                    force_rebuild: self.force_rebuild,
                });

                let tx = tx.clone();
                active_jobs += 1;

                tokio::spawn(async move {
                    let result = build_task(ctx, pkg, dep_hashes).await;
                    let _ = tx.send(result).await;
                });
            }

            // Check for deadlock
            if active_jobs == 0 && remaining_packages > 0 {
                return Err(crate::Error::build(
                    "Build deadlocked: packages remain but none are ready to build.",
                    "This indicates a circular dependency or a bug in dependency resolution.",
                ));
            }

            // Wait for next completion
            if let Some(result) = rx.recv().await {
                active_jobs -= 1;
                match result {
                    TaskResult::Built { name, hash } => {
                        progress.finish_package(&name);
                        built_packages.push(name.clone());
                        package_hashes.insert(name.clone(), hash);

                        // Unlock dependents
                        if let Some(deps) = dependents.get(&name) {
                            for dep in deps {
                                if let Some(degree) = in_degree.get_mut(dep) {
                                    *degree -= 1;
                                    if *degree == 0 {
                                        queue.push_back(dep.clone());
                                    }
                                }
                            }
                        }
                        remaining_packages -= 1;
                    }
                    TaskResult::Skipped { name, hash } => {
                        skipped_packages.push(name.clone());
                        package_hashes.insert(name.clone(), hash);

                        // Unlock dependents (same as built)
                        if let Some(deps) = dependents.get(&name) {
                            for dep in deps {
                                if let Some(degree) = in_degree.get_mut(dep) {
                                    *degree -= 1;
                                    if *degree == 0 {
                                        queue.push_back(dep.clone());
                                    }
                                }
                            }
                        }
                        remaining_packages -= 1;
                    }
                    TaskResult::Failed { name, error } => {
                        progress.fail_package(&name, &error);
                        return Err(crate::Error::build(
                            format!("Build failed for {}: {}", name, error),
                            "Check the build output for details",
                        ));
                    }
                }
            } else {
                break;
            }
        }

        progress.finish();
        Ok((built_packages, skipped_packages))
    }
}

/// Context for building a single package
struct BuildContext {
    workspace_root: camino::Utf8PathBuf,
    workspace_config: Arc<crate::config::Config>,
    all_packages: Arc<HashMap<String, Package>>,
    build_order: Arc<Vec<String>>,
    symlink_install: bool,
    jobs: usize,
    jobserver: Option<jobserver::Client>,
    progress: BuildProgress,
    cache_manager: Arc<Mutex<CacheManager>>,
    force_rebuild: bool,
}

async fn build_task(
    ctx: Arc<BuildContext>,
    package: Package,
    dep_hashes: Vec<String>,
) -> TaskResult {
    // 1. Compute Hash
    let dep_hash_refs: Vec<&str> = dep_hashes.iter().map(|s| s.as_str()).collect();
    let current_hash = match compute_package_hash(&package.path, &dep_hash_refs) {
        Ok(h) => h,
        Err(e) => {
            return TaskResult::Failed {
                name: package.name,
                error: e.to_string(),
            };
        }
    };

    // 2. Check Cache
    if !ctx.force_rebuild {
        let mut cm = ctx.cache_manager.lock().await;
        match cm.needs_rebuild(&package.name, &current_hash) {
            Ok(false) => {
                tracing::info!("Skipping {} (up to date)", package.name);
                ctx.progress.skip_package(&package.name);
                return TaskResult::Skipped {
                    name: package.name,
                    hash: current_hash,
                };
            }
            Ok(true) => {} // Proceed to build
            Err(e) => {
                return TaskResult::Failed {
                    name: package.name,
                    error: e.to_string(),
                };
            }
        }
    }

    // 3. Build
    ctx.progress
        .start_package(&package.name, &package.build_type.to_string());
    tracing::info!("Building {} ({})", package.name, package.build_type);

    let package_clone = package.clone();

    // Call async build
    let build_result = build_package_async(&ctx, &package_clone).await;

    match build_result {
        Ok(()) => {
            // Mark success in cache
            let mut cm = ctx.cache_manager.lock().await;
            match cm.mark_success(&package.name, current_hash.clone()) {
                Ok(_) => TaskResult::Built {
                    name: package.name,
                    hash: current_hash,
                },
                Err(e) => TaskResult::Failed {
                    name: package.name,
                    error: e.to_string(),
                },
            }
        }
        Err(e) => TaskResult::Failed {
            name: package.name,
            error: e.to_string(),
        },
    }
}

/// Build a single package asynchronously
#[allow(clippy::too_many_arguments)]
async fn build_package_async(ctx: &BuildContext, package: &Package) -> Result<()> {
    // Reconstruct workspace for this thread
    let workspace = Workspace {
        root: ctx.workspace_root.clone(),
        config: (*ctx.workspace_config).clone(),
        packages: (*ctx.all_packages).clone(),
        build_order: (*ctx.build_order).clone(),
    };

    let package_name = package.name.clone();
    let progress = ctx.progress.clone();
    let log_callback = Arc::new(move |line: &str| {
        progress.update_package_log(&package_name, line);
    });

    match package.build_type {
        BuildType::AmentCmake => {
            let options = AmentCmakeBuildOptions {
                jobs: ctx.jobs,
                symlink_install: ctx.symlink_install,
                jobserver: ctx.jobserver.as_ref(),
                log_callback: Some(log_callback),
            };
            AmentCmakeBuilder::build(&workspace, package, &options).await
        }
        BuildType::AmentPython => {
            let options = AmentPythonBuildOptions {
                symlink_install: ctx.symlink_install,
                log_callback: Some(log_callback),
            };
            AmentPythonBuilder::build(&workspace, package, &options).await
        }
        BuildType::Cmake => {
            tracing::warn!("cmake build not yet implemented for {}", package.name);
            Ok(())
        }
        BuildType::Other(ref t) => {
            tracing::warn!("Unknown build type '{}' for {}, skipping", t, package.name);
            Ok(())
        }
    }
}
