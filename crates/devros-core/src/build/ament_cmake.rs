//! ament_cmake package builder
//!
//! This module handles building packages that use the ament_cmake build system.

use camino::Utf8PathBuf;
use std::collections::HashMap;
use std::process::Command;

use crate::Result;
use crate::package::Package;
use crate::workspace::Workspace;

use super::environment::compute_build_environment;

/// Builder for ament_cmake packages
pub struct AmentCmakeBuilder;

/// Options for building ament_cmake packages
pub struct AmentCmakeBuildOptions<'a> {
    /// Number of parallel jobs
    pub jobs: usize,
    /// Whether to use symlink install
    pub symlink_install: bool,
    /// Optional jobserver client for coordinating parallel builds
    pub jobserver: Option<&'a jobserver::Client>,
}

impl AmentCmakeBuilder {
    /// Build an ament_cmake package
    pub fn build(
        workspace: &Workspace,
        package: &Package,
        options: &AmentCmakeBuildOptions,
    ) -> Result<()> {
        let build_dir = workspace.package_build_dir(&package.name);
        let install_dir = workspace.package_install_dir(&package.name);
        let source_dir = &package.path;

        // Ensure directories exist
        std::fs::create_dir_all(&build_dir)?;
        std::fs::create_dir_all(&install_dir)?;

        // Build CMake arguments
        let cmake_args = Self::build_cmake_args(
            &build_dir,
            source_dir,
            &install_dir,
            workspace,
            options.symlink_install,
        );

        // Compute environment for this package
        let env = compute_build_environment(workspace, package)?;

        // Configure
        Self::run_configure(&cmake_args, &env, &package.name)?;

        // Build with jobserver support
        Self::run_build(
            &build_dir,
            &env,
            options.jobs,
            options.jobserver,
            &package.name,
        )?;

        // Install
        Self::run_install(&build_dir, &env, &package.name)?;

        Ok(())
    }

    fn build_cmake_args(
        build_dir: &Utf8PathBuf,
        source_dir: &camino::Utf8Path,
        install_dir: &Utf8PathBuf,
        workspace: &Workspace,
        symlink_install: bool,
    ) -> Vec<String> {
        let mut cmake_args = vec![
            "-B".to_string(),
            build_dir.to_string(),
            "-S".to_string(),
            source_dir.to_string(),
            "-G".to_string(),
            "Unix Makefiles".to_string(),
            format!("-DCMAKE_INSTALL_PREFIX={}", install_dir),
        ];

        if let Some(build_type) = &workspace.config.build.ament_cmake.build_type {
            cmake_args.push(format!("-DCMAKE_BUILD_TYPE={}", build_type));
        }

        // Add symlink install flag
        if symlink_install {
            cmake_args.push("-DAMENT_CMAKE_SYMLINK_INSTALL=1".to_string());
        }

        // Add user-specified CMake args
        cmake_args.extend(workspace.config.build.ament_cmake.cmake_args.clone());

        cmake_args
    }

    fn run_configure(
        cmake_args: &[String],
        env: &HashMap<String, String>,
        package_name: &str,
    ) -> Result<()> {
        tracing::debug!("Running cmake configure: cmake {:?}", cmake_args);
        let status = Command::new("cmake").args(cmake_args).envs(env).status()?;

        if !status.success() {
            return Err(crate::Error::build(
                format!("CMake configure failed for {}", package_name),
                "Check the CMake output for errors",
            ));
        }

        Ok(())
    }

    fn run_build(
        build_dir: &Utf8PathBuf,
        env: &HashMap<String, String>,
        jobs: usize,
        jobserver: Option<&jobserver::Client>,
        package_name: &str,
    ) -> Result<()> {
        tracing::debug!("Running cmake build with {} jobs", jobs);
        let mut build_cmd = Command::new("cmake");
        build_cmd.args([
            "--build",
            build_dir.as_str(),
            "--parallel",
            &jobs.to_string(),
        ]);
        build_cmd.envs(env);

        // Configure jobserver for the build command
        if let Some(js) = jobserver {
            js.configure(&mut build_cmd);
            tracing::debug!("Configured jobserver for cmake build");
        }

        let status = build_cmd.status()?;

        if !status.success() {
            return Err(crate::Error::build(
                format!("CMake build failed for {}", package_name),
                "Check the build output for errors",
            ));
        }

        Ok(())
    }

    fn run_install(
        build_dir: &Utf8PathBuf,
        env: &HashMap<String, String>,
        package_name: &str,
    ) -> Result<()> {
        // Note: We must set current_dir to build_dir to ensure symlink_install_manifest.txt
        // is created in the build directory, not in the current working directory
        tracing::debug!("Running cmake install");
        let status = Command::new("cmake")
            .args(["--install", build_dir.as_str()])
            .current_dir(build_dir)
            .envs(env)
            .status()?;

        if !status.success() {
            return Err(crate::Error::build(
                format!("CMake install failed for {}", package_name),
                "Check the install output for errors",
            ));
        }

        Ok(())
    }
}
