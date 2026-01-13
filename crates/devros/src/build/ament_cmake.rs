//! ament_cmake package builder
//!
//! This module handles building packages that use the ament_cmake build system.

use camino::Utf8PathBuf;
use std::collections::HashMap;
use tokio::process::Command;

use crate::Result;
use crate::dsv::write_colcon_marker_file;
use crate::package::Package;
use crate::workspace::Workspace;

use super::command_logger::{LogCallback, run_command_with_logging};
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
    /// Optional callback for log lines
    pub log_callback: Option<LogCallback>,
}

impl AmentCmakeBuilder {
    /// Build an ament_cmake package
    pub async fn build(
        workspace: &Workspace,
        package: &Package,
        options: &AmentCmakeBuildOptions<'_>,
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
        Self::run_configure(
            &cmake_args,
            &env,
            &package.name,
            options.log_callback.clone(),
        )
        .await?;

        // Build with jobserver support
        Self::run_build(
            &build_dir,
            &env,
            options.jobs,
            options.jobserver,
            &package.name,
            options.log_callback.clone(),
        )
        .await?;

        // Install
        Self::run_install(
            &build_dir,
            &env,
            &package.name,
            options.log_callback.clone(),
        )
        .await?;

        // Generate colcon marker file for this package
        Self::generate_colcon_files(workspace, package)?;

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

    async fn run_configure(
        cmake_args: &[String],
        env: &HashMap<String, String>,
        package_name: &str,
        log_callback: Option<LogCallback>,
    ) -> Result<()> {
        tracing::debug!("Running cmake configure: cmake {:?}", cmake_args);
        let mut cmd = Command::new("cmake");
        cmd.args(cmake_args).envs(env);

        run_command_with_logging(&mut cmd, package_name, "CMake configure", log_callback).await
    }

    async fn run_build(
        build_dir: &Utf8PathBuf,
        env: &HashMap<String, String>,
        jobs: usize,
        jobserver: Option<&jobserver::Client>,
        package_name: &str,
        log_callback: Option<LogCallback>,
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
            js.configure(build_cmd.as_std_mut());
            tracing::debug!("Configured jobserver for cmake build");
        }

        run_command_with_logging(&mut build_cmd, package_name, "CMake build", log_callback).await
    }

    async fn run_install(
        build_dir: &Utf8PathBuf,
        env: &HashMap<String, String>,
        package_name: &str,
        log_callback: Option<LogCallback>,
    ) -> Result<()> {
        // Note: We must set current_dir to build_dir to ensure symlink_install_manifest.txt
        // is created in the build directory, not in the current working directory
        tracing::debug!("Running cmake install");
        let mut cmd = Command::new("cmake");
        cmd.args(["--install", build_dir.as_str()])
            .current_dir(build_dir)
            .envs(env);

        run_command_with_logging(&mut cmd, package_name, "CMake install", log_callback).await
    }

    /// Generate colcon-compatible files for the package
    fn generate_colcon_files(workspace: &Workspace, package: &Package) -> Result<()> {
        let install_dir = workspace.package_install_dir(&package.name);

        // Get all runtime dependencies and sort them alphabetically
        // (colcon sorts dependencies alphabetically in the marker file)
        let mut run_deps: Vec<String> = package.run_dependencies().map(|s| s.to_string()).collect();
        run_deps.sort();
        let run_deps_refs: Vec<&str> = run_deps.iter().map(|s| s.as_str()).collect();

        // Write colcon marker file
        write_colcon_marker_file(&install_dir, &package.name, &run_deps_refs)?;

        // Generate cmake_prefix_path hooks for CMake packages
        // This is needed so CMake can find the package's config files
        Self::generate_cmake_prefix_path_hooks(&install_dir, &package.name)?;

        // Generate ld_library_path hooks if the package has shared libraries
        // This is needed so the shared libraries can be found at runtime
        Self::generate_ld_library_path_hooks(&install_dir, &package.name)?;

        // For ament_cmake packages, ament_cmake generates local_setup.* files during CMake install
        // We need to update package.dsv to also include hooks
        let share_dir = install_dir.join("share").join(&package.name);
        let package_dsv_path = share_dir.join("package.dsv");

        // Generate/update package.dsv with all hooks
        Self::update_package_dsv_with_hooks(&package_dsv_path, &package.name, &install_dir)?;

        Ok(())
    }

    /// Generate cmake_prefix_path hooks for CMake packages
    fn generate_cmake_prefix_path_hooks(
        install_dir: &camino::Utf8PathBuf,
        package_name: &str,
    ) -> Result<()> {
        let hook_dir = install_dir.join("share").join(package_name).join("hook");
        std::fs::create_dir_all(&hook_dir)?;

        // Generate hook/cmake_prefix_path.dsv
        std::fs::write(
            hook_dir.join("cmake_prefix_path.dsv"),
            "prepend-non-duplicate;CMAKE_PREFIX_PATH;\n",
        )?;

        // Generate hook/cmake_prefix_path.sh (colcon-compatible)
        std::fs::write(
            hook_dir.join("cmake_prefix_path.sh"),
            r#"# generated from colcon_core/shell/template/hook_prepend_value.sh.em

_colcon_prepend_unique_value CMAKE_PREFIX_PATH "$COLCON_CURRENT_PREFIX"
"#,
        )?;

        tracing::debug!("Generated cmake_prefix_path hooks in {}", hook_dir);
        Ok(())
    }

    /// Generate ld_library_path hooks for packages with shared libraries
    ///
    /// This mimics colcon_library_path behavior: if the lib directory contains
    /// any .so files, generate hooks to add it to LD_LIBRARY_PATH.
    fn generate_ld_library_path_hooks(
        install_dir: &camino::Utf8PathBuf,
        package_name: &str,
    ) -> Result<()> {
        let lib_dir = install_dir.join("lib");

        // Check if the lib directory contains any .so files
        if !lib_dir.exists() {
            return Ok(());
        }

        let has_shared_libs = std::fs::read_dir(&lib_dir)?
            .filter_map(|e| e.ok())
            .any(|entry| entry.path().extension().is_some_and(|ext| ext == "so"));

        if !has_shared_libs {
            return Ok(());
        }

        let hook_dir = install_dir.join("share").join(package_name).join("hook");
        std::fs::create_dir_all(&hook_dir)?;

        // Generate hook/ld_library_path_lib.dsv
        std::fs::write(
            hook_dir.join("ld_library_path_lib.dsv"),
            "prepend-non-duplicate;LD_LIBRARY_PATH;lib\n",
        )?;

        // Generate hook/ld_library_path_lib.sh (colcon-compatible)
        std::fs::write(
            hook_dir.join("ld_library_path_lib.sh"),
            r#"# generated from colcon_core/shell/template/hook_prepend_value.sh.em

_colcon_prepend_unique_value LD_LIBRARY_PATH "$COLCON_CURRENT_PREFIX/lib"
"#,
        )?;

        tracing::debug!(
            "Generated ld_library_path_lib hooks in {} for package {}",
            hook_dir,
            package_name
        );
        Ok(())
    }

    /// Update package.dsv to include all required hooks
    fn update_package_dsv_with_hooks(
        package_dsv_path: &camino::Utf8PathBuf,
        package_name: &str,
        install_dir: &camino::Utf8PathBuf,
    ) -> Result<()> {
        let share_dir = package_dsv_path.parent().unwrap();
        std::fs::create_dir_all(share_dir)?;

        // Read existing content if present
        let existing_content = if package_dsv_path.exists() {
            std::fs::read_to_string(package_dsv_path)?
        } else {
            String::new()
        };

        let mut lines: Vec<String> = existing_content.lines().map(|s| s.to_string()).collect();
        let mut modified = false;

        // Check and add cmake_prefix_path hooks
        let cmake_hook_dsv = format!("source;share/{}/hook/cmake_prefix_path.dsv", package_name);
        let cmake_hook_sh = format!("source;share/{}/hook/cmake_prefix_path.sh", package_name);

        if !existing_content.contains(&cmake_hook_dsv) {
            lines.insert(0, cmake_hook_dsv);
            modified = true;
        }
        if !existing_content.contains(&cmake_hook_sh) {
            lines.insert(1, cmake_hook_sh);
            modified = true;
        }

        // Check and add ld_library_path_lib hooks if the hook files exist
        let hook_dir = install_dir.join("share").join(package_name).join("hook");
        let ld_lib_dsv_path = hook_dir.join("ld_library_path_lib.dsv");
        let ld_lib_sh_path = hook_dir.join("ld_library_path_lib.sh");

        if ld_lib_dsv_path.exists() && ld_lib_sh_path.exists() {
            let ld_hook_dsv = format!("source;share/{}/hook/ld_library_path_lib.dsv", package_name);
            let ld_hook_sh = format!("source;share/{}/hook/ld_library_path_lib.sh", package_name);

            if !existing_content.contains(&ld_hook_dsv) {
                // Insert after cmake hooks (after position 2)
                let insert_pos = std::cmp::min(2, lines.len());
                lines.insert(insert_pos, ld_hook_dsv);
                modified = true;
            }
            if !existing_content.contains(&ld_hook_sh) {
                // Insert after the dsv entry
                let insert_pos = std::cmp::min(3, lines.len());
                lines.insert(insert_pos, ld_hook_sh);
                modified = true;
            }
        }

        // Write back if modified
        if modified {
            let content = lines.join("\n") + "\n";
            std::fs::write(package_dsv_path, content)?;
            tracing::debug!("Updated {} with hooks", package_dsv_path);
        }

        Ok(())
    }
}
