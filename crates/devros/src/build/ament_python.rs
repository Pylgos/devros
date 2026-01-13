//! ament_python package builder
//!
//! This module handles building packages that use the ament_python build system.

use camino::{Utf8Path, Utf8PathBuf};
use std::collections::HashMap;
use tokio::process::Command;

use crate::Result;
use crate::dsv::{generate_ament_python_package_dsv, write_colcon_marker_file};
use crate::package::Package;
use crate::workspace::Workspace;

use super::command_logger::{LogCallback, run_command_with_logging};
use super::environment::compute_build_environment;

/// Builder for ament_python packages
pub struct AmentPythonBuilder;

/// Options for building ament_python packages
pub struct AmentPythonBuildOptions {
    /// Whether to use symlink install (develop mode)
    pub symlink_install: bool,
    /// Optional callback for log lines
    pub log_callback: Option<LogCallback>,
}

impl AmentPythonBuilder {
    /// Build an ament_python package
    pub async fn build(
        workspace: &Workspace,
        package: &Package,
        options: &AmentPythonBuildOptions,
    ) -> Result<()> {
        let build_dir = workspace.package_build_dir(&package.name);
        let install_dir = workspace.package_install_dir(&package.name);
        let source_dir = &package.path;

        // Ensure directories exist
        std::fs::create_dir_all(&build_dir)?;
        std::fs::create_dir_all(&install_dir)?;

        // Get Python lib directory path
        let python_lib_dir = get_python_lib_dir().await?;
        let python_lib_path = install_dir.join(&python_lib_dir);
        std::fs::create_dir_all(&python_lib_path)?;

        // Create prefix_override directory with sitecustomize.py
        let prefix_override = build_dir.join("prefix_override");
        std::fs::create_dir_all(&prefix_override)?;

        // Generate sitecustomize.py to redirect installation
        let sitecustomize_content = generate_sitecustomize(&install_dir)?;
        std::fs::write(
            prefix_override.join("sitecustomize.py"),
            sitecustomize_content,
        )?;

        // Compute environment for this package
        let mut env = compute_build_environment(workspace, package)?;

        // Set PYTHONPATH with proper priority:
        // 1. prefix_override (for sitecustomize.py)
        // 2. Python lib path (for self-dependency resolution)
        // 3. Existing PYTHONPATH
        let existing_pythonpath = env.get("PYTHONPATH").cloned().unwrap_or_default();
        let new_pythonpath = format!(
            "{}:{}:{}",
            prefix_override, python_lib_path, existing_pythonpath
        );
        env.insert("PYTHONPATH".to_string(), new_pythonpath);

        // Remove coverage-related env vars that interfere with sitecustomize
        env.remove("COV_CORE_SOURCE");

        if options.symlink_install {
            // Symlink install mode: use develop command
            Self::build_develop(
                package,
                &build_dir,
                &install_dir,
                source_dir,
                &env,
                options.log_callback.clone(),
            )
            .await?;
        } else {
            // Standard install mode
            Self::build_install(
                package,
                &build_dir,
                &install_dir,
                source_dir,
                &env,
                options.log_callback.clone(),
            )
            .await?;
        }

        // Post-processing: ensure package.xml is installed
        ensure_package_xml_installed(package, &install_dir, options.symlink_install)?;

        // Ensure ament resource index entry exists
        ensure_ament_resource_index(package, &install_dir)?;

        // Generate colcon-compatible package.dsv and hook files
        generate_python_package_environment_files(workspace, package, options.symlink_install)
            .await?;

        Ok(())
    }

    /// Build ament_python package using setup.py install (non-symlink mode)
    async fn build_install(
        package: &Package,
        build_dir: &Utf8PathBuf,
        install_dir: &Utf8PathBuf,
        source_dir: &Utf8Path,
        env: &HashMap<String, String>,
        log_callback: Option<LogCallback>,
    ) -> Result<()> {
        // Run egg_info
        let egg_base = build_dir.to_string();
        let mut cmd = Command::new("python3");
        cmd.args([
            "-W",
            "ignore:setup.py install is deprecated",
            "-W",
            "ignore:easy_install command is deprecated",
            "setup.py",
            "egg_info",
            "--egg-base",
            &egg_base,
        ])
        .current_dir(source_dir)
        .envs(env);

        run_command_with_logging(
            &mut cmd,
            &package.name,
            "setup.py egg_info",
            log_callback.clone(),
        )
        .await?;

        // Run build and install
        let build_base = build_dir.join("build");
        let install_log = build_dir.join("install.log");

        let mut cmd = Command::new("python3");
        cmd.args([
            "-W",
            "ignore:setup.py install is deprecated",
            "-W",
            "ignore:easy_install command is deprecated",
            "setup.py",
            "build",
            "--build-base",
            build_base.as_str(),
            "install",
            "--prefix",
            install_dir.as_str(),
            "--record",
            install_log.as_str(),
            "--single-version-externally-managed",
        ])
        .current_dir(source_dir)
        .envs(env);

        run_command_with_logging(&mut cmd, &package.name, "setup.py install", log_callback).await
    }

    /// Build ament_python package using setup.py develop (symlink mode)
    async fn build_develop(
        package: &Package,
        build_dir: &Utf8PathBuf,
        install_dir: &Utf8PathBuf,
        source_dir: &Utf8Path,
        env: &HashMap<String, String>,
        log_callback: Option<LogCallback>,
    ) -> Result<()> {
        // Create symlinks from source to build directory
        create_python_build_symlinks(source_dir, build_dir)?;

        // Run develop command from build directory
        let build_base = build_dir.join("build");

        let mut cmd = Command::new("python3");
        cmd.args([
            "-W",
            "ignore:setup.py install is deprecated",
            "-W",
            "ignore:easy_install command is deprecated",
            "setup.py",
            "develop",
            "--prefix",
            install_dir.as_str(),
            "--build-directory",
            build_base.as_str(),
            "--editable",
            "--no-deps",
        ])
        .current_dir(build_dir)
        .envs(env);

        run_command_with_logging(&mut cmd, &package.name, "setup.py develop", log_callback).await
    }
}

/// Get Python version string (e.g., "3.12")
async fn get_python_version() -> Result<String> {
    let output = Command::new("python3")
        .args([
            "-c",
            "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')",
        ])
        .output()
        .await?;

    if !output.status.success() {
        return Err(crate::Error::build(
            "Failed to get Python version",
            "Ensure python3 is installed and available in PATH",
        ));
    }

    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

/// Get Python lib directory (e.g., "lib/python3.12/site-packages")
async fn get_python_lib_dir() -> Result<String> {
    let version = get_python_version().await?;
    Ok(format!("lib/python{}/site-packages", version))
}

/// Generate sitecustomize.py content to redirect Python installation prefix
fn generate_sitecustomize(install_dir: &Utf8PathBuf) -> Result<String> {
    Ok(format!(
        r#"# Generated by devros - redirects Python installation to install directory
import sys
sys.prefix = '{install_dir}'
sys.exec_prefix = '{install_dir}'
"#,
        install_dir = install_dir
    ))
}

/// Create symlinks from source directory to build directory for Python develop mode
fn create_python_build_symlinks(source_dir: &Utf8Path, build_dir: &Utf8PathBuf) -> Result<()> {
    // Files to symlink
    let files_to_symlink = ["setup.py", "setup.cfg", "package.xml"];

    for file in &files_to_symlink {
        let src = source_dir.join(file);
        let dst = build_dir.join(file);

        if src.exists() {
            // Remove existing symlink/file if it exists
            if dst.exists() || dst.is_symlink() {
                std::fs::remove_file(&dst)?;
            }
            std::os::unix::fs::symlink(&src, &dst)?;
        }
    }

    // Symlink resource directory if it exists
    let resource_src = source_dir.join("resource");
    let resource_dst = build_dir.join("resource");
    if resource_src.exists() {
        if resource_dst.exists() || resource_dst.is_symlink() {
            if resource_dst.is_symlink() {
                std::fs::remove_file(&resource_dst)?;
            } else {
                std::fs::remove_dir_all(&resource_dst)?;
            }
        }
        std::os::unix::fs::symlink(&resource_src, &resource_dst)?;
    }

    // Symlink Python package directories
    // Look for directories that could be Python packages (contain __init__.py)
    for entry in std::fs::read_dir(source_dir)? {
        let entry = entry?;
        let path = entry.path();

        if path.is_dir() {
            let dir_name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");

            // Skip hidden directories and common non-package directories
            if dir_name.starts_with('.') || dir_name == "build" || dir_name == "resource" {
                continue;
            }

            // Check if it's a Python package (has __init__.py)
            if path.join("__init__.py").exists() {
                let src = Utf8PathBuf::try_from(path.clone()).map_err(|e| {
                    crate::Error::build(
                        format!("Path is not valid UTF-8: {:?}", e),
                        "Ensure all file paths contain only valid UTF-8 characters",
                    )
                })?;
                let dst = build_dir.join(dir_name);

                if dst.exists() || dst.is_symlink() {
                    if dst.is_symlink() {
                        std::fs::remove_file(&dst)?;
                    } else {
                        std::fs::remove_dir_all(&dst)?;
                    }
                }
                std::os::unix::fs::symlink(&src, &dst)?;
            }
        }
    }

    Ok(())
}

/// Ensure package.xml is installed (copied or symlinked)
fn ensure_package_xml_installed(
    package: &Package,
    install_dir: &Utf8PathBuf,
    symlink: bool,
) -> Result<()> {
    let share_dir = install_dir.join("share").join(&package.name);
    std::fs::create_dir_all(&share_dir)?;

    let src = package.path.join("package.xml");
    let dst = share_dir.join("package.xml");

    if src.exists() && !dst.exists() {
        if symlink {
            std::os::unix::fs::symlink(&src, &dst)?;
        } else {
            std::fs::copy(&src, &dst)?;
        }
    }

    Ok(())
}

/// Ensure ament resource index entry exists
fn ensure_ament_resource_index(package: &Package, install_dir: &Utf8PathBuf) -> Result<()> {
    let index_dir = install_dir
        .join("share")
        .join("ament_index")
        .join("resource_index")
        .join("packages");
    std::fs::create_dir_all(&index_dir)?;

    let marker_path = index_dir.join(&package.name);
    if !marker_path.exists() {
        std::fs::write(&marker_path, "")?;
    }

    Ok(())
}

/// Generate Python package environment files (hook files and package.dsv in colcon format)
async fn generate_python_package_environment_files(
    workspace: &Workspace,
    package: &Package,
    symlink_install: bool,
) -> Result<()> {
    let install_dir = workspace.package_install_dir(&package.name);
    let share_dir = install_dir.join("share").join(&package.name);
    let hook_dir = share_dir.join("hook");

    std::fs::create_dir_all(&hook_dir)?;

    // Get Python lib directory
    let python_lib_dir = get_python_lib_dir().await?;

    // Generate hook/ament_prefix_path.dsv
    std::fs::write(
        hook_dir.join("ament_prefix_path.dsv"),
        "prepend-non-duplicate;AMENT_PREFIX_PATH;\n",
    )?;

    // Generate hook/ament_prefix_path.sh (colcon-compatible)
    std::fs::write(
        hook_dir.join("ament_prefix_path.sh"),
        r#"# generated from colcon_core/shell/template/hook_prepend_value.sh.em

_colcon_prepend_unique_value AMENT_PREFIX_PATH "$COLCON_CURRENT_PREFIX"
"#,
    )?;

    // Generate hook/pythonpath.dsv
    std::fs::write(
        hook_dir.join("pythonpath.dsv"),
        format!("prepend-non-duplicate;PYTHONPATH;{}\n", python_lib_dir),
    )?;

    // Generate hook/pythonpath.sh (colcon-compatible)
    std::fs::write(
        hook_dir.join("pythonpath.sh"),
        format!(
            r#"# generated from colcon_core/shell/template/hook_prepend_value.sh.em

_colcon_prepend_unique_value PYTHONPATH "$COLCON_CURRENT_PREFIX/{}"
"#,
            python_lib_dir
        ),
    )?;

    // For symlink install, we need pythonpath_develop hook in build directory
    // (following colcon's behavior)
    if symlink_install {
        let build_dir = workspace.package_build_dir(&package.name);
        let build_hook_dir = build_dir.join("share").join(&package.name).join("hook");
        std::fs::create_dir_all(&build_hook_dir)?;

        // Generate hook/pythonpath_develop.dsv in build directory
        std::fs::write(
            build_hook_dir.join("pythonpath_develop.dsv"),
            format!("prepend-non-duplicate;PYTHONPATH;{}\n", build_dir),
        )?;

        // Generate hook/pythonpath_develop.sh in build directory
        std::fs::write(
            build_hook_dir.join("pythonpath_develop.sh"),
            format!(
                r#"# generated from colcon_core/shell/template/hook_prepend_value.sh.em

_colcon_prepend_unique_value PYTHONPATH "{}"
"#,
                build_dir
            ),
        )?;
    }

    // Get all runtime dependencies (including system packages)
    // The colcon marker file needs all dependencies, not just workspace packages
    let run_deps: Vec<String> = package.run_dependencies().map(|s| s.to_string()).collect();
    let run_deps_refs: Vec<&str> = run_deps.iter().map(|s| s.as_str()).collect();

    // Write colcon marker file
    write_colcon_marker_file(&install_dir, &package.name, &run_deps_refs)?;

    // Build the list of hooks for package.dsv
    let mut all_hooks: Vec<String> = vec![
        format!("share/{}/hook/ament_prefix_path.dsv", package.name),
        format!("share/{}/hook/ament_prefix_path.sh", package.name),
        format!("share/{}/hook/pythonpath.dsv", package.name),
        format!("share/{}/hook/pythonpath.sh", package.name),
    ];

    // Add pythonpath_develop hooks if using symlink install
    // These hooks are in the build directory, so we use relative paths
    if symlink_install {
        // From install/<pkg>/ to build/<pkg>/share/<pkg>/hook/
        // Relative path: ../../build/<pkg>/share/<pkg>/hook/
        all_hooks.push(format!(
            "../../{}/{}/share/{}/hook/pythonpath_develop.dsv",
            workspace.config.workspace.build_dir, package.name, package.name
        ));
        all_hooks.push(format!(
            "../../{}/{}/share/{}/hook/pythonpath_develop.sh",
            workspace.config.workspace.build_dir, package.name, package.name
        ));
    }

    // Generate package.dsv
    let hook_refs: Vec<&str> = all_hooks.iter().map(|s| s.as_str()).collect();
    let package_dsv_content = generate_ament_python_package_dsv(&package.name, &hook_refs);
    std::fs::write(share_dir.join("package.dsv"), package_dsv_content)?;

    Ok(())
}
