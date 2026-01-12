//! Materialization and Python package freezing
//!
//! This module handles converting symlink-based install directories
//! into portable, deployable artifacts.

use camino::{Utf8Path, Utf8PathBuf};
use std::collections::HashSet;
use std::process::Command;

use crate::{Error, Result};

use super::utils::{copy_dir_recursive, make_relative_path};

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

/// Materialize helper that handles merging and freezing operations
pub struct Materializer;

impl Materializer {
    /// Merge a single package's install directory into the staging directory
    pub fn merge_directory(
        source: &Utf8Path,
        target: &Utf8Path,
        install_base: &Utf8Path,
    ) -> Result<()> {
        let walker = walkdir::WalkDir::new(source).follow_links(false);

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
                Self::handle_symlink(src_path, &dst_path, target, install_base)?;
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
    pub fn freeze_python_packages(staging_dir: &Utf8Path) -> Result<()> {
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
                        Self::freeze_site_packages(&site_packages)?;
                    }
                }
            }
        }

        // Fix shebangs in bin directory
        let bin_dir = staging_dir.join("bin");
        if bin_dir.exists() {
            Self::fix_shebangs(&bin_dir)?;
        }

        Ok(())
    }

    /// Freeze .egg-link files in a site-packages directory
    fn freeze_site_packages(site_packages: &Utf8Path) -> Result<()> {
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
                Self::process_egg_link(&path, site_packages, &mut easy_install_entries)?;
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
            Self::clean_easy_install_pth(&easy_install_pth, &easy_install_entries)?;
        }

        // Compile Python files
        Self::compile_python_files(site_packages)?;

        Ok(())
    }

    /// Process a single .egg-link file
    fn process_egg_link(
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
    fn compile_python_files(directory: &Utf8Path) -> Result<()> {
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
    fn fix_shebangs(bin_dir: &Utf8Path) -> Result<()> {
        for entry in std::fs::read_dir(bin_dir)? {
            let entry = entry?;
            let path = Utf8PathBuf::try_from(entry.path()).map_err(|e| {
                Error::deploy(
                    format!("Path is not valid UTF-8: {:?}", e),
                    "Ensure all file paths contain only valid UTF-8 characters",
                )
            })?;

            if path.is_file() {
                Self::fix_shebang(&path)?;
            }
        }

        Ok(())
    }

    /// Fix shebang in a single file
    fn fix_shebang(path: &Utf8Path) -> Result<()> {
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
    pub fn generate_setup_scripts(staging_dir: &Utf8Path) -> Result<()> {
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
}
