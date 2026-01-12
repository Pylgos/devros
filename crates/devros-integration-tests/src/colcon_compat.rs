//! Colcon compatibility testing module
//!
//! This module provides utilities to compare devros build outputs with colcon
//! to ensure compatibility.

use bstr::ByteSlice;
use camino::{Utf8Path, Utf8PathBuf};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Path;
use walkdir::WalkDir;

/// Result type for comparison operations
pub type Result<T> = std::result::Result<T, ComparisonError>;

/// Error type for comparison operations
#[derive(Debug, thiserror::Error)]
pub enum ComparisonError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Walkdir error: {0}")]
    Walkdir(#[from] walkdir::Error),

    #[error("File missing in {location}: {path}")]
    FileMissing { path: String, location: String },

    #[error("Content mismatch for {path}: {details}")]
    ContentMismatch { path: String, details: String },

    #[error("Symlink mismatch for {path}: colcon={colcon_is_symlink}, devros={devros_is_symlink}")]
    SymlinkMismatch {
        path: String,
        colcon_is_symlink: bool,
        devros_is_symlink: bool,
    },

    #[error("DSV entry mismatch for {path}: {details}")]
    DsvMismatch { path: String, details: String },

    #[error("Multiple errors found: {0:?}")]
    Multiple(Vec<ComparisonError>),
}

/// Represents a file entry with metadata for comparison
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FileEntry {
    /// Relative path from install root
    pub relative_path: Utf8PathBuf,
    /// Whether this is a symlink
    pub is_symlink: bool,
    /// Whether this is a directory
    pub is_dir: bool,
    /// File size (0 for directories)
    pub size: u64,
    /// Content hash for regular files (None for dirs/symlinks)
    pub content_hash: Option<String>,
    /// Symlink target for symlinks
    pub symlink_target: Option<Utf8PathBuf>,
}

/// Represents parsed DSV file entries
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DsvEntries {
    /// Source directives (paths to source)
    pub source_entries: BTreeSet<String>,
    /// Other directives
    pub other_entries: BTreeSet<String>,
}

impl DsvEntries {
    /// Parse DSV file, filtering out .ps1 entries
    pub fn parse(path: &Path) -> Result<Self> {
        let content = fs::read_to_string(path)?;
        Self::parse_str(&content)
    }

    /// Parse DSV content from a string, filtering out .ps1 entries
    pub fn parse_str(content: &str) -> Result<Self> {
        let mut source_entries = BTreeSet::new();
        let mut other_entries = BTreeSet::new();

        for line in content.lines() {
            let line = line.trim();

            // Skip empty lines and comments
            if line.is_empty() || line.starts_with('#') {
                continue;
            }

            // Filter out .ps1 entries (Windows PowerShell)
            if line.ends_with(".ps1") {
                continue;
            }

            // Filter out pythonpath_develop entries (symlink-install specific)
            if line.contains("pythonpath_develop") {
                continue;
            }

            // Filter out cmake_prefix_path entries (colcon-specific)
            if line.contains("cmake_prefix_path") {
                continue;
            }

            // Parse entry
            if line.starts_with("source;") {
                source_entries.insert(line.to_string());
            } else {
                other_entries.insert(line.to_string());
            }
        }

        Ok(Self {
            source_entries,
            other_entries,
        })
    }

    /// Compare with another DSV entries set
    pub fn compare(&self, other: &DsvEntries) -> Vec<String> {
        let mut diffs = Vec::new();

        // Compare source entries
        for entry in &self.source_entries {
            if !other.source_entries.contains(entry) {
                diffs.push(format!("Missing source entry: {}", entry));
            }
        }
        for entry in &other.source_entries {
            if !self.source_entries.contains(entry) {
                diffs.push(format!("Extra source entry: {}", entry));
            }
        }

        // Compare other entries
        for entry in &self.other_entries {
            if !other.other_entries.contains(entry) {
                diffs.push(format!("Missing entry: {}", entry));
            }
        }
        for entry in &other.other_entries {
            if !self.other_entries.contains(entry) {
                diffs.push(format!("Extra entry: {}", entry));
            }
        }

        diffs
    }
}

/// Scan a directory and collect file entries
pub fn scan_directory(root: &Utf8Path) -> Result<BTreeMap<Utf8PathBuf, FileEntry>> {
    let mut entries = BTreeMap::new();

    for entry in WalkDir::new(root).follow_links(false) {
        let entry = entry?;
        let path = entry.path();

        // Get relative path
        let relative = path.strip_prefix(root).unwrap_or(path);
        let relative_utf8 = Utf8PathBuf::from_path_buf(relative.to_path_buf()).map_err(|p| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("Invalid UTF-8 path: {:?}", p),
            )
        })?;

        // Skip the root directory itself
        if relative_utf8.as_str().is_empty() {
            continue;
        }

        let metadata = entry.metadata()?;
        let is_symlink = entry.path_is_symlink();
        let is_dir = metadata.is_dir();

        let (content_hash, symlink_target) = if is_symlink {
            let target = fs::read_link(path)?;
            let target_utf8 = Utf8PathBuf::from_path_buf(target).map_err(|p| {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    format!("Invalid UTF-8 symlink target: {:?}", p),
                )
            })?;
            (None, Some(target_utf8))
        } else if is_dir {
            (None, None)
        } else {
            // Calculate hash for regular files
            let content = fs::read(path)?;
            let hash = blake3::hash(&content);
            (Some(hash.to_hex().to_string()), None)
        };

        entries.insert(
            relative_utf8.clone(),
            FileEntry {
                relative_path: relative_utf8,
                is_symlink,
                is_dir,
                size: metadata.len(),
                content_hash,
                symlink_target,
            },
        );
    }

    Ok(entries)
}

/// Files/directories to exclude from comparison
fn should_exclude(path: &str) -> bool {
    // Exclude colcon-specific files at root level
    let colcon_only_files = [
        "_local_setup_util_ps1.py",
        "local_setup.bash",
        "local_setup.zsh",
        "setup.zsh",
        "setup.bash", // devros generates its own setup.bash
    ];
    for f in &colcon_only_files {
        if path == *f {
            return true;
        }
    }

    // Exclude .ps1 files
    if path.ends_with(".ps1") {
        return true;
    }

    // Exclude package.bash, package.sh, package.zsh (colcon generates these, devros doesn't need them)
    if path.ends_with("/package.bash")
        || path.ends_with("/package.sh")
        || path.ends_with("/package.zsh")
    {
        return true;
    }

    // Exclude pythonpath_develop.sh (colcon generates this, devros doesn't use it)
    // Note that we still have to compare pythonpath_develop.dsv files
    if path.ends_with("/pythonpath_develop.sh") {
        return true;
    }

    // These files may differ in content but are acceptable
    let acceptable_difference_in_file = ["local_setup.sh", "_local_setup_util_sh.py", "setup.sh"];
    for f in &acceptable_difference_in_file {
        if path == *f {
            return true;
        }
    }

    false
}

/// Compare two install directories
pub fn compare_install_dirs(
    colcon_dir: &Utf8Path,
    devros_dir: &Utf8Path,
) -> Result<Vec<ComparisonError>> {
    let mut errors = Vec::new();

    let colcon_entries = scan_directory(colcon_dir)?;
    let devros_entries = scan_directory(devros_dir)?;

    // Collect all paths
    let mut all_paths: BTreeSet<&Utf8PathBuf> = BTreeSet::new();
    for path in colcon_entries.keys() {
        if !should_exclude(path.as_str()) {
            all_paths.insert(path);
        }
    }
    for path in devros_entries.keys() {
        if !should_exclude(path.as_str()) {
            all_paths.insert(path);
        }
    }

    for path in all_paths {
        let colcon_entry = colcon_entries.get(path);
        let devros_entry = devros_entries.get(path);

        match (colcon_entry, devros_entry) {
            (Some(_), None) => {
                errors.push(ComparisonError::FileMissing {
                    path: path.to_string(),
                    location: "devros".to_string(),
                });
            }
            (None, Some(_)) => {
                errors.push(ComparisonError::FileMissing {
                    path: path.to_string(),
                    location: "colcon".to_string(),
                });
            }
            (Some(colcon), Some(devros)) => {
                // For DSV files, do special comparison (resolve symlinks and compare content)
                if path.as_str().ends_with(".dsv") {
                    let colcon_path = colcon_dir.join(path);
                    let devros_path = devros_dir.join(path);

                    // Read actual content (following symlinks automatically)
                    let colcon_content = fs::read_to_string(&colcon_path);
                    let devros_content = fs::read_to_string(&devros_path);

                    match (colcon_content, devros_content) {
                        (Ok(colcon_str), Ok(devros_str)) => {
                            // Parse and compare DSV entries
                            match (
                                DsvEntries::parse_str(&colcon_str),
                                DsvEntries::parse_str(&devros_str),
                            ) {
                                (Ok(colcon_dsv), Ok(devros_dsv)) => {
                                    let diffs = colcon_dsv.compare(&devros_dsv);
                                    if !diffs.is_empty() {
                                        errors.push(ComparisonError::DsvMismatch {
                                            path: path.to_string(),
                                            details: diffs.join("; "),
                                        });
                                    }
                                }
                                (Err(e), _) | (_, Err(e)) => {
                                    errors.push(ComparisonError::ContentMismatch {
                                        path: path.to_string(),
                                        details: format!("Failed to parse DSV content: {}", e),
                                    });
                                }
                            }
                        }
                        (Err(e), _) => {
                            errors.push(ComparisonError::ContentMismatch {
                                path: path.to_string(),
                                details: format!("Failed to read colcon DSV: {}", e),
                            });
                        }
                        (_, Err(e)) => {
                            errors.push(ComparisonError::ContentMismatch {
                                path: path.to_string(),
                                details: format!("Failed to read devros DSV: {}", e),
                            });
                        }
                    }
                    continue;
                }

                // For files where symlink status differs, compare actual content
                if colcon.is_symlink != devros.is_symlink {
                    if !colcon.is_dir && !devros.is_dir {
                        let colcon_path = colcon_dir.join(path);
                        let devros_path = devros_dir.join(path);

                        // Read actual content (following symlinks)
                        let colcon_content = fs::read(&colcon_path);
                        let devros_content = fs::read(&devros_path);

                        match (colcon_content, devros_content) {
                            (Ok(c), Ok(d)) if c == d => {
                                // Content matches, symlink status difference is acceptable
                            }
                            (Ok(_), Ok(_)) => {
                                errors.push(ComparisonError::ContentMismatch {
                                    path: path.to_string(),
                                    details: format!(
                                        "Content differs (symlink status also differs: colcon={}, devros={})",
                                        colcon.is_symlink, devros.is_symlink
                                    ),
                                });
                            }
                            (Err(e), _) | (_, Err(e)) => {
                                errors.push(ComparisonError::ContentMismatch {
                                    path: path.to_string(),
                                    details: format!("Failed to read file: {}", e),
                                });
                            }
                        }
                    } else {
                        errors.push(ComparisonError::SymlinkMismatch {
                            path: path.to_string(),
                            colcon_is_symlink: colcon.is_symlink,
                            devros_is_symlink: devros.is_symlink,
                        });
                    }
                    continue;
                }

                // For regular files (both are regular or both are symlinks), compare content
                if !colcon.is_dir {
                    let colcon_path = colcon_dir.join(path);
                    let devros_path = devros_dir.join(path);

                    // Read actual content (following symlinks)
                    let colcon_content = fs::read(&colcon_path);
                    let devros_content = fs::read(&devros_path);

                    match (colcon_content, devros_content) {
                        (Ok(c), Ok(d)) => {
                            let c_bstr = bstr::BStr::new(&c);
                            let d_bstr = bstr::BStr::new(&d);

                            let colcon_content_normalized = c_bstr
                                .replace(colcon_dir.as_str(), "{INSTALL_DIR}")
                                .replace("colcon_build", "build");
                            let devros_content_normalized =
                                d_bstr.replace(devros_dir.as_str(), "{INSTALL_DIR}");

                            if colcon_content_normalized != devros_content_normalized {
                                let c_str = str::from_utf8(&c);
                                let d_str = str::from_utf8(&d);

                                let details = match (c_str, d_str) {
                                    (Ok(c_utf8), Ok(d_utf8)) => {
                                        let changes = similar::TextDiff::from_lines(c_utf8, d_utf8)
                                            .unified_diff()
                                            .context_radius(3)
                                            .to_string();
                                        format!("File content differs:\n{}", changes)
                                    }
                                    _ => "File content differs (binary data)".to_string(),
                                };

                                errors.push(ComparisonError::ContentMismatch {
                                    path: path.to_string(),
                                    details,
                                });
                            }
                        }
                        (Err(e), _) | (_, Err(e)) => {
                            errors.push(ComparisonError::ContentMismatch {
                                path: path.to_string(),
                                details: format!("Failed to read file: {}", e),
                            });
                        }
                    }
                }
            }
            (None, None) => unreachable!(),
        }
    }

    Ok(errors)
}

/// Result of a compatibility test
#[derive(Debug)]
pub struct CompatibilityTestResult {
    /// Errors found during comparison
    pub errors: Vec<ComparisonError>,
    /// Number of files compared
    pub files_compared: usize,
    /// Files only in colcon
    pub colcon_only: Vec<String>,
    /// Files only in devros  
    pub devros_only: Vec<String>,
}

impl CompatibilityTestResult {
    /// Check if the test passed (no errors)
    pub fn is_success(&self) -> bool {
        self.errors.is_empty()
    }

    /// Generate a summary report
    pub fn summary(&self) -> String {
        let mut report = String::new();

        report.push_str(&format!("Files compared: {}\n", self.files_compared));

        if !self.colcon_only.is_empty() {
            report.push_str(&format!(
                "\nFiles only in colcon ({}):\n",
                self.colcon_only.len()
            ));
            for f in &self.colcon_only {
                report.push_str(&format!("  - {}\n", f));
            }
        }

        if !self.devros_only.is_empty() {
            report.push_str(&format!(
                "\nFiles only in devros ({}):\n",
                self.devros_only.len()
            ));
            for f in &self.devros_only {
                report.push_str(&format!("  - {}\n", f));
            }
        }

        if !self.errors.is_empty() {
            report.push_str(&format!("\nErrors ({}):\n", self.errors.len()));
            for e in &self.errors {
                report.push_str(&format!("  - {}\n", e));
            }
        }

        if self.is_success() {
            report.push_str("\n✓ All compatibility checks passed\n");
        } else {
            report.push_str("\n✗ Compatibility checks failed\n");
        }

        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dsv_entries_parse() {
        let content = r#"source;share/pkg/hook/test.dsv
source;share/pkg/hook/test.sh
source;share/pkg/hook/test.ps1
prepend-non-duplicate;PATH;bin
"#;

        let entries = DsvEntries::parse_str(content).unwrap();

        // .ps1 should be filtered out
        assert!(
            entries
                .source_entries
                .contains("source;share/pkg/hook/test.dsv")
        );
        assert!(
            entries
                .source_entries
                .contains("source;share/pkg/hook/test.sh")
        );
        assert!(!entries.source_entries.iter().any(|e| e.ends_with(".ps1")));

        assert!(
            entries
                .other_entries
                .contains("prepend-non-duplicate;PATH;bin")
        );
    }

    #[test]
    fn test_dsv_entries_compare() {
        let dsv1 = DsvEntries {
            source_entries: ["source;a.dsv", "source;b.sh"]
                .iter()
                .map(|s| s.to_string())
                .collect(),
            other_entries: ["set;VAR;value"].iter().map(|s| s.to_string()).collect(),
        };

        let dsv2 = DsvEntries {
            source_entries: ["source;a.dsv", "source;c.sh"]
                .iter()
                .map(|s| s.to_string())
                .collect(),
            other_entries: ["set;VAR;value"].iter().map(|s| s.to_string()).collect(),
        };

        let diffs = dsv1.compare(&dsv2);
        assert!(!diffs.is_empty());
        assert!(diffs.iter().any(|d| d.contains("b.sh")));
        assert!(diffs.iter().any(|d| d.contains("c.sh")));
    }

    #[test]
    fn test_should_exclude() {
        assert!(should_exclude("share/pkg/hook/test.ps1"));
        assert!(should_exclude("share/pkg/hook/pythonpath_develop.sh"));
        assert!(should_exclude("share/pkg/package.bash"));
        assert!(should_exclude("share/pkg/package.sh"));
        assert!(should_exclude("setup.bash"));

        // DSV files are not excluded
        assert!(!should_exclude("share/pkg/package.dsv"));
        assert!(!should_exclude("share/example_py/hook/test.dsv"));
    }
}

/// Compare environment variables between colcon and devros setup scripts
///
/// This function sources the setup.bash from both install directories and compares
/// the resulting environment variables. Only differences not related to install
/// directory paths are considered errors.
pub fn compare_environment_variables(
    colcon_install: &Utf8Path,
    devros_install: &Utf8Path,
    base_env: &std::collections::HashMap<String, String>,
    devros_binary: &Path,
) -> Result<Vec<EnvDifference>> {
    let colcon_env = get_environment(&format!("source {}/setup.bash", colcon_install), base_env)?;
    let devros_env = get_environment(
        &format!("eval \"$({} env shell)\"", devros_binary.display()),
        base_env,
    )?;

    let mut differences = Vec::new();

    // Get all keys
    let mut all_keys: BTreeSet<&str> = BTreeSet::new();
    for key in colcon_env.keys() {
        all_keys.insert(key);
    }
    for key in devros_env.keys() {
        all_keys.insert(key);
    }

    // Compare each key
    for key in all_keys {
        let colcon_val = colcon_env.get(key);
        let devros_val = devros_env.get(key);

        match (colcon_val, devros_val) {
            (Some(c), Some(d)) => {
                // Normalize paths - replace install directory paths
                let c_normalized =
                    normalize_paths(c, colcon_install.as_str(), devros_install.as_str());
                let d_normalized =
                    normalize_paths(d, devros_install.as_str(), colcon_install.as_str());

                if c_normalized != d_normalized {
                    // Check if the difference is only in build directory paths
                    let c_norm2 = c_normalized.replace("colcon_build", "build");
                    if c_norm2 != d_normalized {
                        differences.push(EnvDifference::ValueDiffers {
                            key: key.to_string(),
                            colcon_value: c.clone(),
                            devros_value: d.clone(),
                        });
                    }
                }
            }
            (Some(c), None) => {
                // Skip COLCON_PREFIX_PATH - this is expected to differ
                if !should_skip_env_key(key) {
                    differences.push(EnvDifference::OnlyInColcon {
                        key: key.to_string(),
                        value: c.clone(),
                    });
                }
            }
            (None, Some(d)) => {
                if !should_skip_env_key(key) {
                    differences.push(EnvDifference::OnlyInDevros {
                        key: key.to_string(),
                        value: d.clone(),
                    });
                }
            }
            (None, None) => unreachable!(),
        }
    }

    Ok(differences)
}

/// Difference between colcon and devros environment
#[derive(Debug, Clone)]
pub enum EnvDifference {
    /// Key only present in colcon environment
    OnlyInColcon { key: String, value: String },
    /// Key only present in devros environment
    OnlyInDevros { key: String, value: String },
    /// Value differs between colcon and devros
    ValueDiffers {
        key: String,
        colcon_value: String,
        devros_value: String,
    },
}

impl std::fmt::Display for EnvDifference {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EnvDifference::OnlyInColcon { key, value } => {
                write!(f, "Only in colcon: {}={}", key, value)
            }
            EnvDifference::OnlyInDevros { key, value } => {
                write!(f, "Only in devros: {}={}", key, value)
            }
            EnvDifference::ValueDiffers {
                key,
                colcon_value,
                devros_value,
            } => {
                write!(
                    f,
                    "Value differs for {}: colcon='{}', devros='{}'",
                    key, colcon_value, devros_value
                )
            }
        }
    }
}

/// Get environment after sourcing setup.bash
fn get_environment(
    command: &str,
    base_env: &std::collections::HashMap<String, String>,
) -> Result<std::collections::HashMap<String, String>> {
    use std::process::Command;

    let cmd = format!("{} && env", command);

    let output = Command::new("bash")
        .args(["-c", &cmd])
        .envs(base_env)
        .output()?;

    if !output.status.success() {
        return Err(ComparisonError::Io(std::io::Error::other(format!(
            "Failed to source setup.bash: {}",
            String::from_utf8_lossy(&output.stderr)
        ))));
    }

    let env_str = String::from_utf8_lossy(&output.stdout);
    let mut env = std::collections::HashMap::new();

    for line in env_str.lines() {
        if let Some((key, value)) = line.split_once('=') {
            env.insert(key.to_string(), value.to_string());
        }
    }

    Ok(env)
}

/// Normalize paths in a value by replacing install directory paths
fn normalize_paths(value: &str, source_path: &str, _target_path: &str) -> String {
    value.replace(source_path, "{INSTALL_DIR}")
}

/// Check if an environment key should be skipped in comparison
fn should_skip_env_key(key: &str) -> bool {
    // Skip shell-specific variables
    let skip_keys = [
        "_",
        "BASH_FUNC_",
        "SHLVL",
        "OLDPWD",
        "PWD",
        "COLUMNS",
        "LINES",
        "SHLVL",
    ];

    for skip in &skip_keys {
        if key.starts_with(skip) {
            return true;
        }
    }

    false
}
