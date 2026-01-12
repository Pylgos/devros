//! Configuration file parsing and merging
//!
//! This module handles parsing of `devros.toml` and `devros.local.toml` files,
//! and implements the merge logic as specified in specs/01_configuration.md.

use camino::{Utf8Path, Utf8PathBuf};
use serde::{Deserialize, Serialize};

use crate::Result;

/// Main configuration structure for devros
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct Config {
    /// Workspace settings
    pub workspace: WorkspaceConfig,

    /// Build settings
    pub build: BuildConfig,
}

/// Workspace configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct WorkspaceConfig {
    /// Workspace root directory (default: ".")
    pub root: Utf8PathBuf,

    /// Build artifacts directory (default: "build")
    pub build_dir: Utf8PathBuf,

    /// Installation directory (default: "install")
    pub install_dir: Utf8PathBuf,

    /// State directory for devros internal files (default: ".devros")
    pub state_dir: Utf8PathBuf,
}

impl Default for WorkspaceConfig {
    fn default() -> Self {
        Self {
            root: Utf8PathBuf::from("."),
            build_dir: Utf8PathBuf::from("build"),
            install_dir: Utf8PathBuf::from("install"),
            state_dir: Utf8PathBuf::from(".devros"),
        }
    }
}

/// Build configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct BuildConfig {
    /// Packages to skip during build
    #[serde(default)]
    pub skip_packages: Vec<String>,

    /// Number of parallel jobs (default: number of logical CPUs)
    pub jobs: Option<usize>,

    /// ament_cmake specific settings
    pub ament_cmake: AmentCmakeConfig,
}

/// ament_cmake specific build configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct AmentCmakeConfig {
    /// CMAKE_BUILD_TYPE (default: "RelWithDebInfo")
    pub build_type: String,

    /// Additional CMake arguments
    #[serde(default)]
    pub cmake_args: Vec<String>,

    /// Export compile_commands.json (default: false)
    #[serde(default)]
    pub export_compile_commands: bool,
}

impl Default for AmentCmakeConfig {
    fn default() -> Self {
        Self {
            build_type: "RelWithDebInfo".to_string(),
            cmake_args: Vec::new(),
            export_compile_commands: false,
        }
    }
}

impl Config {
    /// Load configuration from a workspace directory.
    ///
    /// This loads `devros.toml` and optionally merges `devros.local.toml` if it exists.
    pub fn load(workspace_root: &Utf8Path) -> Result<Self> {
        let config_path = workspace_root.join("devros.toml");
        let local_config_path = workspace_root.join("devros.local.toml");

        // Load base config if it exists
        let base_config = if config_path.exists() {
            let content = std::fs::read_to_string(&config_path)?;
            toml::from_str::<toml::Value>(&content)?
        } else {
            toml::Value::Table(toml::map::Map::new())
        };

        // Load local config if it exists
        let local_config = if local_config_path.exists() {
            let content = std::fs::read_to_string(&local_config_path)?;
            Some(toml::from_str::<toml::Value>(&content)?)
        } else {
            None
        };

        // Merge configs
        let merged = if let Some(local) = local_config {
            merge_toml_values(base_config, local)
        } else {
            base_config
        };

        // Deserialize merged config
        let config: Config = merged.try_into()?;

        Ok(config)
    }

    /// Load configuration from a string (for testing)
    pub fn parse(content: &str) -> Result<Self> {
        let config: Config = toml::from_str(content)?;
        Ok(config)
    }

    /// Merge two configurations (local overrides base)
    pub fn merge(base: Self, local: Self) -> Self {
        // Convert both to TOML values
        let base_value = toml::Value::try_from(base)
            .expect("Config struct should always serialize to TOML - this is a bug in devros");
        let local_value = toml::Value::try_from(local)
            .expect("Config struct should always serialize to TOML - this is a bug in devros");

        // Merge
        let merged = merge_toml_values(base_value, local_value);

        // Convert back
        merged.try_into().expect(
            "Merged TOML should always deserialize to Config - this is a bug in devros. \
             Check that all Config fields have appropriate serde defaults.",
        )
    }

    /// Get the effective number of jobs
    pub fn effective_jobs(&self) -> usize {
        self.build
            .jobs
            .unwrap_or_else(|| std::thread::available_parallelism().map_or(1, |n| n.get()))
    }
}

/// Merge two TOML values according to the merge specification:
/// - Tables: recursively merged
/// - Arrays: local replaces base (not merged)
/// - Primitives: local overrides base
fn merge_toml_values(base: toml::Value, local: toml::Value) -> toml::Value {
    match (base, local) {
        (toml::Value::Table(mut base_table), toml::Value::Table(local_table)) => {
            for (key, local_value) in local_table {
                if let Some(base_value) = base_table.remove(&key) {
                    base_table.insert(key, merge_toml_values(base_value, local_value));
                } else {
                    base_table.insert(key, local_value);
                }
            }
            toml::Value::Table(base_table)
        }
        // For arrays and primitives, local completely overrides base
        (_, local) => local,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = Config::default();

        assert_eq!(config.workspace.root, Utf8PathBuf::from("."));
        assert_eq!(config.workspace.build_dir, Utf8PathBuf::from("build"));
        assert_eq!(config.workspace.install_dir, Utf8PathBuf::from("install"));
        assert_eq!(config.workspace.state_dir, Utf8PathBuf::from(".devros"));
        assert!(config.build.skip_packages.is_empty());
        assert!(config.build.jobs.is_none());
        assert_eq!(config.build.ament_cmake.build_type, "RelWithDebInfo");
    }

    #[test]
    fn test_parse_minimal_config() {
        let content = "";
        let config = Config::parse(content).unwrap();

        assert_eq!(config.workspace.root, Utf8PathBuf::from("."));
    }

    #[test]
    fn test_parse_full_config() {
        let content = r#"
[workspace]
root = "."
build_dir = "build"
install_dir = "install"
state_dir = ".devros"

[build]
skip_packages = ["pkg_a", "pkg_b"]
jobs = 8

[build.ament_cmake]
build_type = "Release"
cmake_args = ["-DFOO=1"]
export_compile_commands = true
"#;

        let config = Config::parse(content).unwrap();

        assert_eq!(config.workspace.root, Utf8PathBuf::from("."));
        assert_eq!(config.workspace.build_dir, Utf8PathBuf::from("build"));
        assert_eq!(config.build.skip_packages, vec!["pkg_a", "pkg_b"]);
        assert_eq!(config.build.jobs, Some(8));
        assert_eq!(config.build.ament_cmake.build_type, "Release");
        assert_eq!(config.build.ament_cmake.cmake_args, vec!["-DFOO=1"]);
        assert!(config.build.ament_cmake.export_compile_commands);
    }

    #[test]
    fn test_merge_configs_via_toml_value() {
        // Test the actual merge logic used by Config::load (TOML value based)
        let base = r#"
[build]
jobs = 8
skip_packages = ["pkg_a", "pkg_b"]

[build.ament_cmake]
build_type = "Release"
cmake_args = ["-DFOO=1"]
"#;

        let local = r#"
[build]
jobs = 16

[build.ament_cmake]
cmake_args = ["-DBAR=2"]
"#;

        // Simulate what Config::load does
        let base_value: toml::Value = toml::from_str(base).unwrap();
        let local_value: toml::Value = toml::from_str(local).unwrap();
        let merged_value = merge_toml_values(base_value, local_value);
        let merged: Config = merged_value.try_into().unwrap();

        // jobs should be overridden by local
        assert_eq!(merged.build.jobs, Some(16));

        // skip_packages should be from base (local didn't define it)
        assert_eq!(merged.build.skip_packages, vec!["pkg_a", "pkg_b"]);

        // build_type should be from base (local didn't define it)
        assert_eq!(merged.build.ament_cmake.build_type, "Release");

        // cmake_args is an array, so local replaces base completely
        assert_eq!(merged.build.ament_cmake.cmake_args, vec!["-DBAR=2"]);
    }

    #[test]
    fn test_load_from_directory() {
        let temp_dir = tempfile::tempdir().unwrap();
        let workspace_root = Utf8Path::from_path(temp_dir.path()).unwrap();

        // Create devros.toml
        let config_content = r#"
[build]
jobs = 4
skip_packages = ["test_pkg"]
"#;
        std::fs::write(workspace_root.join("devros.toml"), config_content).unwrap();

        // Create devros.local.toml
        let local_content = r#"
[build]
jobs = 16
"#;
        std::fs::write(workspace_root.join("devros.local.toml"), local_content).unwrap();

        let config = Config::load(workspace_root).unwrap();

        // Local should override base
        assert_eq!(config.build.jobs, Some(16));
        // Base value should be preserved for non-overridden fields
        assert_eq!(config.build.skip_packages, vec!["test_pkg"]);
    }

    #[test]
    fn test_load_missing_config_files() {
        let temp_dir = tempfile::tempdir().unwrap();
        let workspace_root = Utf8Path::from_path(temp_dir.path()).unwrap();

        // Both files missing should return default config
        let config = Config::load(workspace_root).unwrap();

        assert_eq!(config.workspace.root, Utf8PathBuf::from("."));
        assert!(config.build.skip_packages.is_empty());
    }
}
