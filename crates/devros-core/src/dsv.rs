//! DSV (Dynamic Source Value) file parsing and environment variable management
//!
//! This module handles parsing of .dsv files and computing environment variables
//! according to the specification in specs/05_environment_management.md.

use camino::{Utf8Path, Utf8PathBuf};
use std::collections::{HashMap, HashSet};

use crate::{Error, Result};

/// Operations supported in DSV files
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DsvOperation {
    /// Prepend value to environment variable (with deduplication)
    PrependNonDuplicate {
        variable: String,
        values: Vec<String>,
    },
    /// Prepend value if path exists (with deduplication)
    PrependNonDuplicateIfExists { variable: String, value: String },
    /// Append value to environment variable (with deduplication)
    AppendNonDuplicate { variable: String, value: String },
    /// Set environment variable to value
    Set { variable: String, value: String },
    /// Set environment variable only if not already set
    SetIfUnset { variable: String, value: String },
    /// Source another file
    Source { path: String },
}

/// Parsed DSV file
#[derive(Debug, Clone)]
pub struct DsvFile {
    /// Path to the DSV file
    pub path: Utf8PathBuf,
    /// Operations defined in the file
    pub operations: Vec<DsvOperation>,
}

impl DsvFile {
    /// Parse a DSV file from disk
    pub fn parse(path: &Utf8Path) -> Result<Self> {
        let content = std::fs::read_to_string(path)?;
        Self::parse_str(&content, path.to_path_buf())
    }

    /// Parse DSV content from a string
    pub fn parse_str(content: &str, path: Utf8PathBuf) -> Result<Self> {
        let mut operations = Vec::new();

        for (line_num, line) in content.lines().enumerate() {
            let line = line.trim();

            // Skip empty lines and comments
            if line.is_empty() || line.starts_with('#') {
                continue;
            }

            let parts: Vec<&str> = line.split(';').collect();
            if parts.is_empty() {
                continue;
            }

            let op_type = parts[0];
            match op_type {
                "prepend-non-duplicate" => {
                    if parts.len() < 2 {
                        return Err(Error::dsv_parse(
                            format!(
                                "Invalid prepend-non-duplicate at line {}: missing variable name",
                                line_num + 1
                            ),
                            "Format: prepend-non-duplicate;VARIABLE_NAME;value1;value2;...",
                        ));
                    }
                    let variable = parts[1].to_string();
                    let values: Vec<String> =
                        parts.iter().skip(2).map(|s| (*s).to_string()).collect();
                    operations.push(DsvOperation::PrependNonDuplicate { variable, values });
                }
                "prepend-non-duplicate-if-exists" => {
                    if parts.len() < 3 {
                        return Err(Error::dsv_parse(
                            format!(
                                "Invalid prepend-non-duplicate-if-exists at line {}",
                                line_num + 1
                            ),
                            "Format: prepend-non-duplicate-if-exists;VARIABLE_NAME;value",
                        ));
                    }
                    operations.push(DsvOperation::PrependNonDuplicateIfExists {
                        variable: parts[1].to_string(),
                        value: parts[2].to_string(),
                    });
                }
                "append-non-duplicate" => {
                    if parts.len() < 3 {
                        return Err(Error::dsv_parse(
                            format!("Invalid append-non-duplicate at line {}", line_num + 1),
                            "Format: append-non-duplicate;VARIABLE_NAME;value",
                        ));
                    }
                    operations.push(DsvOperation::AppendNonDuplicate {
                        variable: parts[1].to_string(),
                        value: parts[2].to_string(),
                    });
                }
                "set" => {
                    if parts.len() < 3 {
                        return Err(Error::dsv_parse(
                            format!("Invalid set at line {}", line_num + 1),
                            "Format: set;VARIABLE_NAME;value",
                        ));
                    }
                    operations.push(DsvOperation::Set {
                        variable: parts[1].to_string(),
                        value: parts[2].to_string(),
                    });
                }
                "set-if-unset" => {
                    if parts.len() < 3 {
                        return Err(Error::dsv_parse(
                            format!("Invalid set-if-unset at line {}", line_num + 1),
                            "Format: set-if-unset;VARIABLE_NAME;value",
                        ));
                    }
                    operations.push(DsvOperation::SetIfUnset {
                        variable: parts[1].to_string(),
                        value: parts[2].to_string(),
                    });
                }
                "source" => {
                    if parts.len() < 2 {
                        return Err(Error::dsv_parse(
                            format!("Invalid source at line {}", line_num + 1),
                            "Format: source;path/to/script",
                        ));
                    }
                    operations.push(DsvOperation::Source {
                        path: parts[1].to_string(),
                    });
                }
                _ => {
                    tracing::warn!(
                        line = line_num + 1,
                        operation = op_type,
                        "Unknown DSV operation"
                    );
                }
            }
        }

        Ok(DsvFile { path, operations })
    }
}

/// Environment variable calculator
#[derive(Debug, Default)]
pub struct EnvCalculator {
    /// Current environment state
    env: HashMap<String, String>,
    /// Processed source files (to avoid infinite recursion)
    processed_sources: HashSet<Utf8PathBuf>,
}

impl EnvCalculator {
    /// Create a new environment calculator with initial environment
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a new environment calculator from current process environment
    pub fn from_current_env() -> Self {
        let env = std::env::vars().collect();
        Self {
            env,
            processed_sources: HashSet::new(),
        }
    }

    /// Set initial environment from a HashMap
    pub fn with_env(env: HashMap<String, String>) -> Self {
        Self {
            env,
            processed_sources: HashSet::new(),
        }
    }

    /// Get the computed environment
    pub fn env(&self) -> &HashMap<String, String> {
        &self.env
    }

    /// Get a specific environment variable
    pub fn get(&self, key: &str) -> Option<&str> {
        self.env.get(key).map(|s| s.as_str())
    }

    /// Apply a DSV file's operations
    pub fn apply_dsv(&mut self, dsv: &DsvFile, prefix: &Utf8Path) -> Result<()> {
        for op in &dsv.operations {
            self.apply_operation(op, prefix)?;
        }
        Ok(())
    }

    /// Apply a single DSV operation
    pub fn apply_operation(&mut self, op: &DsvOperation, prefix: &Utf8Path) -> Result<()> {
        match op {
            DsvOperation::PrependNonDuplicate { variable, values } => {
                for value in values.iter().rev() {
                    let resolved = self.resolve_value(value, prefix);
                    self.prepend_non_duplicate(variable, &resolved);
                }
            }
            DsvOperation::PrependNonDuplicateIfExists { variable, value } => {
                let resolved = self.resolve_value(value, prefix);
                if Utf8Path::new(&resolved).exists() {
                    self.prepend_non_duplicate(variable, &resolved);
                }
            }
            DsvOperation::AppendNonDuplicate { variable, value } => {
                let resolved = self.resolve_value(value, prefix);
                self.append_non_duplicate(variable, &resolved);
            }
            DsvOperation::Set { variable, value } => {
                let resolved = self.resolve_value_for_set(value, prefix);
                self.env.insert(variable.clone(), resolved);
            }
            DsvOperation::SetIfUnset { variable, value } => {
                if !self.env.contains_key(variable) {
                    let resolved = self.resolve_value_for_set(value, prefix);
                    self.env.insert(variable.clone(), resolved);
                }
            }
            DsvOperation::Source { path } => {
                self.process_source(path, prefix)?;
            }
        }
        Ok(())
    }

    /// Resolve a value according to DSV rules
    fn resolve_value(&self, value: &str, prefix: &Utf8Path) -> String {
        if value.is_empty() {
            prefix.to_string()
        } else if !value.starts_with('/') {
            // Relative path
            prefix.join(value).to_string()
        } else {
            value.to_string()
        }
    }

    /// Resolve value for set operation
    fn resolve_value_for_set(&self, value: &str, prefix: &Utf8Path) -> String {
        if value.is_empty() {
            return prefix.to_string();
        }

        if !value.starts_with('/') {
            // For relative paths, check if prefix/value exists
            let full_path = prefix.join(value);
            if full_path.exists() {
                return full_path.to_string();
            }
        }

        value.to_string()
    }

    /// Prepend a value to an environment variable (with deduplication)
    fn prepend_non_duplicate(&mut self, variable: &str, value: &str) {
        let current = self.env.get(variable).cloned().unwrap_or_default();
        let parts: Vec<&str> = current.split(':').filter(|s| !s.is_empty()).collect();

        // Check if value already exists
        if parts.contains(&value) {
            return;
        }

        // Prepend
        let new_value = if current.is_empty() {
            value.to_string()
        } else {
            format!("{}:{}", value, current)
        };

        self.env.insert(variable.to_string(), new_value);
    }

    /// Append a value to an environment variable (with deduplication)
    fn append_non_duplicate(&mut self, variable: &str, value: &str) {
        let current = self.env.get(variable).cloned().unwrap_or_default();
        let parts: Vec<&str> = current.split(':').filter(|s| !s.is_empty()).collect();

        // Check if value already exists
        if parts.contains(&value) {
            return;
        }

        // Append
        let new_value = if current.is_empty() {
            value.to_string()
        } else {
            format!("{}:{}", current, value)
        };

        self.env.insert(variable.to_string(), new_value);
    }

    /// Process a source directive
    fn process_source(&mut self, path: &str, prefix: &Utf8Path) -> Result<()> {
        // Try to find the DSV file
        let base_path = if path.starts_with('/') {
            Utf8PathBuf::from(path)
        } else {
            prefix.join(path)
        };

        // Try with .dsv extension first
        let dsv_path = if base_path.extension().is_some() {
            base_path.clone()
        } else {
            base_path.with_extension("dsv")
        };

        // Check for circular references
        if self.processed_sources.contains(&dsv_path) {
            return Ok(());
        }

        if dsv_path.exists() {
            self.processed_sources.insert(dsv_path.clone());
            let dsv = DsvFile::parse(&dsv_path)?;
            let dsv_prefix = dsv_path.parent().unwrap_or(prefix);
            self.apply_dsv(&dsv, dsv_prefix)?;
        }

        Ok(())
    }

    /// Generate shell commands to set the environment
    pub fn generate_shell_commands(&self, shell: ShellType) -> String {
        let mut commands = Vec::new();

        for (key, value) in &self.env {
            match shell {
                ShellType::Bash | ShellType::Zsh => {
                    commands.push(format!("export {}={}", key, shell_escape(value)));
                }
                ShellType::Fish => {
                    commands.push(format!("set -gx {} {}", key, shell_escape(value)));
                }
            }
        }

        commands.join("\n")
    }
}

/// Shell type for command generation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShellType {
    Bash,
    Zsh,
    Fish,
}

impl std::str::FromStr for ShellType {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self> {
        match s.to_lowercase().as_str() {
            "bash" => Ok(ShellType::Bash),
            "zsh" => Ok(ShellType::Zsh),
            "fish" => Ok(ShellType::Fish),
            _ => Err(Error::config(
                format!("Unknown shell type: {}", s),
                "Supported shells: bash, zsh, fish",
            )),
        }
    }
}

/// Escape a string for shell use
fn shell_escape(s: &str) -> String {
    // Use single quotes and escape any single quotes in the string
    format!("'{}'", s.replace('\'', "'\\''"))
}

/// Generate package.dsv content for a package
pub fn generate_package_dsv(_package_name: &str, _prefix: &Utf8Path) -> String {
    let lines = vec![
        // AMENT_PREFIX_PATH
        "prepend-non-duplicate;AMENT_PREFIX_PATH;".to_string(),
        // CMAKE_PREFIX_PATH (same as AMENT_PREFIX_PATH for CMake packages)
        "prepend-non-duplicate;CMAKE_PREFIX_PATH;".to_string(),
        // PATH for bin directory
        "prepend-non-duplicate-if-exists;PATH;bin".to_string(),
        // LD_LIBRARY_PATH
        "prepend-non-duplicate-if-exists;LD_LIBRARY_PATH;lib".to_string(),
        // PYTHONPATH (common patterns)
        "prepend-non-duplicate-if-exists;PYTHONPATH;lib/python3/dist-packages".to_string(),
        "prepend-non-duplicate-if-exists;PYTHONPATH;lib/python3.10/site-packages".to_string(),
        "prepend-non-duplicate-if-exists;PYTHONPATH;lib/python3.11/site-packages".to_string(),
        "prepend-non-duplicate-if-exists;PYTHONPATH;lib/python3.12/site-packages".to_string(),
        // AMENT_CURRENT_PREFIX
        "set;AMENT_CURRENT_PREFIX;".to_string(),
    ];

    lines.join("\n")
}

/// Generate local_setup.sh content
pub fn generate_local_setup_sh(package_name: &str) -> String {
    format!(
        r#"#!/bin/bash
# This file was generated by devros
# Source this file to set up the environment for this package

eval "$(devros env shell --package {} --shell bash)"
"#,
        package_name
    )
}

/// Generate workspace setup.bash content
pub fn generate_workspace_setup_bash() -> String {
    r#"#!/bin/bash
# This file was generated by devros
# Source this file to set up the environment for the entire workspace

eval "$(devros env shell --shell bash)"
"#
    .to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_dsv_operations() {
        let content = r#"
# This is a comment
prepend-non-duplicate;PATH;bin
prepend-non-duplicate-if-exists;LD_LIBRARY_PATH;lib
append-non-duplicate;PYTHONPATH;site-packages
set;MY_VAR;value
set-if-unset;DEFAULT_VAR;default
source;other_package.dsv
"#;

        let dsv = DsvFile::parse_str(content, Utf8PathBuf::from("/test")).unwrap();

        assert_eq!(dsv.operations.len(), 6);

        assert!(matches!(
            &dsv.operations[0],
            DsvOperation::PrependNonDuplicate { variable, values }
            if variable == "PATH" && values == &["bin"]
        ));

        assert!(matches!(
            &dsv.operations[1],
            DsvOperation::PrependNonDuplicateIfExists { variable, value }
            if variable == "LD_LIBRARY_PATH" && value == "lib"
        ));

        assert!(matches!(
            &dsv.operations[2],
            DsvOperation::AppendNonDuplicate { variable, value }
            if variable == "PYTHONPATH" && value == "site-packages"
        ));

        assert!(matches!(
            &dsv.operations[3],
            DsvOperation::Set { variable, value }
            if variable == "MY_VAR" && value == "value"
        ));

        assert!(matches!(
            &dsv.operations[4],
            DsvOperation::SetIfUnset { variable, value }
            if variable == "DEFAULT_VAR" && value == "default"
        ));

        assert!(matches!(
            &dsv.operations[5],
            DsvOperation::Source { path }
            if path == "other_package.dsv"
        ));
    }

    #[test]
    fn test_prepend_non_duplicate() {
        let mut calc = EnvCalculator::new();
        let prefix = Utf8Path::new("/install/pkg");

        calc.apply_operation(
            &DsvOperation::PrependNonDuplicate {
                variable: "PATH".to_string(),
                values: vec!["bin".to_string()],
            },
            prefix,
        )
        .unwrap();

        assert_eq!(calc.get("PATH"), Some("/install/pkg/bin"));

        // Add another value
        calc.apply_operation(
            &DsvOperation::PrependNonDuplicate {
                variable: "PATH".to_string(),
                values: vec!["sbin".to_string()],
            },
            prefix,
        )
        .unwrap();

        assert_eq!(calc.get("PATH"), Some("/install/pkg/sbin:/install/pkg/bin"));

        // Try to add duplicate - should not change
        calc.apply_operation(
            &DsvOperation::PrependNonDuplicate {
                variable: "PATH".to_string(),
                values: vec!["bin".to_string()],
            },
            prefix,
        )
        .unwrap();

        assert_eq!(calc.get("PATH"), Some("/install/pkg/sbin:/install/pkg/bin"));
    }

    #[test]
    fn test_append_non_duplicate() {
        let mut calc = EnvCalculator::new();
        let prefix = Utf8Path::new("/install/pkg");

        calc.apply_operation(
            &DsvOperation::AppendNonDuplicate {
                variable: "PYTHONPATH".to_string(),
                value: "lib".to_string(),
            },
            prefix,
        )
        .unwrap();

        assert_eq!(calc.get("PYTHONPATH"), Some("/install/pkg/lib"));

        calc.apply_operation(
            &DsvOperation::AppendNonDuplicate {
                variable: "PYTHONPATH".to_string(),
                value: "share".to_string(),
            },
            prefix,
        )
        .unwrap();

        assert_eq!(
            calc.get("PYTHONPATH"),
            Some("/install/pkg/lib:/install/pkg/share")
        );
    }

    #[test]
    fn test_set_and_set_if_unset() {
        let mut calc = EnvCalculator::new();
        let prefix = Utf8Path::new("/install/pkg");

        // set should always set
        calc.apply_operation(
            &DsvOperation::Set {
                variable: "MY_VAR".to_string(),
                value: "first".to_string(),
            },
            prefix,
        )
        .unwrap();
        assert_eq!(calc.get("MY_VAR"), Some("first"));

        calc.apply_operation(
            &DsvOperation::Set {
                variable: "MY_VAR".to_string(),
                value: "second".to_string(),
            },
            prefix,
        )
        .unwrap();
        assert_eq!(calc.get("MY_VAR"), Some("second"));

        // set-if-unset should not override
        calc.apply_operation(
            &DsvOperation::SetIfUnset {
                variable: "MY_VAR".to_string(),
                value: "third".to_string(),
            },
            prefix,
        )
        .unwrap();
        assert_eq!(calc.get("MY_VAR"), Some("second"));

        // But should set if not present
        calc.apply_operation(
            &DsvOperation::SetIfUnset {
                variable: "NEW_VAR".to_string(),
                value: "new_value".to_string(),
            },
            prefix,
        )
        .unwrap();
        assert_eq!(calc.get("NEW_VAR"), Some("new_value"));
    }

    #[test]
    fn test_empty_value_uses_prefix() {
        let mut calc = EnvCalculator::new();
        let prefix = Utf8Path::new("/install/pkg_a");

        calc.apply_operation(
            &DsvOperation::PrependNonDuplicate {
                variable: "AMENT_PREFIX_PATH".to_string(),
                values: vec!["".to_string()],
            },
            prefix,
        )
        .unwrap();

        assert_eq!(calc.get("AMENT_PREFIX_PATH"), Some("/install/pkg_a"));
    }

    #[test]
    fn test_shell_command_generation() {
        let mut calc = EnvCalculator::new();
        calc.env.insert("PATH".to_string(), "/usr/bin".to_string());
        calc.env.insert("MY_VAR".to_string(), "value".to_string());

        let bash_cmds = calc.generate_shell_commands(ShellType::Bash);
        assert!(bash_cmds.contains("export PATH='/usr/bin'"));
        assert!(bash_cmds.contains("export MY_VAR='value'"));

        let fish_cmds = calc.generate_shell_commands(ShellType::Fish);
        assert!(fish_cmds.contains("set -gx PATH '/usr/bin'"));
    }

    #[test]
    fn test_shell_escape() {
        assert_eq!(shell_escape("simple"), "'simple'");
        assert_eq!(shell_escape("with space"), "'with space'");
        assert_eq!(shell_escape("it's quoted"), "'it'\\''s quoted'");
    }
}
