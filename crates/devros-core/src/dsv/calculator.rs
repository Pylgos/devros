//! Environment variable calculator

use camino::{Utf8Path, Utf8PathBuf};
use std::collections::{HashMap, HashSet};

use crate::Result;

use super::operation::DsvOperation;
use super::parser::DsvFile;
use super::shell::{ShellType, shell_escape};

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

#[cfg(test)]
mod tests {
    use super::*;

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
}
