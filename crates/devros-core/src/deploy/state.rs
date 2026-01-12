//! Deploy state management
//!
//! This module handles tracking changes between deployments.

use camino::Utf8Path;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use crate::{Error, Result};

/// Deploy state for tracking changes between deployments
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeployState {
    /// Timestamp of the last deployment
    pub timestamp: DateTime<Utc>,
    /// Package hashes at the time of deployment
    pub packages: HashMap<String, String>,
}

impl DeployState {
    /// Create a new deploy state
    pub fn new(packages: HashMap<String, String>) -> Self {
        Self {
            timestamp: Utc::now(),
            packages,
        }
    }

    /// Load deploy state from a file
    pub fn load(path: &Utf8Path) -> Result<Option<Self>> {
        if !path.exists() {
            return Ok(None);
        }

        let content = std::fs::read_to_string(path)?;
        let state: Self = serde_json::from_str(&content).map_err(|e| {
            Error::deploy(
                format!("Failed to parse deploy state: {}", e),
                "The deploy state file may be corrupted. Try deleting it.",
            )
        })?;

        Ok(Some(state))
    }

    /// Save deploy state to a file
    pub fn save(&self, path: &Utf8Path) -> Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }

        let content = serde_json::to_string_pretty(self).map_err(|e| {
            Error::deploy(
                format!("Failed to serialize deploy state: {}", e),
                "This is likely a bug in devros",
            )
        })?;

        std::fs::write(path, content)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_deploy_state_save_load() {
        let temp_dir = TempDir::new().unwrap();
        let state_path = Utf8Path::from_path(temp_dir.path())
            .unwrap()
            .join("state.json");

        let mut packages = HashMap::new();
        packages.insert("pkg_a".to_string(), "hash_a".to_string());
        packages.insert("pkg_b".to_string(), "hash_b".to_string());

        let state = DeployState::new(packages.clone());
        state.save(&state_path).unwrap();

        let loaded = DeployState::load(&state_path).unwrap().unwrap();

        assert_eq!(loaded.packages, packages);
    }

    #[test]
    fn test_deploy_state_missing_file() {
        let temp_dir = TempDir::new().unwrap();
        let state_path = Utf8Path::from_path(temp_dir.path())
            .unwrap()
            .join("nonexistent.json");

        let result = DeployState::load(&state_path).unwrap();
        assert!(result.is_none());
    }
}
