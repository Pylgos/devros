//! Build cache management using Blake3 hashes
//!
//! This module implements a caching system for incremental builds.
//! It computes hashes of package sources and dependencies to determine
//! if a rebuild is necessary.

use camino::{Utf8Path, Utf8PathBuf};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::io::Read;
use walkdir::WalkDir;

use crate::{Error, Result};

/// Status of a cached build
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CacheStatus {
    /// Build succeeded
    Success,
    /// Build failed
    Failed,
}

/// Cache entry for a package
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CacheEntry {
    /// Blake3 hash of the package content
    pub hash: String,
    /// Timestamp of when the cache was created
    pub timestamp: DateTime<Utc>,
    /// Status of the build
    pub status: CacheStatus,
}

impl CacheEntry {
    /// Create a new cache entry with success status
    pub fn success(hash: String) -> Self {
        Self {
            hash,
            timestamp: Utc::now(),
            status: CacheStatus::Success,
        }
    }

    /// Create a new cache entry with failed status
    pub fn failed(hash: String) -> Self {
        Self {
            hash,
            timestamp: Utc::now(),
            status: CacheStatus::Failed,
        }
    }
}

/// Manages build caches for packages
#[derive(Debug)]
pub struct CacheManager {
    /// Base directory for cache storage
    cache_dir: Utf8PathBuf,
    /// In-memory cache of computed hashes
    hash_cache: HashMap<String, String>,
}

impl CacheManager {
    /// Create a new cache manager
    pub fn new(state_dir: &Utf8Path) -> Self {
        Self {
            cache_dir: state_dir.join("cache"),
            hash_cache: HashMap::new(),
        }
    }

    /// Get the cache file path for a package
    fn cache_file(&self, package_name: &str) -> Utf8PathBuf {
        self.cache_dir.join(package_name).join("hash.json")
    }

    /// Load cache entry for a package
    pub fn load(&self, package_name: &str) -> Result<Option<CacheEntry>> {
        let cache_file = self.cache_file(package_name);
        if !cache_file.exists() {
            return Ok(None);
        }

        let content = std::fs::read_to_string(&cache_file)?;
        let entry: CacheEntry = serde_json::from_str(&content).map_err(|e| {
            Error::cache(
                format!("Failed to parse cache file for {}: {}", package_name, e),
                "The cache file may be corrupted. Try deleting it.",
            )
        })?;

        Ok(Some(entry))
    }

    /// Save cache entry for a package
    pub fn save(&self, package_name: &str, entry: &CacheEntry) -> Result<()> {
        let cache_file = self.cache_file(package_name);
        if let Some(parent) = cache_file.parent() {
            std::fs::create_dir_all(parent)?;
        }

        let content = serde_json::to_string_pretty(entry).map_err(|e| {
            Error::cache(
                format!("Failed to serialize cache entry for {}: {}", package_name, e),
                "This is likely a bug in devros",
            )
        })?;

        std::fs::write(&cache_file, content)?;
        Ok(())
    }

    /// Check if a package needs to be rebuilt
    ///
    /// Returns true if the package needs to be rebuilt, false if cached.
    pub fn needs_rebuild(
        &mut self,
        package_name: &str,
        current_hash: &str,
    ) -> Result<bool> {
        match self.load(package_name)? {
            Some(entry) => {
                // Only skip if hash matches AND previous build succeeded
                if entry.hash == current_hash && entry.status == CacheStatus::Success {
                    tracing::debug!(
                        package = package_name,
                        "Cache hit - skipping build"
                    );
                    Ok(false)
                } else {
                    tracing::debug!(
                        package = package_name,
                        old_hash = entry.hash,
                        new_hash = current_hash,
                        "Cache miss - rebuild needed"
                    );
                    Ok(true)
                }
            }
            None => {
                tracing::debug!(
                    package = package_name,
                    "No cache entry - build needed"
                );
                Ok(true)
            }
        }
    }

    /// Mark a package build as successful
    pub fn mark_success(&mut self, package_name: &str, hash: String) -> Result<()> {
        let entry = CacheEntry::success(hash.clone());
        self.save(package_name, &entry)?;
        self.hash_cache.insert(package_name.to_string(), hash);
        Ok(())
    }

    /// Mark a package build as failed
    pub fn mark_failed(&mut self, package_name: &str, hash: String) -> Result<()> {
        let entry = CacheEntry::failed(hash);
        self.save(package_name, &entry)
    }

    /// Get the cached hash for a package (for dependency calculation)
    pub fn get_hash(&self, package_name: &str) -> Option<&str> {
        self.hash_cache.get(package_name).map(|s| s.as_str())
    }
}

/// Compute Blake3 hash for a package
pub fn compute_package_hash(
    package_path: &Utf8Path,
    dependency_hashes: &[&str],
) -> Result<String> {
    let mut hasher = blake3::Hasher::new();

    // Hash all files in the package directory
    hash_directory(&mut hasher, package_path)?;

    // Include dependency hashes
    for dep_hash in dependency_hashes {
        hasher.update(dep_hash.as_bytes());
    }

    Ok(hasher.finalize().to_hex().to_string())
}

/// Hash all files in a directory
fn hash_directory(hasher: &mut blake3::Hasher, dir: &Utf8Path) -> Result<()> {
    // Patterns to ignore
    const IGNORE_PATTERNS: &[&str] = &[
        ".git",
        ".gitignore",
        "__pycache__",
        "*.pyc",
        "build",
        "install",
        ".devros",
    ];

    let walker = WalkDir::new(dir).follow_links(true).into_iter();

    for entry in walker.filter_entry(|e| !should_ignore(e, IGNORE_PATTERNS)) {
        let entry = entry.map_err(|e| {
            Error::cache(
                format!("Failed to read directory entry: {}", e),
                "Check directory permissions",
            )
        })?;

        let path = entry.path();
        if path.is_file() {
            hash_file(hasher, path)?;
        }
    }

    Ok(())
}

/// Check if a directory entry should be ignored
fn should_ignore(entry: &walkdir::DirEntry, patterns: &[&str]) -> bool {
    let file_name = entry.file_name().to_string_lossy();

    for pattern in patterns {
        if pattern.starts_with('*') {
            // Simple suffix matching
            let suffix = &pattern[1..];
            if file_name.ends_with(suffix) {
                return true;
            }
        } else if file_name == *pattern {
            return true;
        }
    }

    false
}

/// Hash a single file
fn hash_file(hasher: &mut blake3::Hasher, path: &std::path::Path) -> Result<()> {
    // Include the relative path in the hash
    if let Some(file_name) = path.file_name().and_then(|n| n.to_str()) {
        hasher.update(file_name.as_bytes());
    }

    // Hash file contents
    let mut file = std::fs::File::open(path)?;
    let mut buffer = [0u8; 8192];

    loop {
        let bytes_read = file.read(&mut buffer)?;
        if bytes_read == 0 {
            break;
        }
        hasher.update(&buffer[..bytes_read]);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    #[test]
    fn test_cache_entry_serialization() {
        let entry = CacheEntry::success("abc123".to_string());
        let json = serde_json::to_string(&entry).unwrap();
        let parsed: CacheEntry = serde_json::from_str(&json).unwrap();

        assert_eq!(parsed.hash, "abc123");
        assert_eq!(parsed.status, CacheStatus::Success);
    }

    #[test]
    fn test_cache_manager_save_load() {
        let temp_dir = TempDir::new().unwrap();
        let state_dir = Utf8Path::from_path(temp_dir.path()).unwrap();

        let manager = CacheManager::new(state_dir);
        let entry = CacheEntry::success("test_hash".to_string());

        manager.save("test_package", &entry).unwrap();
        let loaded = manager.load("test_package").unwrap().unwrap();

        assert_eq!(loaded.hash, "test_hash");
        assert_eq!(loaded.status, CacheStatus::Success);
    }

    #[test]
    fn test_compute_package_hash() {
        let temp_dir = TempDir::new().unwrap();
        let pkg_dir = temp_dir.path().join("test_pkg");
        fs::create_dir(&pkg_dir).unwrap();

        fs::write(pkg_dir.join("file1.txt"), "content1").unwrap();
        fs::write(pkg_dir.join("file2.txt"), "content2").unwrap();

        let pkg_path = Utf8Path::from_path(&pkg_dir).unwrap();
        let hash1 = compute_package_hash(pkg_path, &[]).unwrap();

        // Hash should be consistent
        let hash2 = compute_package_hash(pkg_path, &[]).unwrap();
        assert_eq!(hash1, hash2);

        // Hash should change when file changes
        fs::write(pkg_dir.join("file1.txt"), "modified").unwrap();
        let hash3 = compute_package_hash(pkg_path, &[]).unwrap();
        assert_ne!(hash1, hash3);
    }

    #[test]
    fn test_needs_rebuild() {
        let temp_dir = TempDir::new().unwrap();
        let state_dir = Utf8Path::from_path(temp_dir.path()).unwrap();

        let mut manager = CacheManager::new(state_dir);

        // No cache - should need rebuild
        assert!(manager.needs_rebuild("pkg", "hash1").unwrap());

        // After success - should not need rebuild
        manager.mark_success("pkg", "hash1".to_string()).unwrap();
        assert!(!manager.needs_rebuild("pkg", "hash1").unwrap());

        // Hash changed - should need rebuild
        assert!(manager.needs_rebuild("pkg", "hash2").unwrap());
    }

    #[test]
    fn test_dependency_hash_affects_package_hash() {
        let temp_dir = TempDir::new().unwrap();
        let pkg_dir = temp_dir.path().join("test_pkg");
        fs::create_dir(&pkg_dir).unwrap();
        fs::write(pkg_dir.join("file.txt"), "content").unwrap();

        let pkg_path = Utf8Path::from_path(&pkg_dir).unwrap();

        let hash_no_deps = compute_package_hash(pkg_path, &[]).unwrap();
        let hash_with_dep = compute_package_hash(pkg_path, &["dep_hash"]).unwrap();

        assert_ne!(hash_no_deps, hash_with_dep);
    }
}
