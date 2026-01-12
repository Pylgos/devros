//! Utility functions for deployment
//!
//! This module contains helper functions used by the deployment system.

use camino::{Utf8Path, Utf8PathBuf};
use walkdir::WalkDir;

use crate::{Error, Result};

/// Parse SSH target string to extract host and optional port
///
/// Supports formats:
/// - "user@host" -> ("user@host", None)
/// - "user@host:port" -> ("user@host", Some(port))
pub fn parse_ssh_target(target: &str) -> (&str, Option<u16>) {
    // Check if there's a colon that could indicate a port
    // Format: user@host:port
    if let Some((host_part, port_str)) = target.rsplit_once(':') {
        // Only treat it as a port if it parses as a valid u16
        if let Ok(port) = port_str.parse::<u16>() {
            return (host_part, Some(port));
        }
    }
    (target, None)
}

/// Calculate a relative path from `from` to `to`
pub fn make_relative_path(from: &Utf8Path, to: &Utf8Path) -> Utf8PathBuf {
    // Simple implementation - find common prefix and build relative path
    let from_parts: Vec<_> = from.components().collect();
    let to_parts: Vec<_> = to.components().collect();

    // Find common prefix length
    let mut common_len = 0;
    for (a, b) in from_parts.iter().zip(to_parts.iter()) {
        if a == b {
            common_len += 1;
        } else {
            break;
        }
    }

    // Build relative path
    let mut result = Utf8PathBuf::new();

    // Go up from `from` to common ancestor
    for _ in common_len..from_parts.len() {
        result.push("..");
    }

    // Go down from common ancestor to `to`
    for part in &to_parts[common_len..] {
        result.push(part.as_str());
    }

    result
}

/// Recursively copy a directory
pub fn copy_dir_recursive(src: &Utf8Path, dst: &Utf8Path) -> Result<()> {
    std::fs::create_dir_all(dst)?;

    for entry in WalkDir::new(src).follow_links(false) {
        let entry = entry.map_err(|e| {
            Error::deploy(
                format!("Failed to read directory entry: {}", e),
                "Check directory permissions",
            )
        })?;

        let src_path = Utf8Path::from_path(entry.path()).ok_or_else(|| {
            Error::deploy(
                format!("Path is not valid UTF-8: {:?}", entry.path()),
                "Ensure all file paths contain only valid UTF-8 characters",
            )
        })?;

        let rel_path = src_path.strip_prefix(src).map_err(|_| {
            Error::deploy(
                format!("Failed to strip source prefix from {}", src_path),
                "This is an unexpected internal error",
            )
        })?;

        let dst_path = dst.join(rel_path);

        if entry.file_type().is_dir() {
            std::fs::create_dir_all(&dst_path)?;
        } else if entry.file_type().is_symlink() {
            // Preserve symlinks
            let target = std::fs::read_link(entry.path())?;
            if dst_path.exists() || dst_path.is_symlink() {
                std::fs::remove_file(&dst_path)?;
            }
            std::os::unix::fs::symlink(target, &dst_path)?;
        } else {
            if let Some(parent) = dst_path.parent() {
                std::fs::create_dir_all(parent)?;
            }
            std::fs::copy(src_path, &dst_path)?;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    #[test]
    fn test_make_relative_path() {
        // Same level
        assert_eq!(
            make_relative_path(Utf8Path::new("/a/b/c"), Utf8Path::new("/a/b/d")),
            Utf8PathBuf::from("../d")
        );

        // Deeper path
        assert_eq!(
            make_relative_path(Utf8Path::new("/a/b"), Utf8Path::new("/a/b/c/d")),
            Utf8PathBuf::from("c/d")
        );

        // Shallower path
        assert_eq!(
            make_relative_path(Utf8Path::new("/a/b/c/d"), Utf8Path::new("/a/b")),
            Utf8PathBuf::from("../..")
        );
    }

    #[test]
    fn test_copy_dir_recursive() {
        let temp_dir = TempDir::new().unwrap();
        let src = temp_dir.path().join("src");
        let dst = temp_dir.path().join("dst");

        // Create source structure
        fs::create_dir_all(src.join("subdir")).unwrap();
        fs::write(src.join("file1.txt"), "content1").unwrap();
        fs::write(src.join("subdir/file2.txt"), "content2").unwrap();

        let src_path = Utf8Path::from_path(&src).unwrap();
        let dst_path = Utf8Path::from_path(&dst).unwrap();

        copy_dir_recursive(src_path, dst_path).unwrap();

        // Verify
        assert!(dst.join("file1.txt").exists());
        assert!(dst.join("subdir/file2.txt").exists());
        assert_eq!(
            fs::read_to_string(dst.join("file1.txt")).unwrap(),
            "content1"
        );
        assert_eq!(
            fs::read_to_string(dst.join("subdir/file2.txt")).unwrap(),
            "content2"
        );
    }

    #[test]
    fn test_parse_ssh_target() {
        // Standard format without port
        assert_eq!(parse_ssh_target("user@host"), ("user@host", None));

        // With port
        assert_eq!(parse_ssh_target("user@host:22"), ("user@host", Some(22)));
        assert_eq!(
            parse_ssh_target("admin@192.168.1.100:2222"),
            ("admin@192.168.1.100", Some(2222))
        );

        // Edge cases
        assert_eq!(
            parse_ssh_target("user@host:invalid"),
            ("user@host:invalid", None)
        );
        assert_eq!(
            parse_ssh_target("user@host:99999"),
            ("user@host:99999", None)
        ); // Port out of range
    }
}
