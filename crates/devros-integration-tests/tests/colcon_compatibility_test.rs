//! Integration tests for colcon compatibility
//!
//! These tests build a ROS 2 workspace with both colcon and devros,
//! then compare the outputs to ensure compatibility.

use camino::{Utf8Path, Utf8PathBuf};
use devros_integration_tests::colcon_compat::{
    DsvEntries, compare_environment_variables, compare_install_dirs, scan_directory,
};
use std::fs;
use std::path::PathBuf;
use std::process::Command;

/// Get the source path to the integration workspace template
fn integration_ws_source_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/integration_ws")
        .canonicalize()
        .expect("integration_ws should exist")
}

/// Copy the integration workspace to a temporary directory for isolated testing.
/// This allows multiple tests to run in parallel without interfering with each other.
fn create_isolated_workspace() -> tempfile::TempDir {
    let source = integration_ws_source_path();
    let temp_dir = tempfile::tempdir().expect("Failed to create temp directory");

    // Copy the src directory (which contains the ROS packages)
    let src_source = source.join("src");
    let src_dest = temp_dir.path().join("src");
    copy_dir_recursive(&src_source, &src_dest).expect("Failed to copy workspace");

    temp_dir
}

/// Recursively copy a directory
fn copy_dir_recursive(src: &PathBuf, dst: &PathBuf) -> std::io::Result<()> {
    fs::create_dir_all(dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let file_type = entry.file_type()?;
        let src_path = entry.path();
        let dst_path = dst.join(entry.file_name());

        if file_type.is_dir() {
            copy_dir_recursive(&src_path, &dst_path)?;
        } else {
            fs::copy(&src_path, &dst_path)?;
        }
    }
    Ok(())
}

/// Get the path to the devros binary
fn devros_binary_path() -> PathBuf {
    // Look for the binary in the target directory
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir.join("../..");

    // Try debug first, then release
    let debug_path = workspace_root.join("target/debug/devros");
    let release_path = workspace_root.join("target/release/devros");

    if debug_path.exists() {
        debug_path.canonicalize().unwrap()
    } else if release_path.exists() {
        release_path.canonicalize().unwrap()
    } else {
        panic!("devros binary not found. Run `cargo build` first.");
    }
}

/// Source ROS 2 and return the modified environment
fn ros2_env() -> std::collections::HashMap<String, String> {
    // Try to source ROS 2 Jazzy or Humble
    let output = Command::new("bash")
        .args(["-c", "source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash && env"])
        .output()
        .expect("Failed to source ROS 2");

    let env_str = String::from_utf8_lossy(&output.stdout);
    let mut env = std::collections::HashMap::new();

    for line in env_str.lines() {
        if let Some((key, value)) = line.split_once('=') {
            env.insert(key.to_string(), value.to_string());
        }
    }

    env
}

/// Build with colcon and create a dereferenced copy
fn build_with_colcon(
    ws_path: &PathBuf,
    env: &std::collections::HashMap<String, String>,
) -> Utf8PathBuf {
    let status = Command::new("colcon")
        .args([
            "build",
            "--symlink-install",
            "--build-base",
            "colcon_build",
            "--install-base",
            "colcon_install",
        ])
        .current_dir(ws_path)
        .envs(env)
        .status()
        .expect("Failed to run colcon");

    assert!(status.success(), "colcon build failed");

    Utf8PathBuf::from_path_buf(ws_path.join("colcon_install")).unwrap()
}

/// Build with devros and create a dereferenced copy
fn build_with_devros(
    ws_path: &PathBuf,
    devros_binary: &PathBuf,
    env: &std::collections::HashMap<String, String>,
) -> Utf8PathBuf {
    let status = Command::new(devros_binary)
        .args(["build"])
        .current_dir(ws_path)
        .envs(env)
        .status()
        .expect("Failed to run devros");

    assert!(status.success(), "devros build failed");

    Utf8PathBuf::from_path_buf(ws_path.join("install")).unwrap()
}

#[test]
fn test_colcon_compatibility() {
    let temp_workspace = create_isolated_workspace();
    let ws_path = temp_workspace.path().to_path_buf();
    let devros_binary = devros_binary_path();
    let env = ros2_env();

    // Ensure ROS 2 is available
    if !env.contains_key("AMENT_PREFIX_PATH") {
        eprintln!("ROS 2 environment not available, skipping test");
        return;
    }

    // Build with both tools (no need to clean - workspace is fresh)
    let colcon_install = build_with_colcon(&ws_path, &env);
    let devros_install = build_with_devros(&ws_path, &devros_binary, &env);

    // Compare the outputs
    let errors = compare_install_dirs(&colcon_install, &devros_install)
        .expect("Failed to compare directories");

    // Print results for debugging
    if !errors.is_empty() {
        eprintln!("\n=== Compatibility test found {} errors ===", errors.len());
        for error in &errors {
            eprintln!("  - {}", error);
        }
    }

    assert!(
        errors.is_empty(),
        "Compatibility test failed with {} errors",
        errors.len()
    );
}

#[test]
fn test_dsv_file_parsing() {
    // Test that our DSV parsing correctly handles various formats
    let content = r#"source;share/pkg/local_setup.bash
source;share/pkg/local_setup.dsv
source;share/pkg/local_setup.sh
source;share/pkg/local_setup.zsh
source;share/pkg/hook/test.ps1
prepend-non-duplicate;AMENT_PREFIX_PATH;
prepend-non-duplicate-if-exists;PATH;bin
"#;

    let temp_dir = tempfile::tempdir().unwrap();
    let dsv_path = temp_dir.path().join("test.dsv");
    fs::write(&dsv_path, content).unwrap();

    let entries = DsvEntries::parse(&dsv_path).unwrap();

    // Check that .ps1 is filtered out
    assert!(!entries.source_entries.iter().any(|e| e.ends_with(".ps1")));

    // Check that shell entries are preserved
    assert!(
        entries
            .source_entries
            .contains("source;share/pkg/local_setup.bash")
    );
    assert!(
        entries
            .source_entries
            .contains("source;share/pkg/local_setup.sh")
    );
    assert!(
        entries
            .source_entries
            .contains("source;share/pkg/local_setup.dsv")
    );

    // Check other entries
    assert!(
        entries
            .other_entries
            .contains("prepend-non-duplicate;AMENT_PREFIX_PATH;")
    );
}

#[test]
fn test_directory_scan() {
    let temp_dir = tempfile::tempdir().unwrap();
    let root = temp_dir.path();

    // Create test structure
    fs::create_dir_all(root.join("share/pkg")).unwrap();
    fs::write(root.join("share/pkg/file.txt"), "content").unwrap();

    // Create a symlink (Unix only)
    #[cfg(unix)]
    {
        std::os::unix::fs::symlink("file.txt", root.join("share/pkg/link.txt")).unwrap();
    }

    let root_utf8 = Utf8Path::from_path(root).unwrap();
    let entries = scan_directory(root_utf8).expect("Failed to scan directory");

    // Check that files are found
    assert!(entries.contains_key(&Utf8PathBuf::from("share/pkg/file.txt")));

    // Check file properties
    let file_entry = entries
        .get(&Utf8PathBuf::from("share/pkg/file.txt"))
        .unwrap();
    assert!(!file_entry.is_symlink);
    assert!(!file_entry.is_dir);
    assert!(file_entry.content_hash.is_some());

    // Check symlink (Unix only)
    #[cfg(unix)]
    {
        let link_entry = entries
            .get(&Utf8PathBuf::from("share/pkg/link.txt"))
            .unwrap();
        assert!(link_entry.is_symlink);
        assert_eq!(
            link_entry.symlink_target.as_ref().unwrap().as_str(),
            "file.txt"
        );
    }
}

/// Test that environment variables are equivalent between colcon and devros
///
/// This test builds the workspace with both tools, sources their setup.bash scripts,
/// and compares the resulting environment variables. Differences in install directory
/// paths are normalized and ignored. Only actual functional differences cause failures.
#[test]
fn test_environment_variable_compatibility() {
    let temp_workspace = create_isolated_workspace();
    let ws_path = temp_workspace.path().to_path_buf();
    let devros_binary = devros_binary_path();
    let env = ros2_env();

    // Ensure ROS 2 is available
    if !env.contains_key("AMENT_PREFIX_PATH") {
        eprintln!("ROS 2 environment not available, skipping test");
        return;
    }

    // Build with both tools (no need to clean - workspace is fresh)
    let colcon_install = build_with_colcon(&ws_path, &env);
    let devros_install = build_with_devros(&ws_path, &devros_binary, &env);

    // Compare environment variables
    let differences = compare_environment_variables(&colcon_install, &devros_install, &env)
        .expect("Failed to compare environment variables");

    // Print differences for debugging
    if !differences.is_empty() {
        eprintln!(
            "\n=== Environment variable comparison found {} differences ===",
            differences.len()
        );
        for diff in &differences {
            eprintln!("  - {}", diff);
        }
    }

    // The test should pass if there are no significant differences
    // (differences in install paths are already normalized)
    assert!(
        differences.is_empty(),
        "Environment variable comparison failed with {} differences",
        differences.len()
    );
}
