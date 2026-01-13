//! Integration tests for parallel builds
//!
//! These tests verify that the parallel build functionality works correctly
//! with various dependency patterns.

use std::fs;
use std::path::PathBuf;
use std::process::Command;
use std::time::Instant;

/// Get the source path to the integration workspace template
fn integration_ws_source_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/integration_ws")
        .canonicalize()
        .expect("integration_ws should exist")
}

/// Copy the integration workspace to a temporary directory for isolated testing.
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

/// Test that parallel builds complete successfully
#[test]
fn test_parallel_build_succeeds() {
    let temp_workspace = create_isolated_workspace();
    let ws_path = temp_workspace.path().to_path_buf();
    let devros_binary = devros_binary_path();
    let env = ros2_env();

    // Ensure ROS 2 is available
    if !env.contains_key("AMENT_PREFIX_PATH") {
        eprintln!("ROS 2 environment not available, skipping test");
        return;
    }

    let start = Instant::now();

    // Build with devros using parallel execution
    let output = Command::new(&devros_binary)
        .args(["build", "--jobs", "4"])
        .current_dir(&ws_path)
        .envs(&env)
        .output()
        .expect("Failed to run devros");

    let duration = start.elapsed();

    println!("Build output:\n{}", String::from_utf8_lossy(&output.stdout));
    if !output.status.success() {
        eprintln!("Build stderr:\n{}", String::from_utf8_lossy(&output.stderr));
    }

    assert!(output.status.success(), "devros parallel build failed");

    // Verify all packages were built
    let install_dir = ws_path.join("install");
    let expected_packages = [
        "pkg_a_lib",
        "pkg_b_lib",
        "pkg_c_app",
        "pkg_d_py",
        "pkg_e_standalone",
        "example_node",
        "example_py",
    ];

    for pkg_name in &expected_packages {
        let pkg_install_dir = install_dir.join(pkg_name);
        assert!(
            pkg_install_dir.exists(),
            "Package {} was not built (install dir missing: {})",
            pkg_name,
            pkg_install_dir.display()
        );
    }

    println!("Parallel build completed in {:?}", duration);
    println!(
        "All {} packages built successfully",
        expected_packages.len()
    );
}

/// Test that dependency order is respected in parallel builds
#[test]
fn test_parallel_build_respects_dependencies() {
    let temp_workspace = create_isolated_workspace();
    let ws_path = temp_workspace.path().to_path_buf();
    let devros_binary = devros_binary_path();
    let env = ros2_env();

    // Ensure ROS 2 is available
    if !env.contains_key("AMENT_PREFIX_PATH") {
        eprintln!("ROS 2 environment not available, skipping test");
        return;
    }

    // Build with devros
    let output = Command::new(&devros_binary)
        .args(["build"])
        .current_dir(&ws_path)
        .envs(&env)
        .output()
        .expect("Failed to run devros");

    assert!(output.status.success(), "devros build failed");

    // Verify that pkg_c_app can find its dependencies
    // If dependencies weren't built first, the cmake find_package would fail
    let install_dir = ws_path.join("install");

    // pkg_c_app should have been built successfully
    assert!(
        install_dir.join("pkg_c_app").exists(),
        "pkg_c_app should be built (it depends on pkg_a_lib and pkg_b_lib)"
    );

    // example_py should have been built successfully
    assert!(
        install_dir.join("example_py").exists(),
        "example_py should be built (it depends on example_node)"
    );
}

/// Test that cache works correctly with parallel builds
#[test]
fn test_parallel_build_with_cache() {
    let temp_workspace = create_isolated_workspace();
    let ws_path = temp_workspace.path().to_path_buf();
    let devros_binary = devros_binary_path();
    let env = ros2_env();

    // Ensure ROS 2 is available
    if !env.contains_key("AMENT_PREFIX_PATH") {
        eprintln!("ROS 2 environment not available, skipping test");
        return;
    }

    // First build
    let output1 = Command::new(&devros_binary)
        .args(["build"])
        .current_dir(&ws_path)
        .envs(&env)
        .output()
        .expect("Failed to run devros");

    assert!(output1.status.success(), "First build failed");

    // Second build should use cache
    let start = Instant::now();
    let output2 = Command::new(&devros_binary)
        .args(["build"])
        .current_dir(&ws_path)
        .envs(&env)
        .output()
        .expect("Failed to run devros");

    let duration = start.elapsed();

    assert!(output2.status.success(), "Second build failed");

    // Cache hit build should be very fast
    println!("Cached build completed in {:?}", duration);

    // The output should indicate packages were skipped
    let output_str = String::from_utf8_lossy(&output2.stdout);
    let stderr_str = String::from_utf8_lossy(&output2.stderr);
    let combined = format!("{}{}", output_str, stderr_str);

    // At least some packages should be skipped (up to date)
    assert!(
        combined.contains("up to date") || combined.contains("Skipping"),
        "Expected some packages to be skipped due to cache"
    );
}
