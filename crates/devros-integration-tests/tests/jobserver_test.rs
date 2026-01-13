//! Integration tests for jobserver functionality
//!
//! These tests verify that the jobserver implementation correctly limits
//! process counts during parallel builds, preventing process explosion.

use devros_integration_tests::process_monitor::ProcessMonitor;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

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
        .args([
            "-c",
            "source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash && env",
        ])
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

/// Create a mock workspace for testing with 4 independent packages
fn create_mock_workspace() -> tempfile::TempDir {
    let temp_dir = tempfile::tempdir().expect("Failed to create temp directory");
    let src_dir = temp_dir.path().join("src");
    fs::create_dir(&src_dir).expect("Failed to create src directory");

    // Create 4 independent packages that can be built in parallel
    for pkg_name in &["pkg_a", "pkg_b", "pkg_c", "pkg_d"] {
        create_mock_package(&src_dir, pkg_name);
    }

    temp_dir
}

/// Create a single mock package with multiple build targets
fn create_mock_package(src_dir: &Path, name: &str) {
    let pkg_dir = src_dir.join(name);
    fs::create_dir(&pkg_dir).expect("Failed to create package directory");

    // Create package.xml
    let package_xml = format!(
        r#"<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{name}</name>
  <version>0.0.1</version>
  <description>Mock package for jobserver testing</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"#
    );
    fs::write(pkg_dir.join("package.xml"), package_xml).expect("Failed to write package.xml");

    // Create CMakeLists.txt with 4 custom targets that each run sleep
    let cmakelists = format!(
        r#"cmake_minimum_required(VERSION 3.5)
project({name})

find_package(ament_cmake REQUIRED)

# 4 independent custom targets
# Each runs sleep to simulate build time
# Make will try to run these in parallel
add_custom_target(target1 ALL
    COMMAND ${{CMAKE_COMMAND}} -E echo "Building target1 in ${{PROJECT_NAME}}..."
    COMMAND sleep 2
    COMMAND ${{CMAKE_COMMAND}} -E echo "target1 complete"
)

add_custom_target(target2 ALL
    COMMAND ${{CMAKE_COMMAND}} -E echo "Building target2 in ${{PROJECT_NAME}}..."
    COMMAND sleep 2
    COMMAND ${{CMAKE_COMMAND}} -E echo "target2 complete"
)

add_custom_target(target3 ALL
    COMMAND ${{CMAKE_COMMAND}} -E echo "Building target3 in ${{PROJECT_NAME}}..."
    COMMAND sleep 2
    COMMAND ${{CMAKE_COMMAND}} -E echo "target3 complete"
)

add_custom_target(target4 ALL
    COMMAND ${{CMAKE_COMMAND}} -E echo "Building target4 in ${{PROJECT_NAME}}..."
    COMMAND sleep 2
    COMMAND ${{CMAKE_COMMAND}} -E echo "target4 complete"
)

ament_package()
"#
    );
    fs::write(pkg_dir.join("CMakeLists.txt"), cmakelists).expect("Failed to write CMakeLists.txt");
}

/// Test that jobserver prevents process explosion
#[test]
fn test_jobserver_prevents_process_explosion() {
    let workspace = create_mock_workspace();
    let workspace_path = workspace.path();
    let devros_binary = devros_binary_path();
    let env = ros2_env();

    // Ensure ROS 2 is available
    if !env.contains_key("AMENT_PREFIX_PATH") {
        eprintln!("ROS 2 environment not available, skipping test");
        return;
    }

    let jobs = 4;

    // Start monitoring sleep processes
    // We monitor only "sleep" processes to avoid counting unrelated system processes
    let monitor = ProcessMonitor::start("sleep 2");

    // Build with devros using limited parallelism
    let output = Command::new(&devros_binary)
        .arg("build")
        .arg("--jobs")
        .arg(jobs.to_string())
        .current_dir(workspace_path)
        .envs(&env)
        .output()
        .expect("Failed to run devros");

    // Stop monitoring and get statistics
    let stats = monitor.stop();

    // Print build output for debugging
    if !output.status.success() {
        eprintln!("Build stdout:\n{}", String::from_utf8_lossy(&output.stdout));
        eprintln!("Build stderr:\n{}", String::from_utf8_lossy(&output.stderr));
    }

    // Verify build succeeded
    assert!(
        output.status.success(),
        "Build should succeed. Check output above for errors."
    );

    // Analyze process statistics
    let max_allowed = jobs + 2;
    assert!(
        stats.max_processes <= max_allowed,
        "Process explosion detected! Max sleep processes: {} (expected <= {}). Samples: {:?}",
        stats.max_processes,
        max_allowed,
        stats.samples
    );

    // Verify all packages were built
    let install_dir = workspace_path.join("install");
    for pkg_name in &["pkg_a", "pkg_b", "pkg_c", "pkg_d"] {
        let pkg_install_dir = install_dir.join(pkg_name);
        assert!(
            pkg_install_dir.exists(),
            "Package {} should be installed",
            pkg_name
        );
    }
}
