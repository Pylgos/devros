//! Integration tests for workspace discovery and analysis

use devros_core::package::BuildType;
use devros_core::workspace::Workspace;
use std::path::PathBuf;

fn fixtures_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/fixtures")
        .canonicalize()
        .expect("fixtures directory should exist")
}

#[test]
fn test_discover_simple_workspace() {
    let workspace_path = fixtures_path().join("simple_workspace");
    let workspace_root =
        camino::Utf8Path::from_path(&workspace_path).expect("path should be valid UTF-8");

    let workspace = Workspace::discover(workspace_root).expect("workspace discovery should succeed");

    // Should find both packages
    assert_eq!(workspace.packages.len(), 2);
    assert!(workspace.packages.contains_key("pkg_a"));
    assert!(workspace.packages.contains_key("pkg_b"));

    // Check pkg_a
    let pkg_a = workspace.packages.get("pkg_a").unwrap();
    assert_eq!(pkg_a.name, "pkg_a");
    assert_eq!(pkg_a.version, "0.1.0");
    assert_eq!(pkg_a.build_type, BuildType::AmentCmake);

    // Check pkg_b
    let pkg_b = workspace.packages.get("pkg_b").unwrap();
    assert_eq!(pkg_b.name, "pkg_b");
    assert_eq!(pkg_b.build_depend, vec!["pkg_a"]);

    // Check build order: pkg_a should come before pkg_b
    let a_pos = workspace
        .build_order
        .iter()
        .position(|n| n == "pkg_a")
        .unwrap();
    let b_pos = workspace
        .build_order
        .iter()
        .position(|n| n == "pkg_b")
        .unwrap();
    assert!(a_pos < b_pos, "pkg_a should be built before pkg_b");
}

#[test]
fn test_workspace_paths() {
    let workspace_path = fixtures_path().join("simple_workspace");
    let workspace_root =
        camino::Utf8Path::from_path(&workspace_path).expect("path should be valid UTF-8");

    let workspace = Workspace::discover(workspace_root).expect("workspace discovery should succeed");

    // Test build directory path
    let build_dir = workspace.package_build_dir("pkg_a");
    assert!(build_dir.ends_with("build/pkg_a"));

    // Test install directory path
    let install_dir = workspace.package_install_dir("pkg_a");
    assert!(install_dir.ends_with("install/pkg_a"));

    // Test state directory path
    let state_dir = workspace.state_dir();
    assert!(state_dir.ends_with(".devros"));
}
