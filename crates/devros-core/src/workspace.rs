//! Workspace analysis and dependency resolution
//!
//! This module handles:
//! - Package discovery (finding package.xml files)
//! - Dependency graph construction
//! - Topological sorting for build order

use camino::{Utf8Path, Utf8PathBuf};
use petgraph::algo::toposort;
use petgraph::graph::DiGraph;
use std::collections::{HashMap, HashSet};
use walkdir::WalkDir;

use crate::config::Config;
use crate::package::Package;
use crate::{Error, Result};

/// Marker files that indicate a directory should be ignored
const IGNORE_MARKERS: &[&str] = &["COLCON_IGNORE", "AMENT_IGNORE", "CATKIN_IGNORE"];

/// Directories that are always excluded from search
const EXCLUDED_DIRS: &[&str] = &["build", "install", "log", ".devros"];

/// Represents a ROS 2 workspace
#[derive(Debug)]
pub struct Workspace {
    /// Root directory of the workspace
    pub root: Utf8PathBuf,

    /// Configuration
    pub config: Config,

    /// Discovered packages indexed by name
    pub packages: HashMap<String, Package>,

    /// Build order (topologically sorted package names)
    pub build_order: Vec<String>,
}

impl Workspace {
    /// Discover and analyze a workspace
    pub fn discover(root: &Utf8Path) -> Result<Self> {
        let config = Config::load(root)?;
        Self::with_config(root, config)
    }

    /// Discover workspace with a specific configuration
    pub fn with_config(root: &Utf8Path, config: Config) -> Result<Self> {
        let root = root.canonicalize_utf8().map_err(|e| {
            Error::workspace(
                format!("Failed to canonicalize workspace root: {}", e),
                "Ensure the path exists and is accessible",
            )
        })?;

        // Discover packages
        let packages = discover_packages(&root, &config)?;

        // Build dependency graph and get build order
        let build_order = compute_build_order(&packages)?;

        Ok(Workspace {
            root,
            config,
            packages,
            build_order,
        })
    }

    /// Get packages in build order
    pub fn packages_in_build_order(&self) -> impl Iterator<Item = &Package> {
        self.build_order.iter().filter_map(|name| self.packages.get(name))
    }

    /// Get a package by name
    pub fn get_package(&self, name: &str) -> Option<&Package> {
        self.packages.get(name)
    }

    /// Get the build directory for a package
    pub fn package_build_dir(&self, package_name: &str) -> Utf8PathBuf {
        self.root
            .join(&self.config.workspace.build_dir)
            .join(package_name)
    }

    /// Get the install directory for a package
    pub fn package_install_dir(&self, package_name: &str) -> Utf8PathBuf {
        self.root
            .join(&self.config.workspace.install_dir)
            .join(package_name)
    }

    /// Get the state directory
    pub fn state_dir(&self) -> Utf8PathBuf {
        self.root.join(&self.config.workspace.state_dir)
    }
}

/// Discover all packages in a workspace
fn discover_packages(root: &Utf8Path, config: &Config) -> Result<HashMap<String, Package>> {
    let mut packages = HashMap::new();
    let skip_packages: HashSet<_> = config.build.skip_packages.iter().collect();

    // Start search from src directory if it exists, otherwise from root
    let search_root = if root.join("src").exists() {
        root.join("src")
    } else {
        root.to_path_buf()
    };

    let walker = WalkDir::new(&search_root).follow_links(true);

    for entry in walker.into_iter().filter_entry(|e| should_visit(e, root)) {
        let entry = entry.map_err(|e| {
            Error::workspace(
                format!("Failed to read directory entry: {}", e),
                "Check directory permissions",
            )
        })?;

        let path = entry.path();
        if path.is_file() && path.file_name() == Some(std::ffi::OsStr::new("package.xml")) {
            let utf8_path = Utf8Path::from_path(path).ok_or_else(|| {
                Error::workspace(
                    format!("Path is not valid UTF-8: {:?}", path),
                    "Ensure all paths are valid UTF-8",
                )
            })?;

            match Package::from_path(utf8_path) {
                Ok(package) => {
                    // Skip packages in skip_packages list
                    if skip_packages.contains(&package.name) {
                        tracing::debug!(name = %package.name, "Skipping package (in skip_packages)");
                        continue;
                    }

                    tracing::debug!(name = %package.name, path = %package.path, "Discovered package");

                    if let Some(existing) = packages.insert(package.name.clone(), package) {
                        return Err(Error::workspace(
                            format!("Duplicate package name: {}", existing.name),
                            format!(
                                "Package '{}' exists at both {} and the newly found location",
                                existing.name, existing.path
                            ),
                        ));
                    }
                }
                Err(e) => {
                    tracing::warn!(path = %utf8_path, error = %e, "Failed to parse package.xml");
                }
            }
        }
    }

    Ok(packages)
}

/// Check if a directory entry should be visited during package discovery
fn should_visit(entry: &walkdir::DirEntry, workspace_root: &Utf8Path) -> bool {
    let path = entry.path();

    // Always visit files
    if path.is_file() {
        return true;
    }

    // Get directory name
    let Some(dir_name) = path.file_name().and_then(|n| n.to_str()) else {
        return false;
    };

    // Skip hidden directories (starting with .)
    if dir_name.starts_with('.') {
        return false;
    }

    // Skip excluded directories at workspace root level
    let utf8_path = match Utf8Path::from_path(path) {
        Some(p) => p,
        None => return false,
    };

    // Check if this is a direct child of workspace root
    if let Some(parent) = utf8_path.parent() {
        if parent == workspace_root && EXCLUDED_DIRS.contains(&dir_name) {
            return false;
        }
    }

    // Check for ignore markers
    for marker in IGNORE_MARKERS {
        if path.join(marker).exists() {
            return false;
        }
    }

    true
}

/// Compute the build order using topological sort
fn compute_build_order(packages: &HashMap<String, Package>) -> Result<Vec<String>> {
    let package_names: HashSet<_> = packages.keys().collect();

    // Build directed graph: edge from A to B means "A depends on B" (B must be built before A)
    let mut graph = DiGraph::<&str, ()>::new();
    let mut node_indices: HashMap<&str, _> = HashMap::new();

    // Add nodes
    for name in packages.keys() {
        let idx = graph.add_node(name.as_str());
        node_indices.insert(name.as_str(), idx);
    }

    // Add edges for dependencies
    for (name, package) in packages {
        let dependent_idx = node_indices[name.as_str()];

        for dep in package.build_order_dependencies() {
            // Only consider dependencies that are in the workspace
            if package_names.contains(&dep.to_string()) {
                let dependency_idx = node_indices[dep];
                // Add edge: dependency -> dependent (dependency must be built first)
                graph.add_edge(dependency_idx, dependent_idx, ());
            }
        }
    }

    // Perform topological sort
    match toposort(&graph, None) {
        Ok(sorted_indices) => {
            let build_order: Vec<String> = sorted_indices
                .into_iter()
                .map(|idx| graph[idx].to_string())
                .collect();
            Ok(build_order)
        }
        Err(cycle) => {
            // Find the packages involved in the cycle
            let cycle_package = graph[cycle.node_id()].to_string();
            Err(Error::circular_dependency(vec![cycle_package]))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    fn create_package_xml(name: &str, deps: &[&str], build_type: &str) -> String {
        let dep_elements: String = deps
            .iter()
            .map(|d| format!("  <build_depend>{}</build_depend>\n", d))
            .collect();

        format!(
            r#"<?xml version="1.0"?>
<package format="3">
  <name>{}</name>
  <version>1.0.0</version>
{}  <export>
    <build_type>{}</build_type>
  </export>
</package>"#,
            name, dep_elements, build_type
        )
    }

    fn setup_test_workspace() -> TempDir {
        let temp_dir = TempDir::new().unwrap();
        let src_dir = temp_dir.path().join("src");
        fs::create_dir(&src_dir).unwrap();

        // Package A (no deps)
        let pkg_a = src_dir.join("pkg_a");
        fs::create_dir(&pkg_a).unwrap();
        fs::write(
            pkg_a.join("package.xml"),
            create_package_xml("pkg_a", &[], "ament_cmake"),
        )
        .unwrap();

        // Package B (depends on A)
        let pkg_b = src_dir.join("pkg_b");
        fs::create_dir(&pkg_b).unwrap();
        fs::write(
            pkg_b.join("package.xml"),
            create_package_xml("pkg_b", &["pkg_a"], "ament_cmake"),
        )
        .unwrap();

        // Package C (depends on A and B)
        let pkg_c = src_dir.join("pkg_c");
        fs::create_dir(&pkg_c).unwrap();
        fs::write(
            pkg_c.join("package.xml"),
            create_package_xml("pkg_c", &["pkg_a", "pkg_b"], "ament_cmake"),
        )
        .unwrap();

        temp_dir
    }

    #[test]
    fn test_discover_packages() {
        let temp_dir = setup_test_workspace();
        let root = Utf8Path::from_path(temp_dir.path()).unwrap();

        let workspace = Workspace::discover(root).unwrap();

        assert_eq!(workspace.packages.len(), 3);
        assert!(workspace.packages.contains_key("pkg_a"));
        assert!(workspace.packages.contains_key("pkg_b"));
        assert!(workspace.packages.contains_key("pkg_c"));
    }

    #[test]
    fn test_build_order() {
        let temp_dir = setup_test_workspace();
        let root = Utf8Path::from_path(temp_dir.path()).unwrap();

        let workspace = Workspace::discover(root).unwrap();

        // pkg_a should come before pkg_b and pkg_c
        // pkg_b should come before pkg_c
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
        let c_pos = workspace
            .build_order
            .iter()
            .position(|n| n == "pkg_c")
            .unwrap();

        assert!(a_pos < b_pos);
        assert!(a_pos < c_pos);
        assert!(b_pos < c_pos);
    }

    #[test]
    fn test_circular_dependency_detection() {
        let temp_dir = TempDir::new().unwrap();
        let src_dir = temp_dir.path().join("src");
        fs::create_dir(&src_dir).unwrap();

        // Create circular dependency: A -> B -> C -> A
        let pkg_a = src_dir.join("pkg_a");
        fs::create_dir(&pkg_a).unwrap();
        fs::write(
            pkg_a.join("package.xml"),
            create_package_xml("pkg_a", &["pkg_c"], "ament_cmake"),
        )
        .unwrap();

        let pkg_b = src_dir.join("pkg_b");
        fs::create_dir(&pkg_b).unwrap();
        fs::write(
            pkg_b.join("package.xml"),
            create_package_xml("pkg_b", &["pkg_a"], "ament_cmake"),
        )
        .unwrap();

        let pkg_c = src_dir.join("pkg_c");
        fs::create_dir(&pkg_c).unwrap();
        fs::write(
            pkg_c.join("package.xml"),
            create_package_xml("pkg_c", &["pkg_b"], "ament_cmake"),
        )
        .unwrap();

        let root = Utf8Path::from_path(temp_dir.path()).unwrap();
        let result = Workspace::discover(root);

        assert!(matches!(result, Err(Error::CircularDependency { .. })));
    }

    #[test]
    fn test_ignore_markers() {
        let temp_dir = TempDir::new().unwrap();
        let src_dir = temp_dir.path().join("src");
        fs::create_dir(&src_dir).unwrap();

        // Normal package
        let pkg_a = src_dir.join("pkg_a");
        fs::create_dir(&pkg_a).unwrap();
        fs::write(
            pkg_a.join("package.xml"),
            create_package_xml("pkg_a", &[], "ament_cmake"),
        )
        .unwrap();

        // Package with COLCON_IGNORE
        let pkg_ignored = src_dir.join("pkg_ignored");
        fs::create_dir(&pkg_ignored).unwrap();
        fs::write(
            pkg_ignored.join("package.xml"),
            create_package_xml("pkg_ignored", &[], "ament_cmake"),
        )
        .unwrap();
        fs::write(pkg_ignored.join("COLCON_IGNORE"), "").unwrap();

        let root = Utf8Path::from_path(temp_dir.path()).unwrap();
        let workspace = Workspace::discover(root).unwrap();

        assert_eq!(workspace.packages.len(), 1);
        assert!(workspace.packages.contains_key("pkg_a"));
        assert!(!workspace.packages.contains_key("pkg_ignored"));
    }

    #[test]
    fn test_skip_packages_config() {
        let temp_dir = setup_test_workspace();
        let root = Utf8Path::from_path(temp_dir.path()).unwrap();

        // Create config file with skip_packages
        let config_content = r#"
[build]
skip_packages = ["pkg_b"]
"#;
        fs::write(temp_dir.path().join("devros.toml"), config_content).unwrap();

        let workspace = Workspace::discover(root).unwrap();

        assert_eq!(workspace.packages.len(), 2);
        assert!(workspace.packages.contains_key("pkg_a"));
        assert!(!workspace.packages.contains_key("pkg_b"));
        assert!(workspace.packages.contains_key("pkg_c"));
    }

    #[test]
    fn test_hidden_directories_ignored() {
        let temp_dir = TempDir::new().unwrap();
        let src_dir = temp_dir.path().join("src");
        fs::create_dir(&src_dir).unwrap();

        // Normal package
        let pkg_a = src_dir.join("pkg_a");
        fs::create_dir(&pkg_a).unwrap();
        fs::write(
            pkg_a.join("package.xml"),
            create_package_xml("pkg_a", &[], "ament_cmake"),
        )
        .unwrap();

        // Package in hidden directory
        let hidden_dir = src_dir.join(".hidden");
        fs::create_dir(&hidden_dir).unwrap();
        let pkg_hidden = hidden_dir.join("pkg_hidden");
        fs::create_dir(&pkg_hidden).unwrap();
        fs::write(
            pkg_hidden.join("package.xml"),
            create_package_xml("pkg_hidden", &[], "ament_cmake"),
        )
        .unwrap();

        let root = Utf8Path::from_path(temp_dir.path()).unwrap();
        let workspace = Workspace::discover(root).unwrap();

        assert_eq!(workspace.packages.len(), 1);
        assert!(workspace.packages.contains_key("pkg_a"));
    }
}
