//! Package parsing and representation
//!
//! This module handles parsing of ROS 2 package.xml files and provides
//! the Package structure for workspace analysis.

use camino::{Utf8Path, Utf8PathBuf};
use quick_xml::de::from_str;
use serde::{Deserialize, Serialize};

use crate::{Error, Result};

/// Build type for ROS 2 packages
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum BuildType {
    /// CMake-based package using ament
    #[default]
    AmentCmake,
    /// Python-based package using ament
    AmentPython,
    /// Pure CMake package
    Cmake,
    /// Unknown build type
    Other(String),
}

impl From<&str> for BuildType {
    fn from(s: &str) -> Self {
        match s {
            "ament_cmake" => BuildType::AmentCmake,
            "ament_python" => BuildType::AmentPython,
            "cmake" => BuildType::Cmake,
            other => BuildType::Other(other.to_string()),
        }
    }
}

impl std::fmt::Display for BuildType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BuildType::AmentCmake => write!(f, "ament_cmake"),
            BuildType::AmentPython => write!(f, "ament_python"),
            BuildType::Cmake => write!(f, "cmake"),
            BuildType::Other(s) => write!(f, "{}", s),
        }
    }
}

/// Represents a ROS 2 package parsed from package.xml
#[derive(Debug, Clone)]
pub struct Package {
    /// Package name
    pub name: String,

    /// Package version
    pub version: String,

    /// Package description
    pub description: Option<String>,

    /// Build type (ament_cmake, ament_python, etc.)
    pub build_type: BuildType,

    /// Path to the package directory
    pub path: Utf8PathBuf,

    /// Build dependencies
    pub build_depend: Vec<String>,

    /// Build tool dependencies
    pub buildtool_depend: Vec<String>,

    /// Build export dependencies
    pub build_export_depend: Vec<String>,

    /// Execution dependencies
    pub exec_depend: Vec<String>,

    /// Test dependencies
    pub test_depend: Vec<String>,

    /// General dependencies (applies to build, exec, and test)
    pub depend: Vec<String>,
}

impl Package {
    /// Parse a package from a package.xml file
    pub fn from_path(package_xml_path: &Utf8Path) -> Result<Self> {
        let content = std::fs::read_to_string(package_xml_path)?;
        let package_dir = package_xml_path
            .parent()
            .ok_or_else(|| Error::package("Invalid package path", "Path has no parent directory"))?
            .to_path_buf();

        Self::from_str(&content, package_dir)
    }

    /// Parse a package from XML content
    pub fn from_str(content: &str, path: Utf8PathBuf) -> Result<Self> {
        let raw: RawPackageXml = from_str(content)?;

        let build_type = raw
            .export
            .as_ref()
            .and_then(|e| e.build_type.as_deref())
            .map(BuildType::from)
            .unwrap_or(BuildType::AmentCmake);

        Ok(Package {
            name: raw.name,
            version: raw.version,
            description: raw.description,
            build_type,
            path,
            build_depend: extract_deps(&raw.build_depend),
            buildtool_depend: extract_deps(&raw.buildtool_depend),
            build_export_depend: extract_deps(&raw.build_export_depend),
            exec_depend: extract_deps(&raw.exec_depend),
            test_depend: extract_deps(&raw.test_depend),
            depend: extract_deps(&raw.depend),
        })
    }

    /// Get all dependencies that affect build order
    pub fn build_order_dependencies(&self) -> impl Iterator<Item = &str> {
        self.build_depend
            .iter()
            .chain(self.buildtool_depend.iter())
            .chain(self.build_export_depend.iter())
            .chain(self.depend.iter())
            .map(|s| s.as_str())
    }

    /// Get all dependencies including execution and test
    pub fn all_dependencies(&self) -> impl Iterator<Item = &str> {
        self.build_order_dependencies()
            .chain(self.exec_depend.iter().map(|s| s.as_str()))
            .chain(self.test_depend.iter().map(|s| s.as_str()))
    }

    /// Get runtime dependencies (exec_depend + depend)
    /// These are the dependencies needed at runtime/sourcing time.
    pub fn run_dependencies(&self) -> impl Iterator<Item = &str> {
        self.exec_depend
            .iter()
            .chain(self.depend.iter())
            .map(|s| s.as_str())
    }
}

/// Raw package.xml structure for deserialization
#[derive(Debug, Deserialize)]
struct RawPackageXml {
    name: String,
    version: String,
    description: Option<String>,
    #[serde(default)]
    build_depend: Vec<Dependency>,
    #[serde(default)]
    buildtool_depend: Vec<Dependency>,
    #[serde(default)]
    build_export_depend: Vec<Dependency>,
    #[serde(default)]
    exec_depend: Vec<Dependency>,
    #[serde(default)]
    test_depend: Vec<Dependency>,
    #[serde(default)]
    depend: Vec<Dependency>,
    export: Option<Export>,
}

/// Dependency element
#[derive(Debug, Deserialize)]
struct Dependency {
    #[serde(rename = "$text")]
    name: String,
    #[serde(rename = "@condition")]
    condition: Option<String>,
}

/// Export section
#[derive(Debug, Deserialize)]
struct Export {
    build_type: Option<String>,
}

/// Extract dependency names from raw dependency list
///
/// Note: Conditional dependencies (with `condition` attribute) are currently skipped.
/// See: <https://github.com/ros-infrastructure/rep/blob/master/rep-0149.rst#conditional-dependencies>
fn extract_deps(deps: &[Dependency]) -> Vec<String> {
    deps.iter()
        .filter(|d| {
            // Skip dependencies with conditions - they require environment variable evaluation
            // which is not yet implemented
            d.condition.is_none()
        })
        .map(|d| d.name.clone())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_minimal_package_xml() {
        let xml = r#"<?xml version="1.0"?>
<package format="3">
  <name>test_package</name>
  <version>1.0.0</version>
</package>"#;

        let package = Package::from_str(xml, Utf8PathBuf::from("/test")).unwrap();

        assert_eq!(package.name, "test_package");
        assert_eq!(package.version, "1.0.0");
        assert_eq!(package.build_type, BuildType::AmentCmake); // default
        assert!(package.build_depend.is_empty());
    }

    #[test]
    fn test_parse_full_package_xml() {
        let xml = r#"<?xml version="1.0"?>
<package format="3">
  <name>my_ros_package</name>
  <version>2.1.0</version>
  <description>A test ROS 2 package</description>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rclcpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>rclcpp</build_export_depend>
  <exec_depend>rclcpp</exec_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <depend>geometry_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>"#;

        let package = Package::from_str(xml, Utf8PathBuf::from("/my_package")).unwrap();

        assert_eq!(package.name, "my_ros_package");
        assert_eq!(package.version, "2.1.0");
        assert_eq!(package.description.as_deref(), Some("A test ROS 2 package"));
        assert_eq!(package.build_type, BuildType::AmentCmake);
        assert_eq!(package.buildtool_depend, vec!["ament_cmake"]);
        assert_eq!(package.build_depend, vec!["rclcpp", "std_msgs"]);
        assert_eq!(package.build_export_depend, vec!["rclcpp"]);
        assert_eq!(package.exec_depend, vec!["rclcpp"]);
        assert_eq!(package.test_depend, vec!["ament_cmake_gtest"]);
        assert_eq!(package.depend, vec!["geometry_msgs"]);
    }

    #[test]
    fn test_parse_ament_python_package() {
        let xml = r#"<?xml version="1.0"?>
<package format="3">
  <name>python_package</name>
  <version>1.0.0</version>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>"#;

        let package = Package::from_str(xml, Utf8PathBuf::from("/python_pkg")).unwrap();

        assert_eq!(package.build_type, BuildType::AmentPython);
    }

    #[test]
    fn test_build_order_dependencies() {
        let xml = r#"<?xml version="1.0"?>
<package format="3">
  <name>test_pkg</name>
  <version>1.0.0</version>
  <build_depend>dep_a</build_depend>
  <buildtool_depend>dep_b</buildtool_depend>
  <build_export_depend>dep_c</build_export_depend>
  <exec_depend>dep_d</exec_depend>
  <depend>dep_e</depend>
</package>"#;

        let package = Package::from_str(xml, Utf8PathBuf::from("/test")).unwrap();
        let deps: Vec<_> = package.build_order_dependencies().collect();

        // exec_depend should not be included in build order
        assert!(deps.contains(&"dep_a"));
        assert!(deps.contains(&"dep_b"));
        assert!(deps.contains(&"dep_c"));
        assert!(deps.contains(&"dep_e"));
        assert!(!deps.contains(&"dep_d"));
    }

    #[test]
    fn test_build_type_from_str() {
        assert_eq!(BuildType::from("ament_cmake"), BuildType::AmentCmake);
        assert_eq!(BuildType::from("ament_python"), BuildType::AmentPython);
        assert_eq!(BuildType::from("cmake"), BuildType::Cmake);
        assert_eq!(
            BuildType::from("custom"),
            BuildType::Other("custom".to_string())
        );
    }
}
