//! Error types for devros

// This warning is a false positive from thiserror macro expansion
#![allow(unused_assignments)]

use miette::Diagnostic;
use thiserror::Error;

/// Result type alias for devros operations
pub type Result<T> = std::result::Result<T, Error>;

/// Main error type for devros
#[derive(Debug, Error, Diagnostic)]
#[allow(clippy::enum_variant_names)]
pub enum Error {
    /// I/O error
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// TOML parsing error
    #[error("Failed to parse TOML: {0}")]
    TomlParse(#[from] toml::de::Error),

    /// XML parsing error
    #[error("Failed to parse XML: {0}")]
    XmlParse(#[from] quick_xml::DeError),

    /// Configuration error
    #[error("Configuration error: {message}")]
    Config { message: String, help: String },

    /// Package error
    #[error("Package error: {message}")]
    Package { message: String, help: String },

    /// Workspace error
    #[error("Workspace error: {message}")]
    Workspace { message: String, help: String },

    /// Dependency resolution error
    #[error("Dependency resolution error: {message}")]
    DependencyResolution { message: String, help: String },

    /// Circular dependency detected
    #[error("Circular dependency detected: {packages:?}")]
    #[diagnostic(help("Check the dependency declarations in the package.xml files"))]
    CircularDependency {
        /// Packages involved in the cycle
        packages: Vec<String>,
    },

    /// DSV parsing error
    #[error("DSV parsing error: {message}")]
    DsvParse { message: String, help: String },

    /// Cache error
    #[error("Cache error: {message}")]
    Cache { message: String, help: String },

    /// Deploy error
    #[error("Deploy error: {message}")]
    Deploy { message: String, help: String },

    /// Build error
    #[error("Build error: {message}")]
    Build { message: String, help: String },
}

impl Error {
    /// Create a configuration error
    pub fn config(message: impl Into<String>, help: impl Into<String>) -> Self {
        Self::Config {
            message: message.into(),
            help: help.into(),
        }
    }

    /// Create a package error
    pub fn package(message: impl Into<String>, help: impl Into<String>) -> Self {
        Self::Package {
            message: message.into(),
            help: help.into(),
        }
    }

    /// Create a workspace error
    pub fn workspace(message: impl Into<String>, help: impl Into<String>) -> Self {
        Self::Workspace {
            message: message.into(),
            help: help.into(),
        }
    }

    /// Create a dependency resolution error
    pub fn dependency_resolution(message: impl Into<String>, help: impl Into<String>) -> Self {
        Self::DependencyResolution {
            message: message.into(),
            help: help.into(),
        }
    }

    /// Create a circular dependency error
    pub fn circular_dependency(packages: Vec<String>) -> Self {
        Self::CircularDependency { packages }
    }

    /// Create a DSV parsing error
    pub fn dsv_parse(message: impl Into<String>, help: impl Into<String>) -> Self {
        Self::DsvParse {
            message: message.into(),
            help: help.into(),
        }
    }

    /// Create a cache error
    pub fn cache(message: impl Into<String>, help: impl Into<String>) -> Self {
        Self::Cache {
            message: message.into(),
            help: help.into(),
        }
    }

    /// Create a deploy error
    pub fn deploy(message: impl Into<String>, help: impl Into<String>) -> Self {
        Self::Deploy {
            message: message.into(),
            help: help.into(),
        }
    }

    /// Create a build error
    pub fn build(message: impl Into<String>, help: impl Into<String>) -> Self {
        Self::Build {
            message: message.into(),
            help: help.into(),
        }
    }
}
