//! Environment computation for package builds
//!
//! This module handles computing the environment variables needed to build a package.

use std::collections::{HashMap, HashSet};

use crate::dsv::{DsvFile, DsvOperation, EnvCalculator};
use crate::workspace::Workspace;
use crate::{Result, package::Package};

/// Compute the environment variables needed to build a package
pub fn compute_build_environment(
    workspace: &Workspace,
    package: &Package,
) -> Result<HashMap<String, String>> {
    // Start with current environment
    let mut calc = EnvCalculator::from_current_env();

    // Process dependencies in topological order
    let deps: HashSet<_> = package.build_order_dependencies().collect();

    for dep_name in &workspace.build_order {
        if deps.contains(dep_name.as_str()) {
            let dep_install_dir = workspace.package_install_dir(dep_name);
            let dsv_path = dep_install_dir
                .join("share")
                .join(dep_name)
                .join("package.dsv");

            if dsv_path.exists() {
                if let Ok(dsv) = DsvFile::parse(&dsv_path) {
                    calc.apply_dsv(&dsv, &dep_install_dir)?;
                }
            } else {
                // If no DSV file exists yet, at least add to AMENT_PREFIX_PATH
                calc.apply_operation(
                    &DsvOperation::PrependNonDuplicate {
                        variable: "AMENT_PREFIX_PATH".to_string(),
                        values: vec!["".to_string()],
                    },
                    &dep_install_dir,
                )?;

                calc.apply_operation(
                    &DsvOperation::PrependNonDuplicate {
                        variable: "CMAKE_PREFIX_PATH".to_string(),
                        values: vec!["".to_string()],
                    },
                    &dep_install_dir,
                )?;
            }
        }
    }

    Ok(calc.env().clone())
}
