//! Build command implementation

use camino::Utf8Path;
use clap::Args;
use devros_core::package::BuildType;
use devros_core::workspace::Workspace;
use miette::{IntoDiagnostic, Result};
use std::process::Command;

/// Arguments for the build command
#[derive(Debug, Args)]
pub struct BuildArgs {
    /// Build specific packages only
    #[arg(short, long)]
    pub packages: Option<Vec<String>>,

    /// Number of parallel jobs
    #[arg(short, long)]
    pub jobs: Option<usize>,

    /// Use symlink install (default: true)
    #[arg(long, default_value = "true")]
    pub symlink_install: bool,

    /// Dry run - show what would be built
    #[arg(long)]
    pub dry_run: bool,
}

/// Run the build command
pub fn run(workspace_root: &Utf8Path, args: BuildArgs) -> Result<()> {
    tracing::info!("Discovering workspace at {}", workspace_root);

    let workspace = Workspace::discover(workspace_root).into_diagnostic()?;

    tracing::info!(
        "Found {} packages",
        workspace.packages.len()
    );

    // Determine which packages to build
    let packages_to_build: Vec<_> = if let Some(ref names) = args.packages {
        workspace
            .build_order
            .iter()
            .filter(|name| names.contains(name))
            .collect()
    } else {
        workspace.build_order.iter().collect()
    };

    if packages_to_build.is_empty() {
        tracing::warn!("No packages to build");
        return Ok(());
    }

    tracing::info!("Build order: {:?}", packages_to_build);

    if args.dry_run {
        println!("Would build the following packages in order:");
        for name in &packages_to_build {
            if let Some(pkg) = workspace.packages.get(*name) {
                println!("  - {} ({}) at {}", pkg.name, pkg.build_type, pkg.path);
            }
        }
        return Ok(());
    }

    // Get effective jobs
    let jobs = args.jobs.unwrap_or_else(|| workspace.config.effective_jobs());

    // Build each package
    for package_name in packages_to_build {
        let package = workspace
            .packages
            .get(package_name)
            .ok_or_else(|| miette::miette!("Package not found: {}", package_name))?;

        tracing::info!("Building {} ({})", package.name, package.build_type);

        match package.build_type {
            BuildType::AmentCmake => {
                build_ament_cmake(&workspace, package, &args, jobs)?;
            }
            BuildType::AmentPython => {
                tracing::warn!(
                    "ament_python build not yet implemented for {}",
                    package.name
                );
            }
            BuildType::Cmake => {
                tracing::warn!("cmake build not yet implemented for {}", package.name);
            }
            BuildType::Other(ref t) => {
                tracing::warn!(
                    "Unknown build type '{}' for {}, skipping",
                    t,
                    package.name
                );
            }
        }
    }

    // Generate workspace setup.bash
    generate_workspace_setup(&workspace)?;

    tracing::info!("Build complete!");
    Ok(())
}

/// Build an ament_cmake package
fn build_ament_cmake(
    workspace: &Workspace,
    package: &devros_core::package::Package,
    args: &BuildArgs,
    jobs: usize,
) -> Result<()> {
    let build_dir = workspace.package_build_dir(&package.name);
    let install_dir = workspace.package_install_dir(&package.name);
    let source_dir = &package.path;

    // Ensure directories exist
    std::fs::create_dir_all(&build_dir).into_diagnostic()?;
    std::fs::create_dir_all(&install_dir).into_diagnostic()?;

    // Build CMake arguments
    let mut cmake_args = vec![
        "-B".to_string(),
        build_dir.to_string(),
        "-S".to_string(),
        source_dir.to_string(),
        "-G".to_string(),
        "Unix Makefiles".to_string(),
        format!("-DCMAKE_INSTALL_PREFIX={}", install_dir),
        format!(
            "-DCMAKE_BUILD_TYPE={}",
            workspace.config.build.ament_cmake.build_type
        ),
    ];

    // Add symlink install flag
    if args.symlink_install {
        cmake_args.push("-DAMENT_CMAKE_SYMLINK_INSTALL=1".to_string());
    }

    // Add user-specified CMake args
    cmake_args.extend(workspace.config.build.ament_cmake.cmake_args.clone());

    // Compute environment for this package
    let env = compute_build_environment(workspace, package)?;

    // Configure
    tracing::debug!("Running cmake configure: cmake {:?}", cmake_args);
    let status = Command::new("cmake")
        .args(&cmake_args)
        .envs(&env)
        .status()
        .into_diagnostic()?;

    if !status.success() {
        return Err(miette::miette!(
            "CMake configure failed for {}",
            package.name
        ));
    }

    // Build
    tracing::debug!("Running cmake build with {} jobs", jobs);
    let status = Command::new("cmake")
        .args(["--build", build_dir.as_str(), "--parallel", &jobs.to_string()])
        .envs(&env)
        .status()
        .into_diagnostic()?;

    if !status.success() {
        return Err(miette::miette!("CMake build failed for {}", package.name));
    }

    // Install
    tracing::debug!("Running cmake install");
    let status = Command::new("cmake")
        .args(["--install", build_dir.as_str()])
        .envs(&env)
        .status()
        .into_diagnostic()?;

    if !status.success() {
        return Err(miette::miette!("CMake install failed for {}", package.name));
    }

    // Generate package.dsv and local_setup.sh
    generate_package_environment_files(workspace, package)?;

    Ok(())
}

/// Compute the environment variables needed to build a package
fn compute_build_environment(
    workspace: &Workspace,
    package: &devros_core::package::Package,
) -> Result<std::collections::HashMap<String, String>> {
    use devros_core::dsv::{DsvFile, DsvOperation, EnvCalculator};

    // Start with current environment
    let mut calc = EnvCalculator::from_current_env();

    // Process dependencies in topological order
    let deps: std::collections::HashSet<_> = package.build_order_dependencies().collect();

    for dep_name in &workspace.build_order {
        if deps.contains(dep_name.as_str()) {
            let dep_install_dir = workspace.package_install_dir(dep_name);
            let dsv_path = dep_install_dir.join("share").join(dep_name).join("package.dsv");

            if dsv_path.exists() {
                if let Ok(dsv) = DsvFile::parse(&dsv_path) {
                    calc.apply_dsv(&dsv, &dep_install_dir)
                        .into_diagnostic()?;
                }
            } else {
                // If no DSV file exists yet, at least add to AMENT_PREFIX_PATH
                calc.apply_operation(
                    &DsvOperation::PrependNonDuplicate {
                        variable: "AMENT_PREFIX_PATH".to_string(),
                        values: vec!["".to_string()],
                    },
                    &dep_install_dir,
                )
                .into_diagnostic()?;

                calc.apply_operation(
                    &DsvOperation::PrependNonDuplicate {
                        variable: "CMAKE_PREFIX_PATH".to_string(),
                        values: vec!["".to_string()],
                    },
                    &dep_install_dir,
                )
                .into_diagnostic()?;
            }
        }
    }

    Ok(calc.env().clone())
}

/// Generate package environment files (package.dsv and local_setup.sh)
fn generate_package_environment_files(
    workspace: &Workspace,
    package: &devros_core::package::Package,
) -> Result<()> {
    use devros_core::dsv::{generate_local_setup_sh, generate_package_dsv};

    let install_dir = workspace.package_install_dir(&package.name);
    let share_dir = install_dir.join("share").join(&package.name);

    std::fs::create_dir_all(&share_dir).into_diagnostic()?;

    // Generate package.dsv
    let dsv_content = generate_package_dsv(&package.name, &install_dir);
    std::fs::write(share_dir.join("package.dsv"), dsv_content).into_diagnostic()?;

    // Generate local_setup.sh
    let setup_content = generate_local_setup_sh(&package.name);
    let setup_path = share_dir.join("local_setup.sh");
    std::fs::write(&setup_path, setup_content).into_diagnostic()?;

    // Make local_setup.sh executable
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut perms = std::fs::metadata(&setup_path)
            .into_diagnostic()?
            .permissions();
        perms.set_mode(0o755);
        std::fs::set_permissions(&setup_path, perms).into_diagnostic()?;
    }

    Ok(())
}

/// Generate workspace-level setup.bash
fn generate_workspace_setup(workspace: &Workspace) -> Result<()> {
    use devros_core::dsv::generate_workspace_setup_bash;

    let install_dir = workspace.root.join(&workspace.config.workspace.install_dir);
    std::fs::create_dir_all(&install_dir).into_diagnostic()?;

    let content = generate_workspace_setup_bash();
    let setup_path = install_dir.join("setup.bash");
    std::fs::write(&setup_path, content).into_diagnostic()?;

    // Make executable
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut perms = std::fs::metadata(&setup_path)
            .into_diagnostic()?
            .permissions();
        perms.set_mode(0o755);
        std::fs::set_permissions(&setup_path, perms).into_diagnostic()?;
    }

    tracing::info!("Generated {}", setup_path);
    Ok(())
}
