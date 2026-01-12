//! Build command implementation

use camino::{Utf8Path, Utf8PathBuf};
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
                build_ament_python(&workspace, package, &args)?;
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
    // Note: We must set current_dir to build_dir to ensure symlink_install_manifest.txt
    // is created in the build directory, not in the current working directory
    tracing::debug!("Running cmake install");
    let status = Command::new("cmake")
        .args(["--install", build_dir.as_str()])
        .current_dir(&build_dir)
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

/// Get Python version string (e.g., "3.12")
fn get_python_version() -> Result<String> {
    let output = Command::new("python3")
        .args(["-c", "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"])
        .output()
        .into_diagnostic()?;

    if !output.status.success() {
        return Err(miette::miette!("Failed to get Python version"));
    }

    Ok(String::from_utf8_lossy(&output.stdout).trim().to_string())
}

/// Get Python lib directory (e.g., "lib/python3.12/site-packages")
fn get_python_lib_dir() -> Result<String> {
    let version = get_python_version()?;
    Ok(format!("lib/python{}/site-packages", version))
}

/// Build an ament_python package
fn build_ament_python(
    workspace: &Workspace,
    package: &devros_core::package::Package,
    args: &BuildArgs,
) -> Result<()> {
    let build_dir = workspace.package_build_dir(&package.name);
    let install_dir = workspace.package_install_dir(&package.name);
    let source_dir = &package.path;

    // Ensure directories exist
    std::fs::create_dir_all(&build_dir).into_diagnostic()?;
    std::fs::create_dir_all(&install_dir).into_diagnostic()?;

    // Get Python lib directory path
    let python_lib_dir = get_python_lib_dir()?;
    let python_lib_path = install_dir.join(&python_lib_dir);
    std::fs::create_dir_all(&python_lib_path).into_diagnostic()?;

    // Create prefix_override directory with sitecustomize.py
    let prefix_override = build_dir.join("prefix_override");
    std::fs::create_dir_all(&prefix_override).into_diagnostic()?;

    // Generate sitecustomize.py to redirect installation
    let sitecustomize_content = generate_sitecustomize(&install_dir)?;
    std::fs::write(prefix_override.join("sitecustomize.py"), sitecustomize_content)
        .into_diagnostic()?;

    // Compute environment for this package
    let mut env = compute_build_environment(workspace, package)?;

    // Set PYTHONPATH with proper priority:
    // 1. prefix_override (for sitecustomize.py)
    // 2. Python lib path (for self-dependency resolution)
    // 3. Existing PYTHONPATH
    let existing_pythonpath = env.get("PYTHONPATH").cloned().unwrap_or_default();
    let new_pythonpath = format!(
        "{}:{}:{}",
        prefix_override,
        python_lib_path,
        existing_pythonpath
    );
    env.insert("PYTHONPATH".to_string(), new_pythonpath);

    // Remove coverage-related env vars that interfere with sitecustomize
    env.remove("COV_CORE_SOURCE");

    if args.symlink_install {
        // Symlink install mode: use develop command
        build_ament_python_develop(workspace, package, &build_dir, &install_dir, source_dir, &env)?;
    } else {
        // Standard install mode
        build_ament_python_install(workspace, package, &build_dir, &install_dir, source_dir, &env)?;
    }

    // Post-processing: ensure package.xml is installed
    ensure_package_xml_installed(package, &install_dir, args.symlink_install)?;

    // Ensure ament resource index entry exists
    ensure_ament_resource_index(package, &install_dir)?;

    // Generate package.dsv and local_setup.sh
    generate_python_package_environment_files(workspace, package)?;

    Ok(())
}

/// Build ament_python package using setup.py install (non-symlink mode)
fn build_ament_python_install(
    workspace: &Workspace,
    package: &devros_core::package::Package,
    build_dir: &Utf8PathBuf,
    install_dir: &Utf8PathBuf,
    source_dir: &Utf8Path,
    env: &std::collections::HashMap<String, String>,
) -> Result<()> {
    let _ = workspace; // unused but kept for consistency

    // Run egg_info
    let egg_base = build_dir.to_string();
    let status = Command::new("python3")
        .args([
            "-W", "ignore:setup.py install is deprecated",
            "-W", "ignore:easy_install command is deprecated",
            "setup.py",
            "egg_info",
            "--egg-base", &egg_base,
        ])
        .current_dir(source_dir)
        .envs(env)
        .status()
        .into_diagnostic()?;

    if !status.success() {
        return Err(miette::miette!("setup.py egg_info failed for {}", package.name));
    }

    // Run build and install
    let build_base = build_dir.join("build");
    let install_log = build_dir.join("install.log");

    let status = Command::new("python3")
        .args([
            "-W", "ignore:setup.py install is deprecated",
            "-W", "ignore:easy_install command is deprecated",
            "setup.py",
            "build", "--build-base", build_base.as_str(),
            "install",
            "--prefix", install_dir.as_str(),
            "--record", install_log.as_str(),
            "--single-version-externally-managed",
        ])
        .current_dir(source_dir)
        .envs(env)
        .status()
        .into_diagnostic()?;

    if !status.success() {
        return Err(miette::miette!("setup.py install failed for {}", package.name));
    }

    Ok(())
}

/// Build ament_python package using setup.py develop (symlink mode)
fn build_ament_python_develop(
    workspace: &Workspace,
    package: &devros_core::package::Package,
    build_dir: &Utf8PathBuf,
    install_dir: &Utf8PathBuf,
    source_dir: &Utf8Path,
    env: &std::collections::HashMap<String, String>,
) -> Result<()> {
    let _ = workspace; // unused but kept for consistency

    // Create symlinks from source to build directory
    create_python_build_symlinks(source_dir, build_dir)?;

    // Run develop command from build directory
    let build_base = build_dir.join("build");

    let status = Command::new("python3")
        .args([
            "-W", "ignore:setup.py install is deprecated",
            "-W", "ignore:easy_install command is deprecated",
            "setup.py",
            "develop",
            "--prefix", install_dir.as_str(),
            "--build-directory", build_base.as_str(),
            "--editable",
            "--no-deps",
        ])
        .current_dir(build_dir)
        .envs(env)
        .status()
        .into_diagnostic()?;

    if !status.success() {
        return Err(miette::miette!("setup.py develop failed for {}", package.name));
    }

    Ok(())
}

/// Create symlinks from source directory to build directory for Python develop mode
fn create_python_build_symlinks(source_dir: &Utf8Path, build_dir: &Utf8PathBuf) -> Result<()> {
    // Files to symlink
    let files_to_symlink = ["setup.py", "setup.cfg", "package.xml"];

    for file in &files_to_symlink {
        let src = source_dir.join(file);
        let dst = build_dir.join(file);

        if src.exists() {
            // Remove existing symlink/file if it exists
            if dst.exists() || dst.is_symlink() {
                std::fs::remove_file(&dst).into_diagnostic()?;
            }
            std::os::unix::fs::symlink(&src, &dst).into_diagnostic()?;
        }
    }

    // Symlink resource directory if it exists
    let resource_src = source_dir.join("resource");
    let resource_dst = build_dir.join("resource");
    if resource_src.exists() {
        if resource_dst.exists() || resource_dst.is_symlink() {
            if resource_dst.is_symlink() {
                std::fs::remove_file(&resource_dst).into_diagnostic()?;
            } else {
                std::fs::remove_dir_all(&resource_dst).into_diagnostic()?;
            }
        }
        std::os::unix::fs::symlink(&resource_src, &resource_dst).into_diagnostic()?;
    }

    // Symlink Python package directories
    // Look for directories that could be Python packages (contain __init__.py)
    for entry in std::fs::read_dir(source_dir).into_diagnostic()? {
        let entry = entry.into_diagnostic()?;
        let path = entry.path();

        if path.is_dir() {
            let dir_name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");

            // Skip hidden directories and common non-package directories
            if dir_name.starts_with('.') || dir_name == "build" || dir_name == "resource" {
                continue;
            }

            // Check if it's a Python package (has __init__.py)
            if path.join("__init__.py").exists() {
                let src = Utf8PathBuf::try_from(path.clone()).into_diagnostic()?;
                let dst = build_dir.join(dir_name);

                if dst.exists() || dst.is_symlink() {
                    if dst.is_symlink() {
                        std::fs::remove_file(&dst).into_diagnostic()?;
                    } else {
                        std::fs::remove_dir_all(&dst).into_diagnostic()?;
                    }
                }
                std::os::unix::fs::symlink(&src, &dst).into_diagnostic()?;
            }
        }
    }

    Ok(())
}

/// Generate sitecustomize.py content to redirect Python installation prefix
fn generate_sitecustomize(install_dir: &Utf8PathBuf) -> Result<String> {
    Ok(format!(
        r#"# Generated by devros - redirects Python installation to install directory
import sys
sys.prefix = '{install_dir}'
sys.exec_prefix = '{install_dir}'
"#,
        install_dir = install_dir
    ))
}

/// Ensure package.xml is installed (copied or symlinked)
fn ensure_package_xml_installed(
    package: &devros_core::package::Package,
    install_dir: &Utf8PathBuf,
    symlink: bool,
) -> Result<()> {
    let share_dir = install_dir.join("share").join(&package.name);
    std::fs::create_dir_all(&share_dir).into_diagnostic()?;

    let src = package.path.join("package.xml");
    let dst = share_dir.join("package.xml");

    if src.exists() && !dst.exists() {
        if symlink {
            std::os::unix::fs::symlink(&src, &dst).into_diagnostic()?;
        } else {
            std::fs::copy(&src, &dst).into_diagnostic()?;
        }
    }

    Ok(())
}

/// Ensure ament resource index entry exists
fn ensure_ament_resource_index(
    package: &devros_core::package::Package,
    install_dir: &Utf8PathBuf,
) -> Result<()> {
    let index_dir = install_dir
        .join("share")
        .join("ament_index")
        .join("resource_index")
        .join("packages");
    std::fs::create_dir_all(&index_dir).into_diagnostic()?;

    let marker_path = index_dir.join(&package.name);
    if !marker_path.exists() {
        std::fs::write(&marker_path, "").into_diagnostic()?;
    }

    Ok(())
}

/// Generate Python package environment files (package.dsv and local_setup.sh)
fn generate_python_package_environment_files(
    workspace: &Workspace,
    package: &devros_core::package::Package,
) -> Result<()> {
    let install_dir = workspace.package_install_dir(&package.name);
    let share_dir = install_dir.join("share").join(&package.name);

    std::fs::create_dir_all(&share_dir).into_diagnostic()?;

    // Generate package.dsv with Python-specific paths
    let dsv_content = generate_python_package_dsv(&package.name)?;
    std::fs::write(share_dir.join("package.dsv"), dsv_content).into_diagnostic()?;

    // Generate local_setup.sh
    let setup_content = devros_core::dsv::generate_local_setup_sh(&package.name);
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

/// Generate package.dsv content for Python packages
fn generate_python_package_dsv(package_name: &str) -> Result<String> {
    let python_lib_dir = get_python_lib_dir()?;

    let lines = vec![
        // AMENT_PREFIX_PATH
        "prepend-non-duplicate;AMENT_PREFIX_PATH;".to_string(),
        // PATH for bin directory
        "prepend-non-duplicate-if-exists;PATH;bin".to_string(),
        // PYTHONPATH for the Python lib directory
        format!("prepend-non-duplicate-if-exists;PYTHONPATH;{}", python_lib_dir),
        // AMENT_CURRENT_PREFIX
        "set;AMENT_CURRENT_PREFIX;".to_string(),
    ];

    let _ = package_name; // unused but kept for potential future use
    Ok(lines.join("\n"))
}
