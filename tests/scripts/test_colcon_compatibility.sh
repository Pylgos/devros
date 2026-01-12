#!/bin/bash
# Test script to compare colcon and devros build outputs
# Usage: ./test_colcon_compatibility.sh <workspace_path> <devros_binary>

set -e

WORKSPACE_PATH="${1:-tests/integration_ws}"
DEVROS_BINARY="${2:-target/debug/devros}"

cd "$(dirname "$0")/../.."  # Go to repo root

echo "=== Testing colcon compatibility ==="
echo "Workspace: $WORKSPACE_PATH"
echo "devros binary: $DEVROS_BINARY"

# Source ROS 2
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

# Clean workspace
cd "$WORKSPACE_PATH"
rm -rf build install log .devros colcon_install colcon_build devros_install devros_build

# Build with colcon
echo ""
echo "=== Building with colcon ==="
colcon build --symlink-install
# Copy install directory (follow symlinks) to preserve content
cp -rL install colcon_install_copy
rm -rf install build log

# Build with devros  
echo ""
echo "=== Building with devros ==="
"../../$DEVROS_BINARY" build
# Copy install directory (follow symlinks) to preserve content
cp -rL install devros_install_copy
rm -rf install build log .devros

# Compare package.dsv files
echo ""
echo "=== Comparing package.dsv files ==="

compare_dsv() {
    local pkg=$1
    local colcon_dsv="colcon_install_copy/$pkg/share/$pkg/package.dsv"
    local devros_dsv="devros_install_copy/$pkg/share/$pkg/package.dsv"
    
    echo "--- $pkg ---"
    if [ ! -f "$colcon_dsv" ]; then
        echo "ERROR: colcon package.dsv not found: $colcon_dsv"
        return 1
    fi
    if [ ! -f "$devros_dsv" ]; then
        echo "ERROR: devros package.dsv not found: $devros_dsv"
        return 1
    fi
    
    # Show the difference (ignoring .ps1 files and pythonpath_develop)
    echo "colcon entries (essential):"
    grep -v '\.ps1$' "$colcon_dsv" | grep -v 'pythonpath_develop' | sort
    echo ""
    echo "devros entries:"
    cat "$devros_dsv" | sort
    echo ""
    
    # Extract essential entries from colcon package.dsv for comparison
    # Filtering pipeline explanation:
    #   1. grep -v '\.ps1$'          - Exclude PowerShell scripts (Windows only)
    #   2. grep -v 'pythonpath_develop' - Exclude develop-mode specific hooks
    #   3. grep -v 'cmake_prefix_path'  - Exclude colcon-specific CMake hooks
    #   4. grep '\.dsv$\|\.sh$\|\.bash$\|\.zsh$' - Keep only shell-related entries
    local colcon_essential=$(grep -v '\.ps1$' "$colcon_dsv" | \
                            grep -v 'pythonpath_develop' | \
                            grep -v 'cmake_prefix_path' | \
                            grep '\.dsv$\|\.sh$\|\.bash$\|\.zsh$' | \
                            sort)
    
    # Extract devros entries (keep only shell-related entries for comparison)
    local devros_entries=$(cat "$devros_dsv" | \
                          grep '\.dsv$\|\.sh$\|\.bash$\|\.zsh$' | \
                          sort)
    
    if [ "$colcon_essential" = "$devros_entries" ]; then
        echo "✓ Essential entries match for $pkg"
        return 0
    else
        echo "⚠ Essential entries differ for $pkg"
        echo "Missing in devros:"
        comm -23 <(echo "$colcon_essential") <(echo "$devros_entries") || true
        echo "Extra in devros:"
        comm -13 <(echo "$colcon_essential") <(echo "$devros_entries") || true
        return 1
    fi
}

# Compare each package
RESULT=0
for pkg_dir in colcon_install_copy/*/; do
    pkg=$(basename "$pkg_dir")
    compare_dsv "$pkg" || RESULT=1
    echo ""
done

# Verify executables work
echo "=== Verifying executables ==="
if [ -x "devros_install_copy/example_node/lib/example_node/example_publisher" ]; then
    echo "✓ example_publisher executable exists"
else
    echo "✗ example_publisher executable missing"
    RESULT=1
fi

# Verify hook files exist for example_py
echo ""
echo "=== Verifying hook files for example_py ==="
HOOK_FILES=(
    "devros_install_copy/example_py/share/example_py/hook/ament_prefix_path.dsv"
    "devros_install_copy/example_py/share/example_py/hook/ament_prefix_path.sh"
    "devros_install_copy/example_py/share/example_py/hook/pythonpath.dsv"
    "devros_install_copy/example_py/share/example_py/hook/pythonpath.sh"
)
for f in "${HOOK_FILES[@]}"; do
    if [ -f "$f" ]; then
        echo "✓ $f exists"
    else
        echo "✗ $f missing"
        RESULT=1
    fi
done

# Clean up
rm -rf colcon_install_copy devros_install_copy

if [ $RESULT -eq 0 ]; then
    echo ""
    echo "=== All compatibility tests passed ==="
else
    echo ""
    echo "=== Some compatibility tests failed ==="
fi

exit $RESULT
