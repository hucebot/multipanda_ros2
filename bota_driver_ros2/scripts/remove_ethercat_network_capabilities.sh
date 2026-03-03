#!/bin/bash
#
# remove_ethercat_network_capabilities.sh
#
# This script removes EtherCAT network capabilities and system library
# configurations that were set up by set_ethercat_network_capabilities.sh
#

# Don't exit on error - continue with other commands to clean up as much as possible
# set -e  # commented out

echo "=== Removing EtherCAT Network Capabilities ==="
echo

# --- Detect ROS 2 distro dynamically ---
ROS_DISTRO=$(ls /opt/ros/ | head -n1)
echo "Found ROS2 distro: ${ROS_DISTRO}"
echo

# --- Determine workspace root robustly ---
find_workspace_root() {
    local current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    # Search upward for workspace markers
    while [ "$current_dir" != "/" ]; do
        # Check for src directory (primary indicator)
        if [ -d "$current_dir/src" ]; then
            # Additional checks for workspace confidence
            if [ -f "$current_dir/src/COLCON_IGNORE" ] || \
               [ -d "$current_dir/build" ] || \
               [ -d "$current_dir/install" ] || \
               find "$current_dir/src" -name "package.xml" -type f 2>/dev/null | grep -q .; then
                echo "$current_dir"
                return 0
            fi
        fi
        
        # Check for other common workspace indicators
        if [ -f "$current_dir/package.xml" ] && [ -d "$current_dir/../src" ]; then
            echo "$(dirname "$current_dir")"
            return 0
        fi
        
        # Move up one directory
        current_dir="$(dirname "$current_dir")"
    done
    
    # Fallback: assume script is in src/*/scripts/ or src/*/*/scripts/
    local script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local fallback_root="$(realpath "${script_dir}/../../..")"
    
    # Verify fallback has workspace structure
    if [ -d "$fallback_root/src" ]; then
        echo "$fallback_root"
        return 0
    fi
    
    # Last resort: try going up one more level
    fallback_root="$(realpath "${script_dir}/../../../..")"
    if [ -d "$fallback_root/src" ]; then
        echo "$fallback_root"
        return 0
    fi
    
    echo "ERROR: Could not find workspace root" >&2
    exit 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Script directory: ${SCRIPT_DIR}"

WORKSPACE_ROOT=$(find_workspace_root)
echo "Workspace root: ${WORKSPACE_ROOT}"

# Note: For removal script, we continue even if workspace isn't built
# since we want to clean up system configurations regardless
if [ ! -d "${WORKSPACE_ROOT}/build" ] && [ ! -d "${WORKSPACE_ROOT}/install" ]; then
    echo "Note: Workspace has not been built yet, but continuing with system cleanup..."
fi

echo

# --- Detect build type (symlink vs regular) ---
detect_build_type() {
    local install_executable="${WORKSPACE_ROOT}/install/bota_driver/lib/bota_driver/bota_driver_node"
    local build_executable="${WORKSPACE_ROOT}/build/bota_driver/bota_driver_node"
    
    if [ -f "${install_executable}" ]; then
        if [ -L "${install_executable}" ]; then
            echo "symlink"
        else
            echo "regular"
        fi
    elif [ -f "${build_executable}" ]; then
        echo "symlink"
    else
        echo "none"
    fi
}

BUILD_TYPE=$(detect_build_type)
echo "Detected build type: ${BUILD_TYPE}"
echo

# --- Remove system library configurations ---
echo "Removing ROS2 library configuration..."
sudo rm -f /etc/ld.so.conf.d/ros2_${ROS_DISTRO}.conf || true
sudo rm -f /etc/ld.so.conf.d/bota_driver.conf || true

echo "Updating library cache..."
sudo ldconfig || true
echo

# --- Remove capabilities from bota_driver binaries ---
if [ "${BUILD_TYPE}" != "none" ]; then
    echo "Removing capabilities from bota driver binaries..."
    
    # Set executable paths based on build type
    if [ "${BUILD_TYPE}" = "symlink" ]; then
        echo "Using build directory paths for symlink install"
        BOTA_DRIVER_NODE="${WORKSPACE_ROOT}/build/bota_driver/bota_driver_node"
        BOTA_DRIVER_LCNODE="${WORKSPACE_ROOT}/build/bota_driver/bota_driver_lcnode"
    else
        echo "Using install directory paths for regular build"
        BOTA_DRIVER_NODE="${WORKSPACE_ROOT}/install/bota_driver/lib/bota_driver/bota_driver_node"
        BOTA_DRIVER_LCNODE="${WORKSPACE_ROOT}/install/bota_driver/lib/bota_driver/bota_driver_lcnode"
    fi
    
    # Remove capabilities from bota_driver_node
    if [ -f "${BOTA_DRIVER_NODE}" ]; then
        sudo setcap -r "${BOTA_DRIVER_NODE}" || true
        echo "Removed capabilities from bota_driver_node at: ${BOTA_DRIVER_NODE}"
    else
        echo "bota_driver_node not found at: ${BOTA_DRIVER_NODE}, skipping"
    fi
    
    # Remove capabilities from bota_driver_lcnode
    if [ -f "${BOTA_DRIVER_LCNODE}" ]; then
        sudo setcap -r "${BOTA_DRIVER_LCNODE}" || true
        echo "Removed capabilities from bota_driver_lcnode at: ${BOTA_DRIVER_LCNODE}"
    else
        echo "bota_driver_lcnode not found at: ${BOTA_DRIVER_LCNODE}, skipping"
    fi
else
    echo "No bota_driver binaries found in workspace, skipping workspace capability removal"
fi

echo

# --- Always try to remove capabilities from ros2_control_node ---
echo "Removing capabilities from ros2_control_node..."
ROS2_CONTROL_NODE="/opt/ros/${ROS_DISTRO}/lib/controller_manager/ros2_control_node"
if [ -f "${ROS2_CONTROL_NODE}" ]; then
    sudo setcap -r "${ROS2_CONTROL_NODE}" || true
    echo "Removed capabilities from ros2_control_node"
else
    echo "ros2_control_node not found at: ${ROS2_CONTROL_NODE}, skipping"
fi

echo
echo "=== EtherCAT Network Capabilities Removal Complete ==="


