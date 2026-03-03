#!/bin/bash
#
# setup_ethercat_capabilities.sh
#
# This script configures the system so EtherCAT-based ROS 2 nodes
# (e.g. bota_driver, ros2_control_node) can use raw network sockets
# without needing sudo, while keeping library paths functional
# when capabilities disable LD_LIBRARY_PATH.
 
# Exit on error
set -e
 
echo "=== Setting up EtherCAT Network Capabilities ==="
echo
 
# --- Check dependencies ---
if ! command -v setcap &> /dev/null; then
    echo "setcap not found. Installing libcap2-bin..."
    sudo apt update
    sudo apt install -y libcap2-bin
    echo "libcap2-bin installed successfully"
else
    echo "setcap is available"
fi
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

# --- Check if workspace has been built at least once ---
if [ ! -d "${WORKSPACE_ROOT}/build" ] && [ ! -d "${WORKSPACE_ROOT}/install" ]; then
    echo
    echo "ERROR: Workspace has not been built yet."
    echo "You need to build the workspace first:"
    echo "  cd ${WORKSPACE_ROOT}"
    echo "  colcon build"
    echo "  # or"
    echo "  colcon build --symlink-install"
    echo
    exit 1
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

# --- Set executable paths based on build type ---
if [ "${BUILD_TYPE}" = "symlink" ]; then
    echo "Using build directory paths for symlink install"
    BOTA_DRIVER_NODE="${WORKSPACE_ROOT}/build/bota_driver/bota_driver_node"
    BOTA_DRIVER_LCNODE="${WORKSPACE_ROOT}/build/bota_driver/bota_driver_lcnode"
    BOTA_LIB_PATH="${WORKSPACE_ROOT}/build/bota_driver"
else
    echo "Using install directory paths for regular build"
    BOTA_DRIVER_NODE="${WORKSPACE_ROOT}/install/bota_driver/lib/bota_driver/bota_driver_node"
    BOTA_DRIVER_LCNODE="${WORKSPACE_ROOT}/install/bota_driver/lib/bota_driver/bota_driver_lcnode"
    BOTA_LIB_PATH="${WORKSPACE_ROOT}/install/bota_driver/lib"
fi

# --- Configure system-wide library paths ---
CONF_FILE="/etc/ld.so.conf.d/ros2_${ROS_DISTRO}.conf"
 
echo "Configuring library paths in ${CONF_FILE}..."
sudo bash -c "cat > ${CONF_FILE}" <<EOF
/opt/ros/${ROS_DISTRO}/lib
/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu
/opt/ros/${ROS_DISTRO}/opt/sdformat_vendor/lib
/opt/ros/${ROS_DISTRO}/opt/rviz_ogre_vendor/lib
/opt/ros/${ROS_DISTRO}/opt/gz_math_vendor/lib
/opt/ros/${ROS_DISTRO}/opt/gz_utils_vendor/lib
/opt/ros/${ROS_DISTRO}/opt/gz_tools_vendor/lib
/opt/ros/${ROS_DISTRO}/opt/gz_cmake_vendor/lib
EOF
 
echo "Library paths added:"
cat ${CONF_FILE}
echo
 
echo "Updating library cache..."
sudo ldconfig
echo "Library cache updated successfully"
echo
 
# --- Set network capabilities ---
echo "Setting network capabilities for bota_driver binaries..."
 
# bota_driver_node (we know it exists from the check above)
sudo setcap cap_net_raw+ep "${BOTA_DRIVER_NODE}"
echo "Set capabilities for bota_driver_node at: ${BOTA_DRIVER_NODE}"
 
# bota_driver_lcnode
if [ -f "${BOTA_DRIVER_LCNODE}" ]; then
    sudo setcap cap_net_raw+ep "${BOTA_DRIVER_LCNODE}"
    echo "Set capabilities for bota_driver_lcnode at: ${BOTA_DRIVER_LCNODE}"
else
    echo "bota_driver_lcnode not found at: ${BOTA_DRIVER_LCNODE}, skipping"
fi
 
# ros2_control_node (for hardware interface)
ROS2_CONTROL_NODE="/opt/ros/${ROS_DISTRO}/lib/controller_manager/ros2_control_node"
if [ -f "${ROS2_CONTROL_NODE}" ]; then
    sudo setcap cap_net_raw+ep "${ROS2_CONTROL_NODE}"
    echo "Set capabilities for ros2_control_node"
else
    echo "ros2_control_node not found, skipping"
fi
 
# --- Add bota_driver library path ---
BOTA_CONF_FILE="/etc/ld.so.conf.d/bota_driver.conf"
 
echo
echo "Adding bota_driver library path to system configuration..."
if echo "${BOTA_LIB_PATH}" | sudo tee "${BOTA_CONF_FILE}" > /dev/null; then
    echo "Added bota_driver library path: ${BOTA_LIB_PATH}"
    echo "Updating library cache for bota_driver..."
    if sudo ldconfig; then
        echo "Library cache updated successfully for bota_driver"
    else
        echo "Warning: Failed to update library cache"
    fi
else
    echo "Error: Failed to add bota_driver library path to system configuration"
    exit 1
fi
 
echo
echo "=== EtherCAT Network Capabilities Setup Complete ==="