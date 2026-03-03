# bota_driver_ros2 

This repository contains the **bota_driver_ros2** package source code. It is built on top of the Bota **Driver for C++**.

Updated documentation about the **Bota Driver for ROS2** bota driver can be found in [Bota FT Stack documentation](https://code.botasys.com/en/gen_a/layer1/driver/driver_ros2.html).

# About
- **Author**: Bota Systems AG (support@botasys.com) 
- **Contributors**: Raul Cruz-Oliver
- **Date**: 27 October 2025 
- **Place**: Zurich, Switzerland 

# Contents
- **/bota_driver_cpp**: C++ implementation of the Bota Driver
- **/hardware**: *ros2_control* hardware interface and transmission files
- **/include**: Header files for the bota_driver_node and bota_driver_node_lc
- **/msg**: Custom message definitions
- **/resources**: Plugin description for ros2_control
- **/scripts**: Scripts for EtherCAT permission grant
- **/src**: Source files for the bota_driver_node and bota_driver_node_lc
- **/urdf**: URDF files for different sensors
- **CMakeLists.txt**: CMake build configuration
- **LICENSE:** License information
- **package.xml:** ROS2 package configuration
- **README.md:** This readme file

# Package Description

The **bota_driver_ros2** package provides a ROS2 interface for the Bota Driver, enabling seamless integration with ROS2-based robotic systems.

The package includes three variants:

1. **bota_driver_node**: A standard ROS2 node providing topics and services.
2. **bota_driver_lcnode**: A life-cycle managed node for better integration with ROS2 lifecycle management. It provides the same topics and services as the standard node, apart from the lifecycle management features.
3. **ros2_control hardware_interface**: A *ros2_control* hardware interface for integration with the *ros2_control* framework.

The choice between the variant to be used depends on the specific requirements of your application. See the [Bota FT Stack documentation](https://code.botasys.com/en/gen_a/layer1/driver/driver_ros2.html) for more details.

The package also includes custom message definitions (found in /msg) as well as URDF files for different Bota Systems FT sensors (found in /urdf).

# Getting Started

1. Clone this repository into your ROS2 workspace's `src` directory:

```bash
# go to your workspace src folder
cd ~/ros2_ws/src

# clone the repository 
git clone https://gitlab.com/botasys/drivers/bota_driver_ros2.git
```

Example workspace structure after cloning:

```text
~/ros2_ws/
├── src/
│   ├── bota_driver_ros2/       # cloned repository
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── src/
│   │   └── ...
│   └── other_package/
├── build/
├── install/
└── log/
```

2. Build your workspace using `colcon build`.

```bash
# go to your workspace root folder
cd ~/ros2_ws

# build the workspace, by default it only builds the standard nodes
colcon build 
```

If you want to build also the **ros2_control hardware interface**, use the following command:

```bash
# make sure to have the neccesary dependencies installed
sudo apt install ros-<DISTRO>-ros2-control 

# go to your workspace root folder
cd ~/ros2_ws

# build the workspace, indicating to build the hardware interface
colcon build --cmake-args -DBUILD_HARDWARE_INTERFACE=TRUE
```

If you only want to explicitly only build the **ros2_control hardware interface**, you can use:

```bash
# build the workspace, indicating to not build the nodes, but build the hardware interface
colcon build --cmake-args -DBUILD_NODES=FALSE -DBUILD_HARDWARE_INTERFACE=TRUE
```

3. Explore the examples:
- For standard ROS 2, see the [bota_driver_ros2_example](https://gitlab.com/botasys/drivers/bota_driver_ros2_example)
- For ros2_control, see the [bota_driver_ros2_control_example](https://gitlab.com/botasys/drivers/bota_driver_ros2_control_example)

# Special settings 

- If working with **CANopen over EtherCAT (CoE)**, ensure that you have the necessary permissions to access the EtherCAT devices. You can use the the provided script in the `/scripts` directory to set the appropriate permissions.

```bash
# go to the scripts folder
cd ~/ros2_ws/src/bota_driver_ros2/scripts

# run the permission grant script
./set_ethercat_network_capabilities.sh

# you can also remove the capabilities later with
./remove_ethercat_network_capabilities.sh
``` 

This needs to be done every time the package is recompiled.

- If working with **Bota Binary**, the user that runs the application must be part of the dialout group to access the serial port. You can add your user to the dialout group with the following command:

```bash
sudo usermod -aG dialout $USER
```

Log out and log back in for the changes to take effect.

This needs to be done only once, not every time the package is recompiled.

# Updates and version pinning

If you want to update the driver to the latest version, you can pull the latest changes from the repository:

```bash
# go to your workspace src folder
cd ~/ros2_ws/src/bota_driver_ros2

# pull the latest changes
git pull
```

If you want a specific version, you can checkout the desired tag:

```bash
# go to your workspace src folder
cd ~/ros2_ws/src/bota_driver_ros2

# checkout the desired tag
git checkout <tag_name>
```

# License
See [LICENSE](LICENSE) file.