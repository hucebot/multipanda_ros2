# Documentation for multipanda_ros2
`multipanda_ros2` is a `ros2_control`-based framework for the popular, now unsupported, Franka Emika Robot (Panda).
It is built on ROS2 Humble, and as a consequence, Ubuntu 22.04. Support for newer versions will be released as they come.
It is capable of controlling an arbitrary number of locally connected FCI-enabled robots accessible via their IP addresses.
The primary features of the framework include:
- ROS2 compatibility (naturally)
- 1khz access to robot data, i.e. the state and model
- All control modes offered by `libfranka`, i.e. torque, joint position/velocity, Cartesian position/velocity (only for real robots), regardless of how many robots are being controlled
- Franka Hand gripper integration
- MuJoCo simulation with identical interfaces to the real robot

In each of the following sections, in-depth details about the various parts of the framework are introduced. The aim of the documentation is to make the package more understandable and easily customizable for your own uses.

## Installation guide
Please see the [main README.md](../README.md) for installation instructions.

## In-depth documentation

### [franka_bringup][bringup]
The `franka_bringup` package contains all the launch files that are used to start the framework.

### [franka_description][description]
The  `franka_description` package is where the robot URDF, `ros2_control` configuration, and MuJoCo `xml` files are stored.

### [franka_hardware][hardware]
The `franka_hardware` package is the core of `multipanda_ros2`. This is where the bulk of the code for interfacing with both the real and simulated robots is. Information on how to use the default Franka Hand grippers is also described here.

### [multimode_controller][mmc]
The `multimode_controller` is a standard `ros2_control` controller, but uses its own controller implementation format, called _controllets_. It is designed to reduce the amount of programming overhead, and to allow extremely fast switching between different controllers within the same control mode.

## Other packages

### franka_control2
A `ros2_control` node copied from the original repo. All the real robot launch files execute this particular version, instead of the one installed using `apt install`, for better transparency. It is identical to the default `ros2_control`'s node.

### franka_example_controllers
A series of example controllers for both one-arm and two-arm cases, some written by Franka Emika, some by me. It is intended to be used as a starting point for writing a new controller.
Please see the example controllers (I recommend `subcriber/cartesian_impedance_controller.cpp/hpp`) on how to structure your own one.

It is split into two categories: a `comless` group, which is the collection of example controllers from Franka that execute a hard-coded trajectory, and has no subscribers. The `subscriber` group, as the name implies, includes a subscriber which you can publish goal messages to.

Generally, you should keep in mind the following checklist:
- Make sure to export your controller as a plugin at the end of the `.cpp` file, like:
``` cpp
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerInterface)
```
- Add the controller to `CMakeLists.txt` under `add_library` section,
- Update the `franka_example_controllers.xml` file with the new controller so that your controller can be found,
- Add your new controller (make sure the names all match) to the `.yaml` config file in `franka_bringup`.

### franka_gripper
Franka Hand action server written by Franka Emika, copied from their repository.

### franka_moveit_config
A package that contains the `moveit` configuration files, along with example launch files.

### franka_msgs
All custom messages/services/actions are included here, except for the ones used by the multi-mode controller.

### franka_robot_state_broadcaster
This implements two broadcasters that publishes the robot model and state data at a lower frequency as a ROS2 topic. It interfaces with `franka_semantic_components`.

### franka_semantic_components
A `SemanticComponentInterface` wrapper for processing the robot model and state data that is updated at 1khz from the main `franka_hardware` package. This provides the interfaces for those data that you can use in your `ros2_control` controllers.



[bringup]: ./chapters/franka_bringup.md
[description]: ./chapters/franka_description.md
[hardware]: ./chapters/franka_hardware.md
[mmc]: ./chapters/franka_multi_mode_controller.md

[libfranka-instructions]: https://frankaemika.github.io/docs/installation_linux.html
[mujoco-instructions]: https://mujoco.readthedocs.io/en/latest/programming/#building-mujoco-from-source
[humble-instructions]: https://docs.ros.org/en/humble/Installation.html
