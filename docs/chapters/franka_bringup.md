# franka_bringup
This package serves as the repository for the launch files that you wlil use to start up the framework.
Broadly speaking, there are currently 2 versions of the launch file that you can use:
- the single-arm `franka.launch.py` and its simulation version, `franka_sim.launch.py`
- the two-arm `dual_franka.launch.py` and its simulation version, `dual_franka_sim.launch.py`.
The launch files are where you define the various configurations for starting up the robot. Several parameters are defined as launch arguments, while others are hard-coded.

## The launch files
### Real robot launch files
The argument for the real robot launch files (i.e. the ones without `sim`) are:
- the robot's name `arm_id` -- this is fixed to `panda` for the single-arm version.
- the robot's IP address `robot_ip`, 
- its gripper configuration `hand`,
- RVIZ visualization toggle, `use_rviz`. 
- Fake hardware utilization, `use_fake_hardware` and `fake_sensor_commands` -- although these are just legacy arguments from the original `franka_ros2` repository; you would now simply use the simulation if you do not have access to a real robot.

Hard-coded values for the real robot launch files are:
- the URDF path, 
- `ros2_control`'s `yaml` configuration file.

### Sim robot launch files
The argument for the MuJoCo simulated robot launch files (i.e. `sim`s) are:
- the robot's name `arm_id` -- this is fixed to `panda` for the single-arm version.
- its gripper configuration `hand`,
- RVIZ visualization toggle, `use_rviz`,

Hard-coded values for the sim robot launch files are:
- the URDF path, typically stored in `franka_description`,
- `mujoco_ros_pkg`'s `yaml` configuration file along with `ros2_control`-specific parameters,
- the `default_scene_xml_file`, which specifies the path to the MuJoCo environment, also stored in `franka_description`. 

For the single-arm version (`franka_sim.launch.py`), the default `scene.xml` will be loaded when no arguments are given, and the `dual_scene.xml` for the dual-arm version.

#### Sim robot's MuJoCo xml
For the sim version, the `arm_id` needs special attention -- this should match the name that you gave to the MuJoCo Panda robot (described in [franka_description](./franka_description.md)). In the `TEMPLATE_panda.xml` file, these names are all marked with `{YOUR_NAME}`. Since MuJoCo does not support parameter passing, this needs to be changed manually. For exact reason for why this is necessary, please see [franka_hardware](./franka_hardware.md).

For example, if you decide to use the name `mobile_arm` as your robot's name, you should replace all instances of `{YOUR_NAME}` in a copy of the template file, and also set the `arm_id` argument as `mobile_arm`. Finally, you need to change the included file in the main `scene.xml` file that you provide as the MuJoCo scene.

#### Ensuring compatibility between URDF and MuJoCo XML
In addition, you need to make sure that the names of the robot joints and actuators in MuJoCo are same as the one given in the URDF files. The `FrankaMjHardware` essentially does string matching between these two robot descriptions, and if the names are not identical, you will get an error. 

#### Where is mujoco_ros_pkg started?
`mujoco_ros_pkg`'s core MuJoCo server is started up by the `FrontendLaunchDescriptionSource` part of the code. Here, a custom `launch_mujoco_ros_server.launch` file is provided in this package, which is identical to the `mujoco_ros_pkg`'s default launch file, except that the node's `name` parameter is removed. This is due to the way node names are handled in ROS2, and setting this parameter will cause all sorts of problems for `ros2_control`. Unfortunately, this issue cannot be addressed in Humble, though the framework works fine without this parameter.

### Structure of the launch files
The different parameters and the way `ros2_control` is loaded are the only major differences between the real and sim launch files. The rest of the code is structured very similarly. Any controller or code that you write for the real robot should work on the sim robot without requiring any additional changes, and vice-versa.

In the real robot, `ros2_control` is started by its own dedicated node, the `franka_control2`. For simulated robot, this is handled by `mujoco_ros_pkg`'s MuJoCo server.

Nodes are launched via the `Node` class in the long `return LaunchDescription` section. The essential nodes that should be launched for all new launch files (specified via the `executable` argument) are:
- `robot_state_publisher` -- this publishes transform data and RVIZ visualization parameters.
- `joint_state_publisher` -- this publishes the robot's joint position data, also needed for RVIZ.

You can also choose to launch the framework with some default `ros2_control` controllers, if they have been defined in the corresponding `yaml` file. The following code would start the `joint_state_broadcaster` controller:
``` python
Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
    output='screen',
)
```
The other launch files in the folder besides the ones already mentioned are essentially different variations of this basic format, with different starting controllers and/or different hard-coded parameters.

### yaml configuration files
The `.yaml` configuration files, found under the `config` folder in the package, manages what controllers are loaded up by `ros2_control` when it is started up. By default, they are in unloaded, unconfigured state.

You define the controllers you wish to add (e.g. `my_controller`) by adding them under this line:
``` yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    cartesian_impedance_example_controller:
      type: franka_example_controllers/CartesianImpedanceExampleController

    ...
    
    my_controller:
      type: some_controller_package/MyController
```
This controller will first need to be compiled and properly configured for it; please see the `franka_example_controller` section in [main](../main.md) for a brief guide.

Parameters for the new controller can then be added under a new root item, like:
``` yaml
controller_manager:
  ros__parameters:
    update_rate: 1000 # Hz
    ...

my_controller:
  ros__parameters:
    arm_id: panda
    ...
```
These will be then available in the controller's functions.

### Simulation yaml configuration files
The simulation' config file is identical to the real robot, except for the fact that an additional block needs to be specified to inform `mujoco_ros_pkg`'s server that the `ros2_control` plugin needs to be loaded up. This block of code at the top of the config file (e.g. in `single_sim_controllers.yaml`) is what does it: 
``` yaml
mujoco_server:
  ros__parameters:
    MujocoPlugins:
      names:
        - mujoco_ros2_control
      mujoco_ros2_control:
        type: mujoco_ros2_control::MujocoRos2ControlPlugin
        hardware: # Not used
          type: mujoco_ros_control/RobotHW
          control_period: 0.001
        params: # Not used
          namespace: ""
          robot_description_node: "robot_state_publisher"
          robot_description: "robot_description"
```
For the most part, you would not need to change anything about this block.

## Other chapters
- [Back to main](../main.md)
- [franka_bringup](./franka_bringup.md)
- [franka_description](./franka_description.md)
- [franka_hardware](./franka_hardware.md)
- [franka_multi_mode_controller](./franka_multi_mode_controller.md)
