# The force-law controller (multi_mode_controller's panda_cartesian_law_controller) on the simulated panda: the scene
# with a wrist force/torque sensor and a block to press on (franka_description/mujoco/franka/scene_law.xml), MuJoCo's
# sensors published by mujoco_ros2_sensors and relayed as the Bota's WrenchStamped (scripts/sim_bota.py), the
# multi-mode controller started with the law (config/sim/single_sim_law.yaml).
#   ros2 launch franka_bringup franka_sim_law.launch.py [headless:=true] [sim_bota:=false]
# sim_bota:=false leaves the relay to someone else (scripts/law_sim_check.py relays it itself, to test a stale force).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description = get_package_share_directory("franka_description")
    bringup = get_package_share_directory("franka_bringup")
    robot_description = Command([
        FindExecutable(name="xacro"), " ",
        os.path.join(description, "robots", "sim", "panda_arm_sim.urdf.xacro"),
        " arm_id:=panda hand:=true initial_positions:=", LaunchConfiguration("initial_positions"),
    ])
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="No MuJoCo window"),
        DeclareLaunchArgument("sim_bota", default_value="true", description="Relay the simulated wrist sensor"),
        DeclareLaunchArgument("initial_positions", default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
                              description="Initial joint positions, in quotes"),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(os.path.join(bringup, "launch", "sim", "launch_mujoco_ros_server.launch")),
            launch_arguments={
                "use_sim_time": "true",
                "modelfile": os.path.join(description, "mujoco", "franka", "scene_law.xml"),
                "ns": "",
                "unpause": "true",
                "headless": LaunchConfiguration("headless"),
                "mujoco_plugin_config": os.path.join(bringup, "config", "sim", "single_sim_law.yaml"),
            }.items(),
        ),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_description}]),
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["joint_state_broadcaster", "-c", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["multi_mode_controller", "-c", "/controller_manager"]),
        ExecuteProcess(cmd=["python3", os.path.join(bringup, "scripts", "sim_bota.py")], output="screen",
                       condition=IfCondition(LaunchConfiguration("sim_bota"))),
    ])
