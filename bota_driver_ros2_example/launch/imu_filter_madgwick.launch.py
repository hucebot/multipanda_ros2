from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for IMU filter.
    
    This launch file sets up:
    - IMU filter Madgwick node
    """

    # ============================================================================
    # IMU FILTER MADGWICK NODE
    # ============================================================================

    # IMU Filter Madgwick node
    imu_filter_madgwick = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[{
            "use_mag": False,                           # Disable magnetometer usage
            "publish_tf": True,                         # Enable TF publishing 
            "reverse_tf": True,                         # Reverse TF publishing (imu->world)
            "world_frame": "enu",                       # Set the world frame to ENU (East-North-Up)
            "fixed_frame": "world",                     # Set the fixed frame to "world", the one that is fixed in Rviz
        }],
        remappings=[
            ("imu/data_raw", "bota_ft_sensor/imu"),               # Subcribed topic (published by the bota_driver_node)
            ("imu/data", "bota_imu_orientation"),       # Published topic, with orientation field populated
        ]
    )

    # ============================================================================
    # LAUNCH DESCRIPTION
    # ============================================================================

    nodes_to_start = [
        imu_filter_madgwick
    ]

    return LaunchDescription(nodes_to_start)
