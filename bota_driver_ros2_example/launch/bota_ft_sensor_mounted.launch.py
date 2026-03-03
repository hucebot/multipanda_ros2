import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for Bota FT sensor driver node for a sensor mounted on a robot.
    
    This launch file sets up:
    - Bota driver node with configuration for FT sensor
    - Robot description from URDF/xacro
    - Robot state publisher
    - Optional RViz visualization
    """

    # ============================================================================
    # LAUNCH ARGUMENTS
    # ============================================================================

    # Declare the launch argument for bota_ft_sensor_link_name
    bota_ft_sensor_link_name_arg = DeclareLaunchArgument(
        'bota_ft_sensor_link_name',
        default_value='bota_ft_sensor',
        description='Prefix for the FT sensor link name'
    )

    declared_arguments = [
        bota_ft_sensor_link_name_arg
    ]

    # ============================================================================
    # BOTA DRIVER NODE
    # ============================================================================

    # Bota Driver Node
    bota_driver_node = Node(
        package="bota_driver",
        executable="bota_driver_node",
        output="screen",
        parameters=[{
            'node_name': LaunchConfiguration('bota_ft_sensor_link_name'), # Set node name to match sensor link name
            'config_file': os.path.join(get_package_share_directory('bota_driver_example'),'bota_config','bota_binary.json'),
            'output_rate': 500.0,
        }]
    )

    # ============================================================================
    # ROBOT DESCRIPTION & ROBOT STATE PUBLISHER
    # ============================================================================

    # Get the path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('bota_driver_example'),
        'urdf',
        'bota_ft_sensor_mounted.urdf.xacro'
    )

    # Convert the xacro file to a URDF
    robot_description = {
        'robot_description': Command([
            'xacro ', xacro_file, 
            ' bota_ft_sensor_link_name:=', LaunchConfiguration('bota_ft_sensor_link_name')
        ])
    }

    # Launch the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ============================================================================
    # VISUALIZATION & RVIZ
    # ============================================================================

    # RViz configuration file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("bota_driver_example"), 
        "rviz", 
        "sensor_mounted_visualization.rviz"
    ])


    # Launch the rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=["-d", rviz_config_file],
        output='screen',
    )

    # ============================================================================
    # LAUNCH DESCRIPTION
    # ============================================================================

    nodes_to_start = [
        robot_state_publisher_node,
        bota_driver_node,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start )
