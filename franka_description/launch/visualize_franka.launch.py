#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    load_gripper_parameter_name = 'load_gripper'
    load_fts_parameter_name = 'load_fts'
    fts_sensor_link_name_parameter = 'fts_sensor_link_name'
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    load_fts = LaunchConfiguration(load_fts_parameter_name)
    fts_sensor_link_name = LaunchConfiguration(fts_sensor_link_name_parameter)

    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots', 'real',
                                     'panda_arm.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper, ' fts:=', load_fts, ' fts_link_name:=', fts_sensor_link_name])

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as end-effector if true. Robot is loaded without '
                        'end-effector otherwise'),
        DeclareLaunchArgument(
            load_fts_parameter_name,
            default_value='true',
            description='Use fts if true. Robot is loaded without fts otherwise'),
        DeclareLaunchArgument(
            fts_sensor_link_name_parameter,
            default_value='bota_ft_sensor',
            description='fts sensor link name'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            # parameters=[{'robot_description': robot_description}],
            parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file])
    ])
