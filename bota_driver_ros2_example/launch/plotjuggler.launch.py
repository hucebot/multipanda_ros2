import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for launching PlotJuggler.
    
    This launch file sets up:
    - PlotJuggler node with a predefined layout
    """

     # ============================================================================
    # PLOTJUGGLER NODE
    # ============================================================================

    # Launch the plotjuggler node
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        output='screen',
        arguments=[
            '-l', os.path.join(
                get_package_share_directory('bota_driver_example'),
                'plotjuggler',
                'layout.xml'
            )
        ]
    )

    # ============================================================================
    # LAUNCH DESCRIPTION
    # ============================================================================

    nodes_to_start = [
        plotjuggler_node
    ]

    return LaunchDescription(nodes_to_start)
