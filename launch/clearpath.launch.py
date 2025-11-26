from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clearpath_docking',
            executable='clearpath_node',
            name='clearpath_node',
            output='screen',
        )
    ])