from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='motor_interface',
            executable='motor_interface_node',
            name='motor_interface_node',
            output='screen',
            parameters=[config_path]
        )
    ])
