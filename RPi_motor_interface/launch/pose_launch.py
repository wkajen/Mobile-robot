#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_interface',
            executable='pose_publisher_node',
            name='pose_publisher',
            output='screen'
        )
    ])
