#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_your_package = get_package_share_directory('motor_interface')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_your_package, 'launch', 'slam_launch.py')
        )
    )

    initial_pose = TimerAction(
        period=3.0,  # Czekamy na uruchomienie SLAM-u
        actions=[
            Node(
                package='motor_interface',
                executable='initial_pose_publisher',
                name='initial_pose_publisher',
                output='screen'
            )
        ]
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py']
        )
    )

    return LaunchDescription([
        slam_launch,
        initial_pose,
        nav2_bringup
    ])