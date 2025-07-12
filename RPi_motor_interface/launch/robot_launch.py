#!/usr/bin/env python3

import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Ścieżki do plików konfiguracyjnych i URDF
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'params.yaml'
    )

    urdf_path = os.path.join(
        os.path.dirname(__file__), 
        '..', 'urdf', 'robot.urdf.xacro'
    )

    # przetworzenie pliku xacro do URDF
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
        Node(
            package='motor_interface',
            executable='motor_interface_node',
            name='motor_interface_node',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='motor_interface',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]  # przekazanie URDF jako parametr
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            arguments=['0.06', '0', '0.10', '0', '0', '0', 'base_footprint', 'laser'],
            output='screen'
        ),
        Node(
            package='motor_interface',
            executable='aht10_node',
            name='aht10_node',
            output='screen'
        )
    ])
