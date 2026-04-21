# Copyright (c) 2026 HerkuleX ROS2 Driver
# SPDX-License-Identifier: LGPL-2.1-or-later

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('herkulex_driver')

    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for HerkuleX servo communication'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='0101',
        description='HerkuleX servo model: 0101, 0201, 0102, 0401, 0402, 0601, 0602'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'herkulex_params.yaml'),
        description='Path to the parameter file'
    )

    # HerkuleX driver node
    herkulex_node = Node(
        package='herkulex_driver',
        executable='herkulex_node',
        name='herkulex_driver',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'model': LaunchConfiguration('model'),
            }
        ],
        remappings=[],
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        model_arg,
        params_file_arg,
        herkulex_node,
    ])
