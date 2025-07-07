#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    config_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'config',
        'laser_filter.yaml'  # Make sure this file exists
    )

    return LaunchDescription([

        # Arguments
        DeclareLaunchArgument('channel_type', default_value=channel_type,
                              description='Specifying channel type of lidar'),
        DeclareLaunchArgument('serial_port', default_value=serial_port,
                              description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate,
                              description='Specifying usb port baudrate to connected lidar'),
        DeclareLaunchArgument('frame_id', default_value=frame_id,
                              description='Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted', default_value=inverted,
                              description='Invert scan data'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate,
                              description='Enable angle compensation'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode,
                              description='Scan mode'),

        # SLLidar node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode
            }],
            output='screen'
        ),

        # Filter node (limits scan to 150 degrees)
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter_node',
            remappings=[
                ('scan', '/scan'),  # input from lidar
                ('scan_filtered', '/scan_filtered')  # output
            ],
            parameters=[config_file],
            output='screen'
        )
    ])
