from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'config',
        'laser_filter.yaml'
    )

    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar',
            output='screen'
        ),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter_node',
            output='screen',
            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered'),
            ],
            parameters=[config]
        )
    ])
