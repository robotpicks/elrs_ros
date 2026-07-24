"""Launch the elrs_driver node on its own.

Publishes /joy from the receiver and (given a /battery source) pushes telemetry back to the
handset. Point a joy-based teleop at /joy downstream, or just inspect /joy to verify your
channel/switch mapping.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('elrs_driver'), 'config', 'elrs_driver.yaml')

    return LaunchDescription([
        Node(
            package='elrs_driver',
            executable='elrs_driver',
            name='elrs_driver',
            parameters=[config],
            output='screen',
        ),
    ])
