#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    rviz_config_dir = os.path.join(
            get_package_share_directory('turtlebot3_wallfollower'),
            'rviz',
            'default_view.rviz')

    return LaunchDescription([
        Node(
            package='turtlebot3_wallfollower',
            executable='wallfollower',
            name='wallfollower',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ])
