#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, \
                             'launch', 'turtlebot3_house.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='turtlebot3_wallfollower',
            executable='wallfollower',
            name='wallfollower',
            output='screen'
        ),
    ])
