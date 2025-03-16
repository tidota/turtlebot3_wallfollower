#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    use_bt = LaunchConfiguration('use_bt')

    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    rviz_config_dir = os.path.join(
            get_package_share_directory('turtlebot3_wallfollower'),
            'rviz',
            'default_view.rviz')

    bt_config_path = os.path.join(
            get_package_share_directory('turtlebot3_wallfollower'),
            'config',
            'bt_config.xml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_bt',
            default_value='false'
        ),
        Node(  # Behavior Tree version of Wallfollower
            condition=IfCondition(
                PythonExpression(['"', use_bt, '".lower() == "true"'])
            ),
            package='turtlebot3_wallfollower',
            executable='wallfollower_bt',
            name='wallfollower_bt',
            parameters=[
                {'bt_config_path': bt_config_path}
            ],
            output='screen'
        ),
        Node( # Original version of Wallfollower
            condition=IfCondition(
                PythonExpression(['"', use_bt, '".lower() != "true"'])
            ),
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
