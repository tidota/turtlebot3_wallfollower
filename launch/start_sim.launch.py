#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_bt = LaunchConfiguration('use_bt')

    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    rviz_config_path = os.path.join(
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
            default_value='true'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, \
                             'launch', 'turtlebot3_house.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(  # Behavior Tree version of Wallfollower
            condition=IfCondition(
                PythonExpression(['"', use_bt, '".lower() == "true"'])
            ),
            package='turtlebot3_wallfollower',
            executable='wallfollower_bt',
            name='wallfollower_bt',
            parameters=[
                {'use_sim_time': use_sim_time},
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
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
