#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ball_collection_dir = get_package_share_directory('turtlebot3_ball_collection')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(ball_collection_dir, 'param', 'ball_collection.yaml'),
            description='Full path to param file to load'
        ),

        Node(
            package='turtlebot3_ball_collection',
            executable='ball_detector_node',
            name='ball_detector',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),

        Node(
            package='turtlebot3_ball_collection',
            executable='clustering_node',
            name='clustering',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),

        Node(
            package='turtlebot3_ball_collection',
            executable='tsp_planner_node',
            name='tsp_planner',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),

        Node(
            package='turtlebot3_ball_collection',
            executable='path_planner_node',
            name='path_planner',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        )
    ])
