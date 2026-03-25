#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_ball_collection_dir = get_package_share_directory('turtlebot3_ball_collection')

    # Gazebo world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'spawn_balls': LaunchConfiguration('spawn_balls'),
            'spawn_balls_count': LaunchConfiguration('spawn_balls_count')
        }.items()
    )

    # Robot Localization (EKF) for fusing Odom & IMU
    ekf_config_path = os.path.join(turtlebot3_ball_collection_dir, 'param', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    # Ball collection pipeline
    ball_collection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_ball_collection_dir, 'launch', 'ball_collection.launch.py')
        )
    )

    # SLAM + Navigation launch /to alternative pre-built map navigation
    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_ball_collection_dir, 'launch', 'slam_navigation.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(turtlebot3_gazebo_dir, 'worlds', 'tennis_court.world'),
            description='Gazebo world file to load'
        ),
        DeclareLaunchArgument(
            'spawn_balls',
            default_value='true',
            description='Whether to spawn random tennis balls'
        ),
        DeclareLaunchArgument(
            'spawn_balls_count',
            default_value='20',
            description='Number of tennis balls to spawn at startup'
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Use SLAM Toolbox for mapping instead of pre-built map'
        ),

        gazebo_launch,
        ekf_node,
        ball_collection_launch,
        slam_nav_launch
    ])
