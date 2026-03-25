#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ball_collection_dir = get_package_share_directory('turtlebot3_ball_collection')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file')

    # Define the path to your YOLO model. You might want to make this a parameter.
    yolo_model_path = os.path.join(
        os.path.dirname(get_package_share_directory('turtlebot3_vision')), 
        '../../yolo11n.pt'  # Adjust path as necessary relative to your install dir
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(ball_collection_dir, 'param', 'ball_collection.yaml'),
            description='Full path to param file to load'
        ),

        Node(
            package='turtlebot3_vision',
            executable='yolo_detector_node',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'model_path': yolo_model_path,
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='turtlebot3_ball_collection',
            executable='rotate_once.py',
            name='initial_rotator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='turtlebot3_ball_collection',
            executable='density_map_builder_node',
            name='density_map_builder',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.1,
                'width': 100,
                'height': 100,
                'origin_x': -5.0,
                'origin_y': -5.0,
                'gaussian_sigma': 0.5,
                'time_decay_factor': 0.99,
                'navigate_on_peak': True
            }]
        )
    ])

