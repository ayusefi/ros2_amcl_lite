#!/usr/bin/env python3
"""
Research Experiment Launch File for AMCL Comparison
This launch file enables research data logging for systematic comparison
between baseline and dynamic-aware AMCL implementations.
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments for experiment configuration
    scenario_arg = DeclareLaunchArgument(
        'scenario_name',
        default_value='static_environment',
        description='Name of the test scenario (static_environment, dynamic_environment, kidnapped_robot)'
    )
    
    algorithm_arg = DeclareLaunchArgument(
        'algorithm_name',
        default_value='amcl_lite_dynamic',
        description='Algorithm variant (amcl_lite_baseline, amcl_lite_dynamic)'
    )
    
    trial_arg = DeclareLaunchArgument(
        'trial_number',
        default_value='1',
        description='Trial number for this experiment run'
    )
    
    enable_dynamic_arg = DeclareLaunchArgument(
        'enable_dynamic_detection',
        default_value='true',
        description='Enable dynamic object detection (false for baseline)'
    )
    
    # Static transform publisher for map -> odom (for visualization)
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # AMCL Lite node with research logging enabled
    amcl_lite_node = Node(
        package='ros2_amcl_lite',
        executable='amcl_lite_node',
        name='amcl_lite_node',
        parameters=[
            {
                'max_particles': 1000,
                'min_particles': 100,
                'enable_dynamic_detection': LaunchConfiguration('enable_dynamic_detection'),
                'dynamic_detection_threshold': 0.5,
                'dynamic_weight': 0.7,
                'sensor_sigma': 0.15,
                'set_initial_pose': True,
                'initial_pose_x': 0.0,
                'initial_pose_y': 0.0,
                'initial_pose_a': 0.0,
                # Research logging parameters
                'enable_research_logging': True,
                'scenario_name': LaunchConfiguration('scenario_name'),
                'algorithm_name': LaunchConfiguration('algorithm_name'),
                'trial_number': LaunchConfiguration('trial_number'),
            }
        ],
        output='screen'
    )
    
    # A* Planner node
    astar_planner = Node(
        package='a_star_planner',
        executable='a_star_planner_node',
        name='a_star_planner',
        output='screen'
    )
    
    # Pure Pursuit Controller
    pure_pursuit_controller = Node(
        package='ros2_pure_pursuit_controller',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        parameters=[
            {
                'lookahead_distance': 1.0,
                'max_linear_velocity': 0.5,
                'max_angular_velocity': 1.0,
                'distance_tolerance': 0.2,
                'angle_tolerance': 0.1,
                'cmd_vel_topic': '/bcr_bot/cmd_vel',
                'odom_topic': '/bcr_bot/odom',
                'path_topic': '/path',
                'pose_topic': '/amcl_lite_pose'
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        scenario_arg,
        algorithm_arg,
        trial_arg,
        enable_dynamic_arg,
        static_transform_publisher,
        amcl_lite_node,
        astar_planner,
        pure_pursuit_controller,
    ])
