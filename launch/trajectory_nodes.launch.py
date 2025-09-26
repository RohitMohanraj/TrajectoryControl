#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    scenario_arg = DeclareLaunchArgument('scenario', default_value='s_curve', description='Test scenario s_curve, circle, figure8')
    scenario = LaunchConfiguration('scenario')

    return LaunchDescription([
        scenario_arg,

        Node(
            package='trajectory_controller',
            executable='test_scenarios_node',
            name='test_scenarios',
            parameters=[{'scenario': scenario}],
            output='screen'
        ),
        Node(
            package='trajectory_controller',
            executable='path_smoother_node',
            name='path_smoother',
            output='screen'
        ),
        Node(
            package='trajectory_controller',
            executable='trajectory_generator_node',
            name='trajectory_generator',
            output='screen'
        ),
        Node(
            package='trajectory_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit_controller',
            output='screen'
        ),
    ])
