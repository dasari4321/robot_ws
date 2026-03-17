#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Integrated launch description for Gazebo, RTAB-Map, and Navigation."""

    # Add a launch argument to switch between SLAM and localization
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Set to "true" to run SLAM, "false" to run localization'
    )
    slam = LaunchConfiguration('slam')

    # Paths to the launch files
    ackermann_gazebo_launch_path = os.path.join(
        get_package_share_directory('ackermann_robot'),
        'launch',
        'ackermann_gazebo.launch.py'
    )
    rtabmap_launch_path = os.path.join(
        get_package_share_directory('ackermann_robot'),
        'launch',
        'rtabmap.launch.py'
    )
    navigation_launch_path = os.path.join(
        get_package_share_directory('ackermann_robot'),
        'launch',
        'navigation.launch.py'
    )

    return LaunchDescription([
        slam_arg,

        # Include ackermann_gazebo launch file (always runs)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ackermann_gazebo_launch_path)
        ),

        # Include rtabmap launch file for SLAM mode
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_path),
            condition=IfCondition(slam)
        ),

        # Include navigation launch file for localization mode
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            condition=UnlessCondition(slam)
        ),
    ])

if __name__ == '__main__':
    import launch
    launch_service = launch.LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()