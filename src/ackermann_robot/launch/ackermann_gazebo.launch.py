#!/usr/bin/env python3
"""Launch Gazebo Harmonic with the Ackermann robot."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo Harmonic with the Ackermann robot for ROS 2 Jazzy."""
    # Get package directory
    ackermann_robot_dir = get_package_share_directory('ackermann_robot')
    # Config files
    ekf_config_path = os.path.join(ackermann_robot_dir, 'config', 'ekf.yaml')
    rviz_config_path = os.path.join(ackermann_robot_dir, 'rviz', 'default.rviz')

    # SDF world file (Gazebo Harmonic uses SDF format)
    world_file = os.path.join(
        ackermann_robot_dir, 'worlds', 'multi_level_parking.sdf')
    # XACRO file
    xacro_file = os.path.join(
        ackermann_robot_dir, 'urdf', 'ackermann_robot.xacro')

    # Process XACRO to URDF
    urdf_content = xacro.process_file(xacro_file).toxml()

    # Bridge configuration (see config/bridges.yaml)
    bridge_params = {
        'use_sim_time': True,
        'bridge_name': 'ackermann_bridge_config',
        'config_file': os.path.join(
            ackermann_robot_dir, 'config', 'bridges.yaml'),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz if true'
        ),

        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                # ParameterValue(urdf_content, value_type=str),
                'robot_description': urdf_content,
                'use_sim_time': True,
            }],
        ),

        # Start Gazebo Sim (gz_sim) as a Node
        ExecuteProcess(
            cmd=['gz', 'sim', world_file, '-v', '4', '-r'],
            output='screen',
            shell=True  # Allows the system to interpret 'gz' as a command
        ),

        # Spawn the robot using ros_gz
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'ackermann_robot',
                '-topic', 'robot_description',
                '-x', '1.0', '-y', '0.0', '-z', '0.028',
                '-euler', '0.0 0.0 1.5708',
                '-use_sim_time', 'true'
            ],
            output='screen',
        ),

        # Start the ROS-Gazebo bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[bridge_params]
        ),

        # Robot Localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True}]
        ),

        # Barometer Converter
        # Node(
        #     package='ackermann_robot',
        #     executable='baro_converter',
        #     name='baro_converter',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # ),

        # Visual Odometry (ORB-based)
        # Node(
        #     package='ackermann_robot',
        #     executable='visual_odom',
        #     name='visual_odom',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # ),

        # BEV Node
        Node(
            package='ackermann_robot',
            executable='bev_node',
            name='bev_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ])