#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
    """Launch Gazebo Jetty with the Ackermann robot for ROS 2 Jazzy."""
    
    # Get package directory
    ackermann_robot_dir = get_package_share_directory('ackermann_robot')
    
    # SDF world file (Gazebo Jetty uses SDF format)
    world_file = os.path.join(ackermann_robot_dir, 'worlds', 'my_world.sdf')
    
    # XACRO file
    xacro_file = os.path.join(ackermann_robot_dir, 'urdf', 'ackermann_robot.xacro')
    
    # Process XACRO to URDF
    urdf_content = xacro.process_file(xacro_file).toxml()

    # Bridge configuration (see config/bridges.yaml)
    bridge_params = {
        'use_sim_time': True,
        'bridge_name': 'ackermann_bridge_config',
        'config_file': os.path.join(ackermann_robot_dir, 'config', 'bridges.yaml'),
    }

    return LaunchDescription([
        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf_content,   #ParameterValue(urdf_content, value_type=str),
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
                '-x', '-3.0', 'z', '0.036',
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
        )
    ])
