#!/usr/bin/env python3
"""Launch the navigation system."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for navigation with map server and RTAB-Map.
    Returns:
        LaunchDescription: Launch description containing map server, RTAB-Map,
            and lifecycle manager nodes.
    """
    map_name = LaunchConfiguration('map_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    db_path = LaunchConfiguration('db_path')
    rtab_config_path = LaunchConfiguration('rtab_config_path')
    lifecycle_nodes = ['map_server']

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value=os.path.join(
            get_package_share_directory('ackermann_robot'),
            'maps',
            'parking_F1.yaml',
        ),
        description='Full path to map yaml file to load'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value=os.path.join(os.path.expanduser('~'), '.ros/rtabmap.db'),
        description='Path to the RTAB-Map database file for localization.'
    )

    rtab_config_path_arg = DeclareLaunchArgument(
        'rtab_config_path',
        default_value=os.path.join(
            get_package_share_directory('ackermann_robot'),
            'config',
            'rtab_config.ini'
        ),
        description='Full path to rtabmap config file to load'
    )

    nav2_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_name},
            {'use_sim_time': use_sim_time}
        ],
    )

    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
            'frame_id': 'base_link',
            'config_path': rtab_config_path,
            'database_path': db_path,
            # Localization mode
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
        }],
        remappings=[
            ('rgb/image', '/depth_camera/image'),
            ('rgb/camera_info', '/depth_camera/camera_info'),
            ('depth/image', '/depth_camera/depth_image'),
            ('odom', '/odometry/filtered')
        ],
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'node_names': lifecycle_nodes},
            {'use_sim_time': use_sim_time},
            {'autostart': True}
        ],
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        db_path_arg,
        rtab_config_path_arg,
        nav2_map_server,
        rtabmap_localization,
        nav2_lifecycle_manager,
    ])
