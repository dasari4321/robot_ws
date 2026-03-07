#!/usr/bin/env python3

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")
    lifecycle_nodes = ["map_server", "amcl"]

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value=os.path.join(
            get_package_share_directory("ackermann_robot"),
            "maps",
            "parking_F1.yaml",
        ),
        description="Full path to map yaml file to load"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("ackermann_robot"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to amcl yaml file to load"
    )

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_name},
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time},
            # Increase transform tolerance to avoid startup race conditions
            # where AMCL starts before the odom->base_link transform is available.
            {"transform_tolerance": 1.0},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        amcl_config_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,
    ])


#ros2 launch slam_toolbox online_async_launch.py slam_params_file:=install/ackermann_robot/share/ackermann_robot/config/mapper_params_online_async.yaml use_sim_time:=True


#ros2 run nav2_map_server map_saver_cli -f parking_F1
