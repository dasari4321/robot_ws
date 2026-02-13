# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    car_dir = get_package_share_directory('parking_car')
    world_dir = get_package_share_directory('parking_world')
    sdf_file = os.path.join(car_dir, 'models', 'marble_husk.sdf')
    world_sdf = os.path.join(world_dir, 'worlds', 'parking_garage', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot state publisher
    params = {'use_sim_time': True, 'robot_description': robot_desc}
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])

    # RViz
    pkg_ros_gz_sim_demos = get_package_share_directory('parking_demos')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
#        arguments=[
#            '-d',
#            os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'robot_description_publisher.rviz')
#        ],

    )

    # Gz - ROS Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            # Joint states (IGN -> ROS2)
            '/world/default/model/marble_husky_sensor_config_1/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            'tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/marble_husky_sensor_config_1/joint/tilt_gimbal_joint/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/marble_husky_sensor_config_1/joint/pan_gimbal_joint/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            'world/default/model/marble_husky_sensor_config_1/link/base_link/sensor/camera_front/camera_info',
        ],
        remappings=[
            ('/world/default/model/marble_husky_sensor_config_1/joint_state', '/joint_states'),
            ('/model/marble_husky_sensor_config_1/pose', '/tf')
        ],
        #output='screen'
    )
#        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image',
#                   '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],

#marble_husky_sensor_config_1::base_link::camera_front] advertised on [world/default/model/marble_husky_sensor_config_1/link/base_link/sensor/camera_front/camera_info]
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(car_dir, 'models', 'marble_husk'))

    # gazebo_server = ExecuteProcess(
    #     cmd=['gz', 'sim', '-v8','-r', '-s', world_sdf],
    #     output='screen',
    # )
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v8 ', world_sdf], 'on_exit_shutdown': 'true'}.items()
    )
    # gazebo_client = ExecuteProcess(
    #     cmd=['gz', 'sim', '-v8', '-g'],
    #     output='screen',
    # )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v8 '}.items()
    )
    # Spawn
    spawn = Node(package='ros_gz_sim', executable='create',
                 parameters=[{
                    'world': 'default',
                    'name': 'marble_husky_sensor_config_1',
                    #'topic': '/robot_description',
                    'file': sdf_file,
                    'allow_renaming': True,  
                    'x': -5.94,
                    'y': 0.0,
                    'z': 5.92,
                    'R': 0.0,
                    'P': 0.0,
                    'Y': 0.0
                    }],
                 output='both')

    return LaunchDescription([
        set_env_vars_resources,
        gzserver,
        robot_state_publisher,
        rviz,
        bridge,
        spawn,
        gzclient
    ])
