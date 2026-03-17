# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from launch.conditions import IfCondition

# def generate_launch_description():
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     db_path = LaunchConfiguration('db_path')

#     # Path to your custom .ini file
#     # Replace 'ackermann_robot' with your actual ROS 2 package name
#     config_path = os.path.join(
#         get_package_share_directory('ackermann_robot'),
#         'config',
#         'rtab_config.ini'
#     )

#     db_path_arg = DeclareLaunchArgument(
#         'db_path',
#         default_value=os.path.join(os.path.expanduser('~'), '.ros/rtabmap.db'),
#         description='Path to the RTAB-Map database file.'
#     )


#     # Common parameters for both nodes
#     common_params = {
#         'use_sim_time': use_sim_time,
#         'config_path': config_path,
#         'subscribe_depth': True,
#         'subscribe_rgb': True,
#         'approx_sync': True,
# #        'subscribe_odom_info': True,
#         'frame_id': 'base_link',
#     }

#     rtabmap_remaps = [
#         ('rgb/image', '/depth_camera/image'),
#         ('rgb/camera_info', '/depth_camera/camera_info'),
#         ('depth/image', '/depth_camera/depth_image'),
#         ('odom', '/odometry/filtered') # Ensure odom is mapped if not using internal
#     ]

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='true',
#             description='Use simulation clock'
#         ),
#         db_path_arg,
#         DeclareLaunchArgument(
#             'rtabmap_viz',
#             default_value='true',
#             description='Launch RTAB-Map UI (rtabmap_viz)'
#         ),

#         # 1. The SLAM Node
#         Node(
#             package='rtabmap_slam',
#             executable='rtabmap',
#             name='rtabmap',
#             output='screen',
#             parameters=[
#                 common_params,
#                 {'database_path': db_path}
#             ],
#             remappings=rtabmap_remaps,
# #            arguments=['-d'],
#         ),

#         # The Localization Node
#         Node(
#             package='rtabmap_odom',
#             executable='rgbd_odometry',
#             output='screen',
#             parameters=[common_params],
#             remappings=rtabmap_remaps
#             ),

#         # The Visualization Node
#         Node(
#             package='rtabmap_viz',
#             executable='rtabmap_viz',
#             name='rtabmap_viz',
#             output='screen',
#             parameters=[common_params],
#             remappings=rtabmap_remaps,
#             condition=IfCondition(LaunchConfiguration('rtabmap_viz'))
#         ),
#     ])


# # def generate_launch_description():
# #     parameters=[{
# #           'frame_id':'camera_link',
# #           'subscribe_depth':True,
# #           'subscribe_odom_info':True,
# #           'approx_sync':False}]

# #     remappings=[
# #           ('rgb/image', '/camera/color/image_raw'),
# #           ('rgb/camera_info', '/camera/color/camera_info'),
# #           ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

# #     return LaunchDescription([

# #         # Make sure IR emitter is enabled
# #         SetParameter(name='depth_module.emitter_enabled', value=1),

# #         # Launch camera driver
# #         IncludeLaunchDescription(
# #             PythonLaunchDescriptionSource([os.path.join(
# #                 get_package_share_directory('realsense2_camera'), 'launch'),
# #                 '/rs_launch.py']),
# #                 launch_arguments={'align_depth.enable': 'true',
# #                                   'rgb_camera.profile': '640x360x30'}.items(),
# #         ),

# #         Node(
# #             package='rtabmap_odom', executable='rgbd_odometry', output='screen',
# #             parameters=parameters,
# #             remappings=remappings),

# #         Node(
# #             package='rtabmap_slam', executable='rtabmap', output='screen',
# #             parameters=parameters,
# #             remappings=remappings,
# #             arguments=['-d']),

# #         Node(
# #             package='rtabmap_viz', executable='rtabmap_viz', output='screen',
# #             parameters=parameters,
# #             remappings=remappings),
# #     ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    db_path = LaunchConfiguration('db_path')

    # Path to your custom .ini file
    config_path = os.path.join(
        get_package_share_directory('ackermann_robot'),
        'config',
        'rtab_config.ini'
    )

    # 1. Common Parameters for all RTAB-Map components
    # Using 'base_link' ensures the math for the ramp climb is done at the robot center
    common_params = {
        'use_sim_time': use_sim_time,
        'config_path': config_path,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': True,
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'wait_for_transform': 0.2,
    }

    # 2. Remappings for Camera Topics
    # We differentiate 'odom' remapping between the Producer (Odom node) and Consumer (SLAM node)
    camera_remaps = [
        ('rgb/image', '/depth_camera/image'),
        ('rgb/camera_info', '/depth_camera/camera_info'),
        ('depth/image', '/depth_camera/depth_image'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('db_path', default_value=os.path.join(os.path.expanduser('~'), '.ros/rtabmap.db')),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),

        # --- ODOMETRY NODE ---
        # Produces Visual Odometry. Set publish_tf to False because your EKF node 
        # (ekf_filter_node) will handle the odom -> base_link transform.
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[common_params, {
                'publish_tf': False,
                'Odom/Strategy': '0',        # Frame-to-Map is better for ramps
                'Odom/FilteringStrategy': '1' # Median filtering helps with Gazebo jitter
            }],
            remappings=camera_remaps + [('odom', '/vo')]
        ),

        # --- SLAM NODE ---
        # Subscribes to the filtered output of your EKF node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[common_params, {
                'database_path': db_path,
                'RGBD/NeighborLinkRefining': 'true', # Corrects drift on inclines
            }],
            remappings=camera_remaps + [('odom', '/odometry/filtered')],
            arguments=['-d']  # Deletes previous database to start fresh on the ramp
        ),

        # --- VISUALIZATION NODE ---
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[common_params],
            remappings=camera_remaps + [('odom', '/odometry/filtered')],
            condition=IfCondition(LaunchConfiguration('rtabmap_viz'))
        ),
    ])