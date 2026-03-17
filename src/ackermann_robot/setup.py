"""Set up the ackermann_robot package."""
from setuptools import setup

package_name = 'ackermann_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', [
            'urdf/ackermann_robot.xacro',
            'urdf/all_sensors.xacro',
            'urdf/sensors_cameras.xacro',
            'urdf/sensors_imu.xacro',
            'urdf/sensors_lidar.xacro'
        ]),
        ('share/' + package_name + '/meshes', [
            'meshes/chassis_visual.stl',
            'meshes/front_left_wheel.stl',
            'meshes/front_right_wheel.stl',
            'meshes/rear_left_wheel.stl',
            'meshes/rear_right_wheel.stl',
            'meshes/parking_garage.stl',
            'meshes/parking_garage.dae',
            'meshes/parking_garage_diffuse.jpg',
            'meshes/parking_garage_spec.jpg'
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/multi_level_parking.sdf'
        ]),
        ('share/' + package_name + '/config', [
            'config/amcl.yaml',
            'config/bridges.yaml',
            'config/ekf.yaml',
            'config/mapper_params_online_async.yaml',
            'config/nav2_params.yaml',
            'config/rtab_config.ini'
        ]),
        ('share/' + package_name + '/maps', [
            'maps/parking_F1.pgm',
            'maps/parking_F1.yaml'
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/default.rviz',
            'rviz/nav2_config.rviz'
        ]),
        ('share/' + package_name + '/launch', [
            'launch/ackermann_gazebo.launch.py',
            'launch/navigation.launch.py',
            'launch/rtabmap.launch.py',
            'launch/integrated.launch.py'
        ]),
        ('share/' + package_name + '/behavior_trees', [
            'behavior_trees/navigate_to_pose_w_backup_recovery.xml',
            'behavior_trees/navigate_through_poses_w_backup_recovery.xml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohana',
    maintainer_email='dmmiitkgp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest'
        ],
    },
    entry_points={
        'console_scripts': [
            'baro_converter = ackermann_robot.baro_converter:main',
            'visual_odom = ackermann_robot.visual_odom:main',
            'camera_transform_extractor = ackermann_robot.calibr:main',
            'bev_node = ackermann_robot.bev_node:main',
        ],
    },
)
