from setuptools import find_packages, setup

package_name = 'ackermann_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),

    ('share/' + package_name, ['package.xml']),

    ('share/' + package_name + '/urdf',
     ['urdf/ackermann_robot.xacro']),

    ('share/' + package_name + '/meshes',[
        'meshes/chassis_visual.stl',
        'meshes/wheel_visual.stl',
        'meshes/chassis_visual.dae',
        'meshes/wheel_visual.dae'
    ]),

    ('share/' + package_name + '/worlds',
     ['worlds/ackermann_world.sdf']),

    ('share/' + package_name + '/config',
     ['config/bridges.yaml']),

    ('share/' + package_name + '/launch',
     ['launch/ackermann_gazebo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohana',
    maintainer_email='dmmiitkgp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
