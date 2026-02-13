# robot_ws
Hi, I have integrated a VSLAM based navigation using ROS2.

source /opt/ros/<distro>/setup.bash
source /home/mohana/Desktop/robot_ws/install/setup.bash
cd /home/mohana/Desktop/robot_ws
colcon build --symlink-install --event-handlers console_direct+
# then source the install again
source install/setup.bashros2 doctor --report
ros2 pkg list | grep -E "nav2|parking|tb4|rplidar"
ros2 launch nav2_minimal_tb4_sim simulation.launch.py
ros2 launch nav2_bringup tb4_simulation_launch.py
ros2 launch parking_world simulation.launch.py
