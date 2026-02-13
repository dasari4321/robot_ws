# robot_ws
Hi, I have integrated a VSLAM based navigation using ROS2.

source /opt/ros/jazzy/setup.bash
source /home/mohana/Desktop/robot_ws/install/setup.bash
cd /home/mohana/Desktop/robot_ws
colcon build --symlink-install --event-handlers console_direct+
# then source the install again
source install/setup.bash
# ros2 doctor --report
