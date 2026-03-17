# Ackermann Robot Package

This package provides a complete simulation and navigation setup for an Ackermann steering robot using **ROS 2 Jazzy** and **Gazebo Harmonic**. It features a custom visual odometry implementation, barometer-based altitude estimation, and a tuned Navigation 2 stack using the MPPI controller.

## 🌟 Features

*   **Ackermann Kinematics**: Accurate simulation using Gazebo Harmonic's Ackermann steering system.
*   **Navigation 2 Integration**: Configured with the **MPPI Controller**, specifically tuned for non-holonomic robots to handle steering constraints effectively.
*   **SLAM**: Mapping capabilities using `slam_toolbox`.
*   **Visual Odometry**: Custom ORB-feature-based visual odometry node (`visual_odom.py`) for GPS-denied environments.
*   **Sensor Fusion**: EKF setup fusing IMU, Wheel Odometry, and Visual Odometry.
*   **Barometer Support**: Custom node (`baro_converter.py`) to convert fluid pressure to altitude for 3D localization context.

## 📋 Prerequisites

Ensure you have the following installed on Ubuntu 24.04:

*   **ROS 2 Jazzy**
*   **Gazebo Harmonic**
*   **Dependencies**:
    ```bash
    sudo apt install ros-jazzy-nav2-bringup ros-jazzy-navigation2 \
                     ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
                     ros-jazzy-xacro ros-jazzy-ros-gz \
                     python3-opencv python3-scipy
    ```

## 🛠️ Installation

1.  **Clone the repository**:
    ```bash
    cd ~/robot_ws/src
    # Clone this package here
    ```

2.  **Install dependencies**:
    ```bash
    cd ~/robot_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the package**:
    ```bash
    colcon build --symlink-install --packages-select ackermann_robot
    source install/setup.bash
    ```

## 🚀 Usage

### 1. Simulation & Visualization
Launch the robot in Gazebo along with RViz. This starts the hardware interface, sensor bridges, and the EKF.

```bash
ros2 launch ackermann_robot ackermann_gazebo.launch.py
```
*   **Note**: To enable the custom `visual_odom` or `baro_converter` nodes, uncomment them in `launch/ackermann_gazebo.launch.py`.

### 2. Mapping (SLAM)
To create a map of the environment:

1.  Launch the simulation (as above).
2.  Run SLAM Toolbox:
    ```bash
    ros2 launch slam_toolbox online_async_launch.py \
      slam_params_file:=$(ros2 pkg prefix ackermann_robot)/share/ackermann_robot/config/mapper_params_online_async.yaml \
      use_sim_time:=True
    ```
3.  Drive the robot using the teleop panel in RViz or:
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
4.  **Save the map**:
    ```bash
    ros2 run nav2_map_server map_saver_cli -f ~/my_map
    ```

### 3. Autonomous Navigation
Launch the full Navigation 2 stack with the MPPI controller.

```bash
ros2 launch ackermann_robot navigation.launch.py map_name:=/path/to/your/map.yaml
```

*   **Default Map**: If no map is specified, it defaults to `maps/parking_F1.yaml`.
*   **Operation**: Use the **2D Goal Pose** tool in RViz to send the robot to a destination. The robot handles the Ackermann constraints (cannot turn in place) automatically.

## 📂 Package Structure

### Custom Nodes
*   **`visual_odom.py`**: Monocular Visual Odometry using ORB features and Essential Matrix estimation. It subscribes to camera images and scales translation based on wheel odometry velocity.
*   **`baro_converter.py`**: Converts Gazebo air pressure topics to `PoseWithCovarianceStamped` (Z-axis) for 3D localization.
*   **`calibr.py`**: Utility to extract camera extrinsic matrices from the TF tree.

### Configuration (`config/`)
*   **`nav2_params.yaml`**: Configuration for AMCL, Costmaps, SmacPlanner, and MPPI Controller.
*   **`ekf.yaml`**: Robot Localization settings for fusing sensors.
*   **`bridges.yaml`**: ROS-Gazebo message bridge configuration.

### Launch Files (`launch/`)
*   **`ackermann_gazebo.launch.py`**: Main entry point for simulation.
*   **`navigation.launch.py`**: Brings up the Nav2 stack with the specified map.

## 📝 Notes on Ackermann Tuning
The `nav2_params.yaml` is specifically tuned for this robot:
*   **Min Turning Radius**: Set to `0.18m` in both the Planner and Controller.
*   **MPPI Controller**: Used instead of DWB to handle the kinematic constraints of car-like steering.
