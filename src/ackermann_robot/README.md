# Ackermann Robot

This package contains the simulation, hardware description, and navigation configuration for an Ackermann steering robot using ROS 2 Jazzy and Gazebo Harmonic.

## Prerequisites

Ensure you have the following installed:
*   **ROS 2 Jazzy**
*   **Gazebo Harmonic** (`ros-jazzy-ros-gz`)
*   **Navigation 2** (`ros-jazzy-nav2-bringup`)
*   **Robot Localization** (`ros-jazzy-robot-localization`)
*   **Xacro** (`ros-jazzy-xacro`)

## Installation

1.  Clone the repository into your workspace `src` directory:
    ```bash
    cd ~/robot_ws/src
    # Clone your repository here
    ```

2.  Install dependencies:
    ```bash
    cd ~/robot_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  Build the package:
    ```bash
    colcon build --symlink-install --packages-select ackermann_robot
    source install/setup.bash
    ```

## Usage

### 1. Simulation Only
To launch the robot in Gazebo along with RViz to visualize the robot state and sensors (Lidar, Camera, IMU, etc.):

```bash
ros2 launch ackermann_robot ackermann_gazebo.launch.py
```

### 2. Navigation (Nav2)
To launch the simulation combined with the Navigation 2 stack. This starts Gazebo, the EKF (Extended Kalman Filter), and the Nav2 lifecycle nodes (AMCL, Planner, Controller).

```bash
ros2 launch ackermann_robot navigation.launch.py
```

Once launched, you can use the **2D Goal Pose** tool in RViz to send the robot to a destination.

## Configuration Files

*   **URDF/Xacro**: Located in `urdf/`. The main entry point is `ackermann_robot.xacro`.
*   **Gazebo Bridge**: Mappings between ROS 2 topics and Gazebo topics are defined in `config/bridges.yaml`.
*   **Navigation**: Nav2 parameters (Costmaps, MPPI/Smac planners) are in `config/nav2_params.yaml`.
*   **Localization**: EKF parameters are in `config/ekf.yaml`.
*   **Maps**: The default map is located at `maps/my_map.yaml`.