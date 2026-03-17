# Ackermann Robot

A ROS 2 package for simulating an Ackermann steering robot, complete with sensor models, control configurations, and advanced robotics algorithms like SLAM, Navigation, and Semantic BEV Fusion.

## Features

*   **Detailed Robot Model**: A URDF model of an Ackermann vehicle created with Xacro.
*   **Rich Simulation Environment**: A multi-level parking garage world for Gazebo.
*   **Comprehensive Sensor Suite**:
    *   Multiple cameras (front, rear, left, right)
    *   Depth Camera
    *   IMU
    *   Lidar
*   **Control**: Configured with `ros2_control` and the `ackermann_steering_controller`.
*   **SLAM and Localization**:
    *   Integrated RTAB-Map for robust 3D mapping and localization.
    *   Optimized parameters for challenging environments like ramps.
    *   Uses `robot_localization` (EKF) to fuse wheel odometry, visual odometry, and IMU data.
*   **Navigation**: Full integration with the Nav2 stack for autonomous point-to-point navigation.
*   **Semantic Perception**: A node for multi-camera Bird's-Eye View (BEV) semantic fusion, which projects 2D object detections into a 2D costmap.

## Prerequisites

*   **ROS 2 Humble**
*   **Gazebo**
*   `ros2_control` and `ros2_controllers`
*   `robot_localization`
*   `rtabmap_ros`
*   `nav2_bringup`

## Installation

1.  **Clone the repository** into your workspace `src` directory:
    ```bash
    cd ~/robot_ws/src
    git clone <repository_url> ackermann_robot
    ```

2.  **Install dependencies** using rosdep:
    ```bash
    cd ~/robot_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the workspace**:
    ```bash
    colcon build --packages-select ackermann_robot
    source install/setup.bash
    ```

## Usage

The primary way to use this package is through the `integrated.launch.py` file, which can configure the robot for different modes like SLAM and Navigation.

### 1. Simulation & Manual Control

For basic simulation and testing, you can run the Gazebo launch file directly. This also starts the EKF for sensor fusion and the BEV image generation node.

1.  Launch the simulation environment:
    ```bash
    ros2 launch ackermann_robot ackermann_gazebo.launch.py
    ```
2.  In a new terminal, run the teleop node to control the robot with your keyboard. The command remaps the topic to the one expected by the Ackermann controller.
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/ackermann_steering_controller/cmd_vel_unstamped
    ```

### 2. SLAM (Simultaneous Localization and Mapping)

To create a map of the environment, run the integrated launch file with the `slam` argument set to `true`. This will launch Gazebo and RTAB-Map in mapping mode.

```bash
ros2 launch ackermann_robot integrated.launch.py slam:=true
```

### 3. Controlling the Robot

Once the simulation is running, you can publish velocity commands. If using the standard Twist controller:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## License

BSD-3-Clause (or specify your license here)