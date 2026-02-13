
---

# ROS 2 Jazzy - Parking Simulation Project

This repository contains the simulation environment and control logic for the **parking** package, running on **Ubuntu 24.04 (Noble Numbat)** and **ROS 2 Jazzy**.

## 1. Prerequisites

Ensure you have Ubuntu 24.04 installed and the ROS 2 Jazzy repositories added to your system.

### Install Dependencies

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install ros-jazzy-desktop \
                 ros-jazzy-ros-gz-sim \
                 ros-jazzy-xacro \
                 python3-colcon-common-extensions -y

```

---

## 2. Workspace Setup

If you haven't created your workspace yet, follow these steps to set up the directory structure.

```bash
# Define your workspace path (optional: add to .bashrc)
export ROS_WS=~/ros2_ws

# Create and navigate to the source folder
mkdir -p $ROS_WS/src
cd $ROS_WS/src

# [Place your 'parking' package folder here]

```

---

## 3. Build & Environment Configuration

You must source the global ROS environment **before** building, and the local install **after** building.

```bash
# 1. Navigate to workspace root
cd $ROS_WS

# 2. Source the global ROS 2 setup
source /opt/ros/jazzy/setup.bash

# 3. Build the specific parking package
colcon build --symlink-install --packages-select parking --event-handlers console_direct+

# 4. Source the local workspace to overlay your package
source install/setup.bash

```

> **Note:** Using `--symlink-install` allows you to modify Python scripts, Xacro files, and launch files without needing to rebuild every time.

---

## 4. Launching the Simulation

To launch the Gazebo Sim environment integrated with ROS 2:

```bash
# Basic launch of Gazebo Sim
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"

```

To launch your specific parking project (assuming you have a launch file):

```bash
ros2 launch parking simulation.launch.py

```

---

## Troubleshooting

* **Command not found (colcon):** Ensure you installed `python3-colcon-common-extensions`.
* **Package not found:** Ensure you ran `source install/setup.bash` *after* a successful build.
* **Gazebo won't open:** Check that your GPU drivers are up to date, as Gazebo Sim (formerly Ignition) relies heavily on OGRE 2.

---

**Would you like me to generate a basic `simulation.launch.py` file to help you get your Xacro robot spawning in Gazebo?**
