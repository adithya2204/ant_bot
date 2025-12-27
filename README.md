# Ant Bot: Bio-inspired Hexapod Navigation

![License](https://img.shields.io/badge/license-Apache--2.0-blue)
![ROS 2](https://img.shields.io/badge/ROS_2-Humble-green)

**Ant Bot** is a ROS 2 simulation of a hexapod robot capable of bio-inspired navigation and locomotion. It features a custom gait controller for tripod walking, path integration for homing, and a physics-based Gazebo simulation.

## Features
* **Custom Gait Controller:** Implementation of a tripod gait with specific "soft-start" ramping and directional control.
* **Path Integration:** `ant_navigator` node that performs dead reckoning to return to home (0,0) after foraging.
* **Physics Simulation:** Full URDF/Xacro model with Gazebo plugins for IMU and `ros2_control`.
* **Vibration Fix:** Includes specific logic to handle 360-degree spins without drift/vibration issues.

## Prerequisites
* **ROS 2** (Humble Hawksbill recommended)
* **Gazebo** (Gazebo 11 Classic)
* **Python Dependencies:**
    * `transforms3d`
    * `numpy`

## Installation

1.  **Create a Workspace** (if you haven't already):
    ```bash
    mkdir -p ~/ant_ws/src
    cd ~/ant_ws/src
    ```

2.  **Clone the Repository:**
    ```bash
    git clone [https://github.com/YOUR_USERNAME/ant_bot.git](https://github.com/YOUR_USERNAME/ant_bot.git)
    ```

3.  **Install Dependencies:**
    ```bash
    cd ~/ant_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4.  **Build the Package:**
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

## Usage

### 1. Launch the Simulation
This launches the robot in Gazebo, spawns the controllers, and starts the gait manager.

```bash
ros2 launch ant_bot display.launch.py
