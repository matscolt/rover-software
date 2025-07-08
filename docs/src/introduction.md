<!-- This is the initial pre-release of this RL-suite. Please let me know if there is information missing by contacting me at abmoRobotics@gmail.com. -->

# AAU Rover Software Overview

Welcome to the AAU Rover Software Overview, the rover consists of these main ROS 2 packages:
- **`gorm_bringup`** - Main system launcher with robot state publisher and control nodes
- **`gorm_base_control`** - Ackermann Kinematics interface for 6-wheel, 4-steer kinematics (`/cmd_vel` → `/motor commands`)
- **`gorm_teleop`** - Manual control via joystick (`teleop_twist_joy_node`, `joy-to-cmd_vel converter`)
- **`gorm_sensors`** - Sensor drivers for RGBD cameras, IMU, etc.

## Table of Contents

- [**Installation**](./getting_started/installation/installation.md): Provides a step-by-step guide to install the AAU Rover Software on the Rover.
- [**Launching**](./getting_started/launching/launching.md): Explains how to launch the AAU Rover Software, including building and running the Docker images.

<!-- - [**Examples**](./getting_started/examples/examples.md): Provides examples to help you get started with the AAU Rover Software. -->

