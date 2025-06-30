# Launching the GORM Rover

The GORM rover is a 6-wheel, 4-steer robotic system built with ROS 2. The software is organized into modular packages running in Docker containers.

## System Overview

The rover consists of these main ROS 2 packages:
- **`gorm_bringup`** - Main system launcher with robot state publisher and control nodes
- **`gorm_base_control`** - Critical interface for 6-wheel, 4-steer kinematics (`/cmd_vel` → motor commands + `/odom`)
- **`gorm_teleop`** - Manual control via joystick (`teleop_twist_joy_node`, joy-to-cmd_vel converter)
- **`gorm_sensors`** - Sensor drivers for RGBD cameras, IMU, etc.
- **`hardware`** - Low-level hardware interface and motor control

## Quick Start with Docker

### 1. Build and Run Container
```bash
cd docker
./run.sh
```
This builds the container and starts it in detached mode with GPU support, device access, and networking.

### 2. Attach to Container
```bash
./attach.sh
```
Opens an interactive bash session inside the `rover-software` container.

### 3. Launch the System
Inside the container, choose your launch mode:

**Full system bringup:**
```bash
ros2 launch gorm_bringup bringup.launch.py
```

**Teleop mode (manual control):**
```bash
ros2 launch gorm_bringup bringup_teleop.launch.py
```

**Individual components:**
```bash
# Motors only
ros2 launch gorm_base_control motor_driver.launch.py

# Cameras only  
ros2 launch gorm_sensors cameras.launch.py

# Joystick control only
ros2 launch gorm_teleop teleop.launch.py
```

## Key Topics

- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/odom` - Odometry feedback (nav_msgs/Odometry)  
- `/joy` - Joystick input
- Camera and sensor topics from `gorm_sensors`
