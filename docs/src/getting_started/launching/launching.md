# Launching the GORM Rover



## Quick Start with Docker

1. **Navigate to the Rover Software Directory**: Open a terminal and navigate to the directory where the rover software is located.
    ```bash
    cd /workspace/rover-software/docker
    ```
2. **Run the Docker Container and Attach to It**: Start the Docker container that contains the rover software and attach to it.
    ```bash
    ./run.sh
    ./attach.sh
    ```
3. **Build the ROS2 Packages**: Inside the Docker container, build the ROS2 packages to ensure everything is set up correctly.
    ```bash
    colcon build
    source install/setup.bash
    ```
4. **Launch the Rover Software**: Finally, launch the rover software using the provided launch file.
    ```bash
    ros2 launch gorm_bringup bringup_teleop.launch.py
    ```

## Launching the GORM Rover
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
