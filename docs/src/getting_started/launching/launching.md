# Launching the GORM Rover



## Quick Start with Docker

### 1. **Connect to the Rover**: SSH into the rover's computer using the following command:
   ```bash
   ssh gorm@192.168.0.2 # or ssh gorm@<ROVER_IP_ADDRESS>
   ```
   The default password is `gorm`.

### 2. Build and Run Container
```bash
cd docker
./run.sh
```
This builds the container and starts it in detached mode with GPU support, device access, and networking.

### 3. Attach to Container
```bash
./attach.sh
```
Opens an interactive bash session inside the `rover-software` container.

### 4. Launch the System
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
