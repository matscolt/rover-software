#!/bin/bash

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source ZED ROS2 workspace
source /home/ros2_ws/install/setup.bash

# Source rover workspace
source /home/workspace/install/setup.bash

# Setup CAN bus for rover
busybox devmem 0x0c303000 32 0x0000C400
busybox devmem 0x0c303008 32 0x0000C458
busybox devmem 0x0c303010 32 0x0000C400
busybox devmem 0x0c303018 32 0x0000C458

modprobe can
modprobe can_raw
modprobe mttcan

ip link set down can0
ip link set down can1

ip link set can0 type can bitrate 1000000 
ip link set can1 type can bitrate 1000000 

ip link set up can0
ip link set up can1

# Add ROS 2 sourcing to bashrc for interactive shells
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source /home/workspace/install/setup.bash" >> ~/.bashrc

# Add convenient aliases
echo "alias bringup='ros2 launch gorm_bringup bringup.launch.py'" >> ~/.bashrc
echo "alias bringup_teleop='ros2 launch gorm_bringup bringup_teleop.launch.py'" >> ~/.bashrc
echo "alias camera='ros2 launch gorm_sensors cameras.launch.py'" >> ~/.bashrc
echo "alias gps='ros2 launch ublox_gps ublox_gps_node_zedf9p-launch.py'" >> ~/.bashrc

# Check for autostart mode or specific launch command
if [[ $1 == "autostart" ]]
then
    echo "Running rover in autostart mode..."
    exec ros2 launch gorm_bringup bringup_teleop.launch.py
elif [[ $1 == "bringup" ]]
then
    echo "Running rover bringup..."
    exec ros2 launch gorm_bringup bringup.launch.py
elif [[ $# -gt 0 ]]
then
    # Execute the provided command
    exec "$@"
else
    # Default: start an interactive bash shell
    exec bash
fi
