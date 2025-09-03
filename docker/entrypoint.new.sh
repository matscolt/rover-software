#!/bin/bash

# Unified entrypoint script for rover software
# Handles both development and production modes based on environment and arguments

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Setup CAN bus for rover (common for both modes)
setup_can_bus() {
    echo "Setting up CAN bus..."
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
}

# Setup ROS environment and aliases for interactive shells
setup_ros_environment() {
    echo "Setting up ROS environment..."
    
    # Add ROS 2 sourcing to bashrc so it's available in all new shells
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    
    # Source ZED ROS2 workspace if it exists
    if [ -d "/home/ros2_ws/install" ]; then
        echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc
        source /home/ros2_ws/install/setup.bash
    fi
    
    # Source rover workspace if it exists (production mode)
    if [ -d "/home/workspace/install" ]; then
        echo "source /home/workspace/install/setup.bash" >> ~/.bashrc
        source /home/workspace/install/setup.bash
    fi
    
    # Add convenient aliases
    echo "alias bringup='ros2 launch gorm_bringup bringup.launch.py'" >> ~/.bashrc
    echo "alias bringup_teleop='ros2 launch gorm_bringup bringup_teleop.launch.py'" >> ~/.bashrc
    echo "alias camera='ros2 launch gorm_sensors cameras.launch.py'" >> ~/.bashrc
    echo "alias gps='ros2 launch ublox_gps ublox_gps_node_zedf9p-launch.py'" >> ~/.bashrc
    
    # Development mode specific aliases
    if [ ! -d "/home/workspace/install" ]; then
        echo "alias build='cd /home/workspace && colcon build && source install/setup.bash'" >> ~/.bashrc
        echo "alias bringup_teleop='cd /home/workspace && colcon build && source install/setup.bash && ros2 launch gorm_bringup bringup_teleop.launch.py'" >> ~/.bashrc
        echo "alias controller='ros2 launch controller controller.launch.py'" >> ~/.bashrc
    fi
}

# Detect mode based on environment
detect_mode() {
    if [ -d "/home/workspace/install" ]; then
        echo "production"
    else
        echo "development"
    fi
}

# Main execution logic
main() {
    local mode=$(detect_mode)
    echo "Starting rover software in $mode mode..."
    
    # Common setup
    setup_can_bus
    setup_ros_environment
    
    # Handle different execution modes
    case "$1" in
        "autostart")
            echo "Running rover in autostart mode..."
            if [ "$mode" = "production" ]; then
                exec ros2 launch gorm_bringup bringup_teleop.launch.py
            else
                # Development mode: build first, then launch
                cd /home/workspace
                colcon build
                source install/setup.bash
                exec ros2 launch gorm_bringup bringup_teleop.launch.py
            fi
            ;;
        "bringup")
            echo "Running rover bringup..."
            if [ "$mode" = "development" ]; then
                cd /home/workspace
                colcon build
                source install/setup.bash
            fi
            exec ros2 launch gorm_bringup bringup.launch.py
            ;;
        "bringup_teleop")
            echo "Running rover bringup with teleop..."
            if [ "$mode" = "development" ]; then
                cd /home/workspace
                colcon build
                source install/setup.bash
            fi
            exec ros2 launch gorm_bringup bringup_teleop.launch.py
            ;;
        *)
            if [ $# -gt 0 ]; then
                # Execute the provided command
                exec "$@"
            else
                # Default: start an interactive bash shell
                echo "Starting interactive bash shell..."
                echo "Available commands: autostart, bringup, bringup_teleop"
                echo "Available aliases: bringup, bringup_teleop, camera, gps, build (dev mode)"
                exec bash
            fi
            ;;
    esac
}

# Execute main function with all arguments
main "$@"
