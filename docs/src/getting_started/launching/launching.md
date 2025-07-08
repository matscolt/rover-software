# Launching the Software
In order to launch the rover software, you need to follow these steps:


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
