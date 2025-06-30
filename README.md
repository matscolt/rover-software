# AAU Space Robotics - System Setup Repository
This repository contains all the files and documentation related to running the system on the physical rover or in simulation.
https://aau-space-robotics.github.io/aau-rover/
## Physical Rover

### Building
1. Run the following code:

```bash
1. cd docker-rover
2. ./build.sh
```

### Running

1. Run the following code:

```bash
1. cd docker-rover
2. ./run.sh --name=CONTAINER_NAME --mode=MODE
```

plaintext

### How to SSH into the Robot

To access the robot via SSH, follow these steps:

1. Open your terminal.

2. Use the following command to SSH into the robot:

```bash
ssh orin@192.168.1.158
```

Enter the password when prompted. The password is 'orin'
   
```bash
./run.sh --name=user
```


4. You will now have access to the root directory of the rover and you can see ros2 topics and furthermore navigate to the files for example here is rover-software. 
```bash
root@ubuntu:/home/orin/ros_ws/src/rover-software#
``

**Note:**

- **CONTAINER_NAME**: Specify the desired name for your container.
- **MODE**: Select the mode of operation for your container. Available options include:
  - **test**: Initiates a temporary container which is removed upon termination, executed with the `--rm` flag.
  - **devel**: Designates the container for development activities.
  - **autostart**: Configures the container to automatically start during the system boot, employing the `--restart always` tag.
  - **autostart_stop**: Disables the auto-start functionality of the container.

## Simulation 

Coming soon ..
