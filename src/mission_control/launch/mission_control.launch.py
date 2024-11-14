from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    joystick_node = Node(
            package='mission_control',
            executable='joystick.py',
            name='joystick_node'
        )
    
    ld.add_action(joystick_node)

    return ld