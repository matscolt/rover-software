from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gorm_base_control',
            executable='ackermann_node',
            name='ackermann_node'
        ),
    ])
