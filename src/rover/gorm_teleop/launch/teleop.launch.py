from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    joy_to_cmd_vel_node = Node(
        package='gorm_teleop',
        executable='joy_to_cmd_vel_node',
        name='joy_to_cmd_vel_node',
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    ld.add_action(joy_to_cmd_vel_node)
    #ld.add_action(joy_node)

    return ld
