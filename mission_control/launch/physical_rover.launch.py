from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    
    joystick_node = Node(
            package='mission_control',
            executable='joystick.py',
            name='joystick_node'
        )
    
    ld.add_action(joystick_node)

    return ld