from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    localization_stack_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("localization_stack"), "/launch/localization_stack.launch.py"]),
    )
    
    software_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("mission_control"), "/launch/software_rover.launch.py"]),
    )

    hardware_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("hardware"), "/launch/hardware.launch.py"]),
    )

    ld.add_action(localization_stack_nodes)
    ld.add_action(software_nodes)
    ld.add_action(hardware_nodes)

    return ld