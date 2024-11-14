from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    ### Include launch files from other packages e.g. zed_wrapper
    mission_control_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("mission_control"), "/launch/mission_control.launch.py"]),
    )
    
    navigation_stack_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("navigation_stack"), "/launch/navigation_stack.launch.py"]),
    )

    manipulation_stack_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("manipulation_stack"), "/launch/manipulation_stack.launch.py"]),
    )

    perception_stack_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("perception_stack"), "/launch/perception_stack.launch.py"]),
    )

    ld.add_action(mission_control_nodes)
    ld.add_action(navigation_stack_nodes)
    ld.add_action(manipulation_stack_nodes)
    ld.add_action(perception_stack_nodes)

    return ld