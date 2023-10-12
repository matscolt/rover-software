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
    

    ld.add_action(mission_control_nodes)


    return ld