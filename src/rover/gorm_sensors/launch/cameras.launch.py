from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    zed_top = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('zed_wrapper'), '/launch/zed2i.launch.py'
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'serial_number': '37915676',
            'zed_id': '0',
            'node_name': 'top_camera'
        }.items()
    )

    zed_tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('zed_wrapper'), '/launch/zed2i.launch.py'
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'serial_number': '35803121',
            'zed_id': '1',
            'node_name': 'tracking'
        }.items()
    )

    camera_pose = Node(
        package='gorm_sensors',
        executable='pose_transform',
        name='camera_pose',
    )

    ld.add_action(zed_top)
    ld.add_action(zed_tracking)
    #ld.add_action(camera_pose)
    return ld
