import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cam_config = os.path.join(
        get_package_share_directory('vh_dronekit'),
        'config',
        'camera_params.yaml'
    )
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='vh_image',
            namespace='',
            executable='vh_image',
            arguments=['--ros-args', '--log-level', 'info'],
            output='log'
        ),
        launch_ros.actions.Node(
            package='usb_cam',
            namespace='',
            executable='usb_cam_node_exe',
            arguments=['--ros-args', '--log-level', 'info'],
            output='log',
            parameters=[
                cam_config
            ]
        ),
    ])