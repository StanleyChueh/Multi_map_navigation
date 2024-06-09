from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('custom_nav'),
    'config',
    'nav_srv.yaml'
    )
    custom_nav_dir = get_package_share_directory('custom_nav')
    manager = os.path.join(custom_nav_dir, 'custom_nav', 'manager.py')

    return LaunchDescription([
        Node(
            package='custom_nav',
            executable='nav_srv',
            name='custom_nav',
            parameters=[config],
        ),
        ExecuteProcess(
            cmd=['python3', manager],
            output='screen'
        )
    ])
