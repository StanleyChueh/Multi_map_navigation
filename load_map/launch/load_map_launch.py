from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='load_map',
            executable='call_load_map_node',
            name='call_load_map',
            output='screen',
            # emulate_tty=True,
            parameters=[
                {'map_url_param': '/home/csl/Desktop/test_load_map_ws/src/load_map/maps/map.yaml'}
            ]
        )
    ])