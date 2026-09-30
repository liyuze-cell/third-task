from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hik_camera_ros2',
            executable='hik_camera_node',
            name='hik_camera',
            output='screen',
            parameters=['config/hik_camera_params.yaml']
        )
    ])