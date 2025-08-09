from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="rover",
                    executable="drive_node",
                    name="drive_node",
                 ),
            Node(package="camera_module",
                    executable="servo_node",
                    name="servo_node",
                 ),
        ]
    )