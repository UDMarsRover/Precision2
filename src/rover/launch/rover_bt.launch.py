from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="rover",
                 executable="bt_drive",
                 name="bt_drive_node",),
        ]
    )