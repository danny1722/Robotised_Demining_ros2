from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mode_selector",
            executable="mode_selector",
            name="mode_selector"
        ),
        Node(
            package="drive_mode",
            executable="drive_mode",
            name="drive_mode"
        ),
        Node(
            package="dig_mode",
            executable="dig_mode",
            name="dig_mode"
        ),
        Node(
            package="image_tools",
            executable="cam2image",
            name="cam2image"
        )
    ])