from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy"
        ),
        Node(
            package="controller_receiver",
            executable="controller_receiver",
            name="controller_receiver"
        ),
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
        )
    ])
