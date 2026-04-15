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
            package="image_tools",
            executable="showimage",
            name="showimage"
        )
    ])