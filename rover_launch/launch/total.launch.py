from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy",
            parameters=[{
                "deadzone": 0.1   # Xbox controller has a deadzone of 0.1, so we set it here to prevent drift
            }]
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
        # DRIVE SYSTEM
        Node(
            package='serial_node',
            executable='serial_node',
            namespace='drive',
            name='serial_node',
            parameters=[{
                'port': '/dev/ttyACM0'
            }]
        ),

        Node(
            package='drive_mode',
            executable='drive_mode',
            namespace='drive',
            name='drive_mode'
        ),
        # DIG SYSTEM
        Node(
            package='serial_node',
            executable='serial_node',
            namespace='dig',
            name='serial_node',
            parameters=[{
                'port': '/dev/ttyACM1'
            }]
        ),

        Node(
            package='dig_mode',
            executable='dig_mode',
            namespace='dig',
            name='dig_mode'
        ),
    ])
