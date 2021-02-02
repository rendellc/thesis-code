from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="control",
            executable="wheel_controller_node",
            name="wheel_controller_fl",
            namespace="vehicle",
            remappings=[
                ("state", "wheel_fl/state"),
                ("command", "wheel_fl/command"),
            ]
        )
    ])