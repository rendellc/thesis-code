from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # must match namespace of wheel plugin
    wheels = ["wheel_fl", "wheel_rl", "wheel_rr", "wheel_fr"]
    wheel_controller_nodes = [
        Node(
            package="control",
            executable="wheel_controller_node",
            name="controller",
            namespace="vehicle/" + wheel,
            parameters=[
                {"update_rate": 60.0},
                {"P_omega": 100.0},
                {"P_delta": 100.0},
            ],
        )
        for wheel in wheels
    ]
    return LaunchDescription(
        wheel_controller_nodes
    )