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
                {"update_rate": 100.0},
                {"P_omega": 100.0},
                {"P_delta": 100.0},
            ],
        )
        for wheel in wheels
    ]
    vehicle_controller_node = Node(
        package="control",
        executable="vehicle_controller_node",
        name="controller",
        namespace="vehicle",
        parameters=[
            {"update_rate": 50.0},
        ],
    )
    return LaunchDescription([
        *wheel_controller_nodes,
        vehicle_controller_node
    ])