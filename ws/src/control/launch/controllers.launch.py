from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pathlib


def generate_launch_description():

    vehicle_controller_parameters = [
        {"update_rate": 50.0},
        {"maximum_curvature": 0.25},
        {"P_heading": 5.0},
        {"P_speed": 5.0},
        {"I_speed": 1.0}
    ]
    wheel_controller_parameters = [
        {"update_rate": 100.0}
    ]

    container = ComposableNodeContainer(
        name="controllers_container",
        namespace="vehicle",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="control",
                plugin="VehicleControllerNode",
                name="controller",
                namespace="vehicle",
                parameters=vehicle_controller_parameters,
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_fl",
                name="controller",
                parameters=wheel_controller_parameters
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_rl",
                name="controller",
                parameters=wheel_controller_parameters
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_rr",
                name="controller",
                parameters=wheel_controller_parameters
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_fr",
                name="controller",
                parameters=wheel_controller_parameters
            ),
        ]
    )

    return LaunchDescription([container])
