from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name="state_estimator_container",
        namespace="vehicle",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="state_estimator",
                plugin="LoadTransferNode",
                name="load_transfer_node",
                namespace="vehicle",
                parameters=[
                    {"vehicle_mass": 2500.0},
                    {"cog_to_rear": 2.0},
                    {"cog_to_front": 0.5},
                    {"width_rear": 2.7},
                    {"width_front": 2.0},
                    {"height_cog": 1.0},
                    {"update_rate": 100.0},
                ]
            )
        ]
    )

    return LaunchDescription([container])