from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    container = ComposableNodeContainer(
        name="controller_container",
        namespace="vehicle",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="control",
                plugin="VehicleControllerNode",
                parameters=[
                    {"update_rate": 50.0}
                ]
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="wheel_fl",
                parameters=[
                    {"update_rate": 100.0}
                ]
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="wheel_rl",
                parameters=[
                    {"update_rate": 100.0}
                ]
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="wheel_rr",
                parameters=[
                    {"update_rate": 100.0}
                ]
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="wheel_fr",
                parameters=[
                    {"update_rate": 100.0}
                ]
            ),
        ]
    )

    return LaunchDescription([container])