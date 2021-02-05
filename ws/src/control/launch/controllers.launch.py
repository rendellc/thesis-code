from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
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
                parameters=[
                    {"update_rate": 50.0}
                ]
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_fl",
                name="controller",
                parameters=[
                    {"update_rate": 100.0}
                ]
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_rl",
                name="controller",
                parameters=[
                    {"update_rate": 100.0}
                ]
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_rr",
                name="controller",
                parameters=[
                    {"update_rate": 100.0}
                ]
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_fr",
                name="controller",
                parameters=[
                    {"update_rate": 100.0}
                ]
            ),
        ]
    )

    return LaunchDescription([container])