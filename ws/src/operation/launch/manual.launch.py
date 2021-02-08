from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name="operation_container",
        namespace="vehicle",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="operation",
                plugin="ManualOperationJoyNode",
                name="manual_joy",
                namespace="vehicle",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                namespace="vehicle",
            )
        ]
    )

    return LaunchDescription([container])