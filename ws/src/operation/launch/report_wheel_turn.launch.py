from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition


from report_utils.launch_parameters import WP_SIMPLE_LAP


def generate_launch_description():
    container = ComposableNodeContainer(
        name="operation_container",
        namespace="vehicle",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="operation",
                plugin="WheelTurner",
                name="wheel_turner",
                namespace="vehicle",
                parameters=[
                    {"slope": 0.5},
                    {"slopetimeshift": 5.0}
                ],
            )
        ]
    )

    return LaunchDescription([container])