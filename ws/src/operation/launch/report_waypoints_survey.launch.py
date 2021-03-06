from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition


from report_utils.launch_parameters import WP_SURVEY


def generate_launch_description():
    container_survey = ComposableNodeContainer(
        name="operation_container",
        namespace="vehicle",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="operation",
                plugin="WaypointPublisherNode",
                name="waypoint_publisher",
                namespace="vehicle",
                parameters=[
                    {"waypoint_xs": list(WP_SURVEY[:, 0])},
                    {"waypoint_ys": list(WP_SURVEY[:, 1])}
                ],
            )
        ]
    )

    return LaunchDescription([container_survey])
