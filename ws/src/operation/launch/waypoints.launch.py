from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition


from report_utils.launch_parameters import WP_SIMPLE_LAP, WP_SINGLE_TURN, WP_SURVEY


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("use_survey", default_value="false",
                              description="Use survey"),
        DeclareLaunchArgument("use_single_turn", default_value="false",
                              description="Use single turn"),
        DeclareLaunchArgument("use_simple_lap", default_value="false",
                              description="Use simple lap")
    ]

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
        ],
        condition=IfCondition(LaunchConfiguration("use_survey")) and
        UnlessCondition(LaunchConfiguration("use_simple_lap")) and
        UnlessCondition(LaunchConfiguration("use_single_turn"))
    )

    return LaunchDescription([*arguments, container_survey])
