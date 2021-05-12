from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import numpy as np

from report_utils.plan_parser import plan_file_to_cartesian


def generate_launch_description():
    waypoints = plan_file_to_cartesian(
        "/home/cale/thesis-code/ws/launch/survey_dragvoll_manual.plan")
    waypoints.astype(float)
    # waypoints = np.array([
    #     [0, 0],
    #     [30, 0],
    #     [30, 30]
    #     #[20, 20],
    #     #[0, 25],
    #     #[-10, 10],
    #     #[-10, 0],
    #     #[0, 0]
    # ], dtype=float)

    container = ComposableNodeContainer(
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
                    {"waypoint_xs": list(waypoints[:, 0])},
                    {"waypoint_ys": list(waypoints[:, 1])}
                ]
            )
        ]
    )

    return LaunchDescription([container])
