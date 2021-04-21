from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import numpy as np


def generate_launch_description():
    waypoints = np.array([
        [0, 0],
        [10, 0],
        [20, 20],
        [0, 25],
        [-10, 10],
        [-10, 0],
        [0, 0]
    ], dtype=float)

    waypoints_xs = list(waypoints[:, 0])
    waypoints_ys = list(waypoints[:, 1])

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
                    #{"waypoint_xs": [0.0, 10.0, 20.0]},
                    #{"waypoint_ys": [0.0, 5.0, -5.0]}
                    {"waypoint_xs": list(waypoints[:, 0])},
                    {"waypoint_ys": list(waypoints[:, 1])}
                ]
            )
        ]
    )

    return LaunchDescription([container])
