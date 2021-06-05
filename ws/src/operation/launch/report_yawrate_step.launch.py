from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import numpy as np

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
                plugin="YawrateStepper",
                name="yawrate_stepper",
                namespace="vehicle",
                parameters=[
                    {"steptime": 10.0},
                    {"stepvalue": 0.3}
                ],
            )
        ]
    )

    return LaunchDescription([container])
