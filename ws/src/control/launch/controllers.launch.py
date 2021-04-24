from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pathlib

from numpy import deg2rad, pi


def generate_launch_description():

    vehicle_controller_parameters = [
        {"update_rate": 50.0},
        {"maximum_curvature": 0.25},
        {"P_yaw": 5.0},
        {"I_yaw": 0.0},
        {"D_yaw": 0.0},
        {"P_speed": 2.0},
        {"I_speed": 1.0},
        {"approach_angle": deg2rad(30)},
        # {"P_approach": 2.0}  # NOTE: mclain recommends 1/radius_min = maximum_curvature
        {"P_approach": 2.0},
        {"I_approach": 0.5},
        {"D_approach": 0.0}
    ]
    wheel_controller_parameters = [
        {"update_rate": 100.0},
        {"P_omega": 200.0},
        #{"P_omega": 100.0},
        {"I_omega": 20.0},
        {"wheel_mass": 200.0},
        {"wheel_radius": 0.505},
        {"wheel_width": 0.4},
        {"sliding_mode_eigenvalue": 5.0},
        {"steer_resistance_factor": 2.0},
        {"beta_0": 0.1},
        {"sign_slide_eps": 10.0}
    ]

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
                parameters=vehicle_controller_parameters,
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_fl",
                name="controller",
                parameters=wheel_controller_parameters
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_rl",
                name="controller",
                parameters=wheel_controller_parameters
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_rr",
                name="controller",
                parameters=wheel_controller_parameters
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace="vehicle/wheel_fr",
                name="controller",
                parameters=wheel_controller_parameters
            ),
        ]
    )

    return LaunchDescription([container])
