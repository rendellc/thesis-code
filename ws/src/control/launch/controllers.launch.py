from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


from numpy import deg2rad, pi


def generate_launch_description():

    vehicle_controller_parameters = [
        {"update_rate": 50.0},
        {"maximum_curvature": 0.75},
        {"pid_active": True},
        {"P_yaw": 2.0},
        {"I_yaw": 0.0},
        {"D_yaw": 0.2},
        {"P_speed": 2.0},
        {"I_speed": 1.0},
        {"approach_angle": deg2rad(30)},
        # {"P_approach": 2.0}  # NOTE: mclain recommends 1/radius_min = maximum_curvature
        {"P_approach": 2.0},
        {"I_approach": 0.5},
        {"D_approach": 0.0},
        {"speed_desired": 2.0},
        {"ilqr_4wis_active": False},
        {"ilqr_cost_x": 1.0},
        {"ilqr_cost_y": 1.0},
        {"ilqr_cost_yaw": 10.0},
        {"ilqr_cost_angular_vel": 0.2},
        {"ilqr_cost_steering_angle": 1.0},
        {"ilqr_trajectory_length": 50},
        {"ilqr_singletrack_active": False}
    ]
    wheel_controller_parameters = [
        {"update_rate": 100.0},
        {"P_omega": 100.0},
        #{"P_omega": 100.0},
        {"I_omega": 10.0},
        {"P_delta": 10.0},
        {"use_robust_rate": True},
        {"robust_rate_softregion": 5.0},
        {"steering_rate_limit": (2*3.14)/2},
        {"max_steering_accel": 0.1},
        {"wheel_mass": 200.0},
        {"wheel_radius": 0.505},
        {"wheel_width": 0.4},
        {"use_sliding_mode": False},
        {"sliding_mode_softregion": 5.0},
        {"sliding_mode_eigenvalue": 5.0},
        {"steer_resistance_factor": 2.0},
        {"beta_0": 0.1},
        {"sign_slide_eps": 10.0}
    ]

    vehicle_namespace = "vehicle"

    container = ComposableNodeContainer(
        name="controllers_container",
        namespace=vehicle_namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="control",
                plugin="VehicleControllerNode",
                name="controller",
                namespace=vehicle_namespace,
                parameters=vehicle_controller_parameters,
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace=vehicle_namespace + "/wheel_fl",
                name="controller",
                parameters=wheel_controller_parameters
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace=vehicle_namespace + "/wheel_rl",
                name="controller",
                parameters=wheel_controller_parameters
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace=vehicle_namespace + "/wheel_rr",
                name="controller",
                parameters=wheel_controller_parameters
            ),
            ComposableNode(
                package="control",
                plugin="WheelControllerNode",
                namespace=vehicle_namespace + "/wheel_fr",
                name="controller",
                parameters=wheel_controller_parameters
            ),
        ]
    )

    return LaunchDescription([container])
