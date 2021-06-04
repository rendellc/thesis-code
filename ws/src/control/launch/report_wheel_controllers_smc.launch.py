from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


from report_utils.launch_parameters import VEHICLE_CONTROLLER_PARAMETERS, WHEEL_CONTROLLER_PARAMETERS, update_parameter
from report_utils.launch_parameters import GUIDANCE_PARAMETERS


def generate_launch_description():

    vehicle_controller_parameters = VEHICLE_CONTROLLER_PARAMETERS
    vehicle_controller_parameters = update_parameter(
        vehicle_controller_parameters, "disable_all", True)

    wheel_controller_parameters = WHEEL_CONTROLLER_PARAMETERS
    wheel_controller_parameters = update_parameter(wheel_controller_parameters,
                                                   "use_sliding_mode", True)
    wheel_controller_parameters = update_parameter(wheel_controller_parameters,
                                                   "use_robust_rate", False)

    vehicle_namespace = "vehicle"

    container = ComposableNodeContainer(
        name="controllers_container",
        namespace=vehicle_namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="control",
                plugin="GuidanceNode",
                name="guidance",
                namespace=vehicle_namespace,
                parameters=GUIDANCE_PARAMETERS,
            ),
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
