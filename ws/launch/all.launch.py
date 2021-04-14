from launch import LaunchDescription

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def include_launch_file(package, launchfile, launch_arguments=[]):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(package) + "/" + launchfile
        ),
        launch_arguments=launch_arguments
    )


def generate_launch_description():

    simulator = include_launch_file(
        "simulator", "launch/simulator.launch.py", [
            # ("gui", "true"),
            # ("verbose", "true"),
        ]
    )

    operation_manual = include_launch_file(
        "operation", "launch/manual.launch.py")
    operation = include_launch_file("operation", "launch/waypoints.launch.py")
    state_estimator = include_launch_file(
        "state_estimator", "launch/state_estimator.launch.py")
    controllers = include_launch_file(
        "control", "launch/controllers.launch.py")

    rviz = Node(
        package="rviz2",
        # namespace="/",
        executable="rviz2",
        name="rviz2",
        arguments=[
            # "--display-config " + get_package_share_directory("report_utils") + "/config.rviz"
            "--display-config", "launch/config.rviz"
        ]
    )
    tf2static = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"]
    )

    ld = LaunchDescription([
        simulator,
        # operation_manual,
        operation,
        state_estimator,
        controllers,
        rviz,
        tf2static
    ])

    return ld
