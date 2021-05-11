from launch import LaunchDescription

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.launch_context import LaunchContext


from ament_index_python.packages import get_package_share_directory


def include_launch_file(package, launchfile, launch_arguments=[]):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(package) + "/" + launchfile
        ),
        launch_arguments=launch_arguments
    )


def generate_launch_description():
    # run_gui = DeclareLaunchArgument("gui", default_value="false",
    #                                 description="Start gzclient")
    run_rviz = DeclareLaunchArgument("rviz", default_value="true",
                                     description="Start rviz")
    run_bag = DeclareLaunchArgument("bag", default_value="false",
                                    description="Do rosbag")

    simulator = include_launch_file(
        "simulator", "launch/simulator.launch.py", [
            #("gui", IfCondition(LaunchConfiguration("gui"))),
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
        ],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )
    tf2static = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"]
    )

    spawner_process = ExecuteProcess(
        cmd=["python3",
             "/home/cale/thesis-code/ws/src/simulator/simulator/spawn_vehicle.py"],
        output="screen"
    )

    rosbag_process = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-a"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("bag"))
    )

    ld = LaunchDescription([
        run_rviz, run_bag,
        simulator,
        # operation_manual,
        operation,
        state_estimator,
        controllers,
        rviz,
        tf2static,
        spawner_process,
        rosbag_process
    ])

    return ld
