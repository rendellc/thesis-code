from threading import Condition
from launch import LaunchDescription
import launch
from launch.substitution import Substitution

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
    run_bag = DeclareLaunchArgument("bag", default_value="true",
                                    description="Do rosbag")
    # bagfile = DeclareLaunchArgument("bagfile", default_value="",
    #                                 description="output of rosbag")

    wp_args = [
        DeclareLaunchArgument("use_manual", default_value="false",
                              description="Use manual mode"),
        DeclareLaunchArgument("use_wp_dev", default_value="false",
                              description="Use waypoints.launch.py"),
        DeclareLaunchArgument("use_survey", default_value="false",
                              description="Use survey"),
        DeclareLaunchArgument("use_single_turn", default_value="false",
                              description="Use single turn"),
        DeclareLaunchArgument("use_simple_lap", default_value="false",
                              description="Use simple lap")
    ]

    simulator = include_launch_file(
        "simulator", "launch/simulator.launch.py", [
            #("gui", IfCondition(LaunchConfiguration("gui"))),
            # ("verbose", "true"),
        ]
    )

   # operation_manual = include_launch_file(
   #     "operation", "launch/manual.launch.py")
    # operation = include_launch_file(

    operations = [
        ExecuteProcess(
            cmd=["ros2", "launch", "operation", "manual.launch.py"],
            condition=IfCondition(LaunchConfiguration("use_manual"))),
        ExecuteProcess(
            cmd=["ros2", "launch", "operation",
                 "waypoints.launch.py"],
            condition=IfCondition(LaunchConfiguration("use_wp_dev"))),
        ExecuteProcess(
            cmd=["ros2", "launch", "operation",
                 "report_waypoints_survey.launch.py"],
            condition=IfCondition(LaunchConfiguration("use_survey"))),
        ExecuteProcess(
            cmd=["ros2", "launch", "operation",
                 "report_waypoints_single_turn.launch.py"],
            condition=IfCondition(LaunchConfiguration("use_single_turn"))),
        ExecuteProcess(
            cmd=["ros2", "launch", "operation",
                 "report_waypoints_simple_lap.launch.py"],
            condition=IfCondition(LaunchConfiguration("use_simple_lap"))),
    ]

    state_estimator = include_launch_file(
        "state_estimator", "launch/state_estimator.launch.py")
    controllers = include_launch_file(
        "control", "launch/controllers_lowcurvature.launch.py")

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
        cmd=["ros2", "bag", "record", "--all", "--output", "bagfolder"],
        condition=IfCondition(LaunchConfiguration("bag"))
    )

    ld = LaunchDescription([
        run_rviz, run_bag, *wp_args,
        simulator,
        *operations,
        state_estimator,
        controllers,
        rviz,
        tf2static,
        spawner_process,
        rosbag_process
    ])

    return ld
