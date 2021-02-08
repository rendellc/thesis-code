from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def include_launch_file(package, launchfile):
    return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory(package) + "/" + launchfile
            )
    )

def generate_launch_description():
    
    simulator = include_launch_file("simulator", "launch/simulator.launch.py")
    operation = include_launch_file("operation", "launch/manual.launch.py")
    state_estimator = include_launch_file("state_estimator", "launch/state_estimator.launch.py")
    controllers = include_launch_file("control", "launch/controllers.launch.py")
    
    return LaunchDescription([simulator, operation, state_estimator, controllers])