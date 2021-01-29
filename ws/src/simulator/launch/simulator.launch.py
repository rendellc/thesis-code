from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("gazebo_ros") + "/launch/gazebo.launch.py"
            )
    )

    # TODO: don't hardcode path
    modelfile = "/home/cale/thesis-code/ws/src/simulator/models/simple_car/model.urdf"
    
    # spawn_entity = Node(
    #     package="gazebo_ros",
    #     executable="spawn_entity.py",
    #     arguments=["-entity", "vehicle", "-file", modelfile]
    # )

    return LaunchDescription([
        gazebo,
    ])    
    
