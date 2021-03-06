import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point

import xacro

from numpy import pi, cos, sin


def spawn(pos, yaw, args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("vehicle_spawner_node")
    client = node.create_client(SpawnEntity, "spawn_entity")
    modelfile = "/home/cale/thesis-code/ws/src/simulator/models/simple_car/model.urdf.xacro"
    node.get_logger().info("parsing [" + modelfile + "]")
    urdf = xacro.process_file(modelfile).toxml()

    request = SpawnEntity.Request()
    request.name = "vehicle"
    request.xml = urdf
    request.robot_namespace = "vehicle"
    request.initial_pose.position.x = pos[0]
    request.initial_pose.position.y = pos[1]
    request.initial_pose.position.z = pos[2]

    request.initial_pose.orientation.w = cos(yaw/2)
    request.initial_pose.orientation.x = sin(yaw/2)*cos(pi/2)
    request.initial_pose.orientation.y = sin(yaw/2)*cos(pi/2)
    request.initial_pose.orientation.z = sin(yaw/2)*cos(0)

    request.reference_frame = "map"

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("service not availabe, waiting again")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    spawn((1.0, 0.0, 1.15975), 0.0)
    #spawn((1.0, 0.0, 1.598), 0.0)


if __name__ == "__main__":
    main()
