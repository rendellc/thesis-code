import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from vehicle_interface.msg import VehicleControllerInfo

from collections import deque
import pickle


class Writer:
    def __init__(self, node, listeners, period, directory):
        self.node = node
        self.listeners = listeners
        self.directory = directory

        self.timer = self.node.create_timer(period, self.timer_callback)

    def timer_callback(self):
        for listener in self.listeners:
            topic = listener.topic.replace("/", "_")
            filename = self.directory + "/" + listener.node.get_name() + topic + ".pickle"
            with open(filename, "wb") as file:
                pickle.dump(listener.data, file)


class Listener:
    def __init__(self, node, topic_type, topic, attributes):
        self.data = deque()
        self.node = node
        self.topic = topic
        self.subscriber = self.node.create_subscription(
            topic_type, topic, self.callback, 1)
        self.attributes = attributes

    def callback(self, msg):
        values = [getattr(msg, attr) for attr in self.attributes]
        self.data.append(values)


def main(args=None):
    rclpy.init(args=args)
    node = Node("plotter_node")

    listeners = []
    listeners.append(
        Listener(node, VehicleControllerInfo,
                 "/vehicle/controller_info", ["header", "cross_track_error"]),
    )
    writer = Writer(node, listeners, 10, "results")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
