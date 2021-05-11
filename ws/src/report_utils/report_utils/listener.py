import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from vehicle_interface.msg import VehicleControllerInfo
from vehicle_interface.msg import WheelControllerInfo

from collections import deque
import pickle

from builtin_interfaces.msg._time import Time as TimeMsg

import pandas as pd


class Writer:
    def __init__(self, node, listeners, period, output):
        self.node = node
        self.listeners = listeners
        self.output = output

        self.timer = self.node.create_timer(period, self.timer_callback)

    @staticmethod
    def convert_time_msgs(time_msgs):
        start_time_msg = TimeMsg()
        elapsed_time = [0.0]*len(time_msgs)

        return start_time_msg, elapsed_time

    def timer_callback(self):
        for listener in self.listeners:
            # if not listener.data:
            #     continue
            #
            data = {}
            for i, attribute in enumerate(listener.attributes):
                N = len(listener.data)
                data_attribute = [listener.data[j][i] for j in range(N)]
                data[listener.topic + "/" + attribute] = data_attribute

            dataframe = pd.DataFrame.from_dict(data)
            dataframe.to_pickle(self.output + listener.topic.replace("/", "_") +
                                ".pickle")

        # with open(output, "wb") as file:
        #     pickle.dump(data, file)


class Listener:
    def __init__(self, node, topic_type, topic, attributes):
        self.data = deque()
        self.node = node
        self.topic = topic
        self.subscriber = self.node.create_subscription(
            topic_type, topic, self.callback, 1)
        self.attributes = attributes

    def get_nested_attr(self, msg, attribute):
        attribute_list = attribute.split(".")
        value = msg
        for attr in attribute_list:
            value = getattr(value, attr)

        return value

    def callback(self, msg):
        values = [self.get_nested_attr(msg, attr) for attr in self.attributes]
        self.data.append(values)


def main(args=None):
    rclpy.init(args=args)
    node = Node("plotter_node")

    listeners = [
        Listener(node, VehicleControllerInfo,
                 "/vehicle/controller_info", ["header.stamp", "cross_track_error"]),
        Listener(node, WheelControllerInfo,
                 "/vehicle/wheel_fl/controller_info", [
                     "header.stamp", "mode",
                     "robust_rate_error_rate", "robust_rate_angular_rate_reference"])
    ]
    writer = Writer(node, listeners, 10, "results/")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
