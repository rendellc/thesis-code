import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from vehicle_interface.msg import VehicleControllerInfo
from vehicle_interface.msg import WheelControllerInfo

from report_utils.listener import Listener

import numpy as np
from rosgraph_msgs.msg._clock import Clock

import matplotlib.pyplot as plt


class ClockMonitor:
    def __init__(self, node):
        self.node = node
        #self.self.node.create_subscription(Clock, "/clock", self.callback, 1)

        self.node.create_timer(1, self.check_clock_publishers)

        publishers = self.node.get_publishers_info_by_topic("/clock")
        self._previous_pub_len = len(publishers)
        self._done = False

    def check_clock_publishers(self):
        publishers = self.node.get_publishers_info_by_topic("/clock")
        if len(publishers) == 0 and self._previous_pub_len > 0:
            self._done = True

        self._previous_pub_len = len(publishers)

    @property
    def done(self):
        return self._done


class Timeseries:
    def __init__(self, node, topic_type, topic, attributes):
        self.listener = Listener(node, topic_type, topic, attributes)

    def to_numpy(self):
        # TODO: also need time
        return np.array(self.listener.data)


def main(args=None):
    rclpy.init(args=args)
    node = Node("plotter_node")

    clock_monitor = ClockMonitor(node)
    steer_torque_fl = Timeseries(node, WheelControllerInfo,
                                 "/vehicle/wheel_fl/controller_info", ["steer_torque"])

    while not clock_monitor.done:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

    # Can write to disk here

    steer_torque_fl = steer_torque_fl.to_numpy()
    plt.plot(steer_torque_fl)

    plt.show()


if __name__ == "__main__":
    main()
