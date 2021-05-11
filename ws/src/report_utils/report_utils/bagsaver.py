from typing import Iterable
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from vehicle_interface.msg import VehicleControllerInfo
from vehicle_interface.msg import WheelControllerInfo

from report_utils.listener import Listener

import numpy as np
from rosgraph_msgs.msg._clock import Clock

import matplotlib.pyplot as plt
from builtin_interfaces.msg._time import Time


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
    def __init__(self, node, topic_type, topic, time_attribute, data_attributes):
        self._data_attributes = data_attributes
        self._time_attribute = time_attribute

        self._time_attribute_index = 0  # time is in index 0 in array below
        attributes = [time_attribute, *data_attributes]
        self.listener = Listener(node, topic_type, topic, attributes)

    @staticmethod
    def convert_time_msgs(time_msgs, start_time=None):
        if start_time is None:
            start_time = time_msgs[0]

        elapsed_time = np.empty(len(time_msgs))
        for i, msg in enumerate(time_msgs):
            elapsed_time[i] = (time_msgs[i].sec - start_time.sec) + \
                (time_msgs[i].nanosec - start_time.nanosec)/(10**9)

        return start_time, elapsed_time

    def to_numpy(self):
        # TODO: also need time
        return np.array(self.listener.data)

    def to_dict(self):
        data = {}
        raw = self.to_numpy()
        time_msgs = raw[:, self._time_attribute_index]
        raw = np.delete(raw, self._time_attribute_index, axis=1)
        start_time, elapsed_time = self.convert_time_msgs(time_msgs)

        data["start_time"] = start_time
        data["elapsed_time"] = elapsed_time
        for i, attribute in enumerate(self._data_attributes):
            data[attribute] = raw[:, i]

        return data


def time_difference(time1: Time, time2: Time) -> float:
    """
    Compute time1 - time2 and return time difference 
    in seconds as a float.
    Only numerically accurate when time1 and time2 
    are close
    """
    dsec = time1.sec - time2.sec
    dnanosec = time1.nanosec - time2.nanosec
    dt = dsec + dnanosec/(10**9)
    return dt


def main(args=None):
    rclpy.init(args=args)
    node = Node("plotter_node")

    clock_monitor = ClockMonitor(node)
    wheel_fl = Timeseries(node, WheelControllerInfo,
                          "/vehicle/wheel_fl/controller_info",
                          "header.stamp",
                          ["steer_torque", "drive_torque"])

    vehicle = Timeseries(node, VehicleControllerInfo,
                         "/vehicle/controller_info",
                         "header.stamp",
                         ["yaw", "cross_track_error", "yaw_error", "speed"])

    while not clock_monitor.done:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

    # Can write to disk here

    wheel_fl = wheel_fl.to_dict()
    vehicle = vehicle.to_dict()

    tv = vehicle["elapsed_time"]
    time_offset_fl = time_difference(
        wheel_fl["start_time"], vehicle["start_time"])
    tw_fl = wheel_fl["elapsed_time"] + time_offset_fl

    # plt.plot(t)

    plt.plot(tv, vehicle["speed"])
    plt.plot(tw_fl, wheel_fl["drive_torque"])

    # plt.plot(wheel_fl["elapsed_time"],
    #          wheel_fl["steer_torque"], label=r"$\tau_s$")
    # print(wheel_fl)
    # plt.plot(steer_torque_fl)
    plt.show()


if __name__ == "__main__":
    main()
