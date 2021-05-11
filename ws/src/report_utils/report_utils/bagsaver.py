import rclpy
from rclpy.node import Node

import numpy as np
# from rosgraph_msgs.msg._clock import Clock
from builtin_interfaces.msg._time import Time

import matplotlib.pyplot as plt

import yaml
import pickle


from report_utils.listener import Listener


class ClockMonitor:
    def __init__(self, node):
        self.node = node

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
        data["elapsed_time"] = elapsed_time.astype(np.float64)
        for i, attribute in enumerate(self._data_attributes):
            data[attribute] = raw[:, i].astype(np.float64)

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
    # TODO: there must be a better way to get arguments for
    # console scripts
    import sys
    console_args = sys.argv[1:]
    arguments = {}
    value_arguments = ["--config"]
    switch_arguments = []
    for i in range(len(console_args)):
        if console_args[i] in value_arguments:
            arguments[console_args[i]] = console_args[i+1]
        if console_args[i] in switch_arguments:
            arguments[console_args[i]] = True

    if not "--config" in arguments.keys():
        print("Config file not provided for bagsaver. arguments were",
              sys.argv[1:])
        sys.exit(1)

    config_yaml_file = arguments["--config"]
    with open(config_yaml_file, "r") as file:
        config = yaml.safe_load(file)

    rclpy.init(args=args)
    node = Node("bagsaver_node")

    clock_monitor = ClockMonitor(node)

    # parse config file and set up listeners
    timeseries = []
    for topic, d in config["topics"].items():
        # Import module with topic msg type
        mod = __import__(d["topic_type_module"], fromlist=[d["topic_type"]])
        topic_type = getattr(mod, d["topic_type"])

        timeseries.append(Timeseries(node,
                                     topic_type,
                                     topic,
                                     d["time_attribute"],
                                     d["data_attributes"]))

    while not clock_monitor.done:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

    data = {}
    for ts in timeseries:
        data[ts.listener.topic] = ts.to_dict()

    # Convert all times so it measured relative
    # to the time owner
    time_owner_topic = config["time"]["relative_to"]
    owner_start_time = data[time_owner_topic]["start_time"]
    for topic in data.keys():
        time_offset = time_difference(
            data[topic]["start_time"], owner_start_time)
        data[topic]["elapsed_time"] = data[topic]["elapsed_time"] + time_offset
        data[topic]["start_time"] = owner_start_time

    # Write to disk
    outputfile = config.get("outputfile", None)
    if not outputfile is None:
        with open(outputfile, "wb") as file:
            pickle.dump(data, file)

    tv = data["/vehicle/controller_info"]["elapsed_time"]
    tw = data["/vehicle/wheel_fl/controller_info"]["elapsed_time"]

    yaw = data["/vehicle/controller_info"]["yaw"]
    speed = data["/vehicle/controller_info"]["speed"]

    delta_fl = data["/vehicle/wheel_fl/controller_info"]["state.steering_angle"]
    delta_fl_r = data["/vehicle/wheel_fl/controller_info"]["reference.steering_angle"]

    def ssa(x): return np.arctan2(np.sin(x), np.cos(x))

    plt.plot(tw, ssa(delta_fl), label=r"$\delta$")
    plt.plot(tw, ssa(delta_fl_r), label=r"$\delta^r$")

    plt.legend()
    #plt.plot(tv, speed)

    plt.show()

    # tv = vehicle["elapsed_time"]
    # time_offset_fl = time_difference(
    #     wheel_fl["start_time"], vehicle["start_time"])
    # tw_fl = wheel_fl["elapsed_time"] + time_offset_fl

    # # plt.plot(t)

    # plt.plot(tv, vehicle["speed"])
    # # plt.plot(tw_fl, wheel_fl["drive_torque"])

    # # plt.plot(wheel_fl["elapsed_time"],
    # #          wheel_fl["steer_torque"], label=r"$\tau_s$")
    # # print(wheel_fl)
    # # plt.plot(steer_torque_fl)
    # plt.show()


if __name__ == "__main__":
    import sys
    main(args=sys.argv[1:])
