import rclpy
from rclpy.node import Node

import numpy as np
# from rosgraph_msgs.msg._clock import Clock
from builtin_interfaces.msg._time import Time

import matplotlib.pyplot as plt

import yaml
import pickle
from pathlib import Path


from report_utils.listener import Listener
import report_utils.plotlib as plotlib


class ClockMonitor:
    def __init__(self, node):
        self.node = node

        self.node.create_timer(0.01, self.check_clock_publishers)

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


def run_with_config(config, args=None, datafile=None):
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
    if not datafile is None:
        datafile = Path(datafile)
        datafile.parent.mkdir(parents=True, exist_ok=True)
        with open(datafile, "wb") as file:
            pickle.dump(data, file)

    return data


def main(args=None):
    # TODO: there must be a better way to get arguments for
    # console scripts
    import sys
    console_args = sys.argv[1:]

    # NOTE: something weird happens when used with
    # subprocess.Popen
    # Quickfix is to split the console arguments manually
    if len(console_args) == 1 and " " in console_args[0]:
        console_args = console_args.split(" ")
        print(console_args)

    if not "--config" in console_args:
        print("Config file not provided for bagsaver. arguments were", console_args)
        sys.exit(1)

    arguments = {}
    value_arguments = ["--config", "--datafile"]
    switch_arguments = ["--display"]
    for i in range(len(console_args)):
        if console_args[i] in value_arguments:
            arguments[console_args[i]] = console_args[i+1]

    for arg in switch_arguments:
        if arg in console_args:
            arguments[arg] = True
        else:
            arguments[arg] = False

    config_yaml_file = arguments["--config"]
    with open(config_yaml_file, "r") as file:
        config = yaml.safe_load(file)

    run_with_config(config, args, arguments["--datafile"])

    return 0


if __name__ == "__main__":
    main()
