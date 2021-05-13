import asyncio
from abc import abstractmethod
import subprocess
import signal

from dataclasses import dataclass

from typing import List

import rclpy
from rclpy.node import Node


@dataclass
class Experiment:
    cmd: List[str]
    process: subprocess.Popen = None


class ExperimentRunnerBase(Node):
    def __init__(self, cmd):
        super().__init__("experiment_runner")
        self._experiment = Experiment(cmd)
        self.future = asyncio.Future()

    @abstractmethod
    def check_if_done(self): ...

    def start(self):
        print("start", self._experiment.cmd)
        start_experiment(self._experiment)

    def stop(self):
        print("stop", self._experiment.cmd)
        stop_experiment(self._experiment)

    def _set_done(self):
        self.future.set_result(0)

    def run(self):
        self.start()
        rclpy.spin_until_future_complete(self, self.future)
        self.stop()
        self.destroy_node()


def start_experiment(experiment):
    experiment.process = subprocess.Popen(
        experiment.cmd, stdout=subprocess.DEVNULL)

    return experiment


def stop_experiment(experiment: Experiment):
    proc = experiment.process
    proc.send_signal(signal.SIGINT)

    escalate = False
    try:
        proc.wait(2)
    except subprocess.TimeoutExpired:
        escalate = True
        pass

    if escalate:
        pids = subprocess.check_output(
            ["pgrep", "-P", str(proc.pid)]).strip().split(b'\n')
        pids = [int(pid) for pid in pids]

        for pid in pids:
            subprocess.run(["kill", str(pid)])

        subprocess.run(["pkill", "gzserver"])
        subprocess.run(["pkill", "gzclient"])

        # sometimes we need to force kill ros2
        subprocess.run(["pkill", "-1", "ros2"])
        subprocess.run(["pkill", "component_conta"])
