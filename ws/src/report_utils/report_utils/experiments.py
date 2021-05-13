import asyncio
from abc import abstractmethod
import subprocess
import signal

from dataclasses import dataclass

from typing import List

from rclpy.node import Node


@dataclass
class Experiment:
    cmd: List[str]
    process: subprocess.Popen = None


class ExperimentRunnerBase(Node):
    def __init__(self, experiment, check_period):
        super().__init__("experiment_watcher")
        self.timer_ = self.create_timer(check_period, self.check_if_done)
        # self._done = False
        self._start_time = self.get_clock().now()

        self.future = asyncio.Future()
        self._experiment = experiment

    @abstractmethod
    def check_if_done(self): ...

    def start(self):
        print("starting")
        start_experiment(self._experiment)

    def stop(self):
        print("stopping")
        stop_experiment(self._experiment)

    def _set_done(self):
        self.future.set_result(0)


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

        # sometimes we need to force kill ros2
        subprocess.run(["pkill", "-1", "ros2"])
        subprocess.run(["pkill", "component_conta"])
