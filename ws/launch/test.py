
import asyncio
from asyncio.tasks import sleep
from rclpy.node import Node
import rclpy


from report_utils.experiments import Experiment
from report_utils.experiments import ExperimentRunnerBase


class ExperimentRunnerTest(ExperimentRunnerBase):
    def __init__(self, experiment):
        super().__init__(experiment, 1.0)

    def check_if_done(self):
        t = self.get_clock().now()
        if (t - self._start_time).nanoseconds > 20 * 10**9:
            self._set_done()


experiment = Experiment(
    ["ros2",
     "launch",
     "launch/all.launch.py",
     "gui:=false",
     "use_simple_lap:=true",
     "bag:=true"])


rclpy.init()
runner = ExperimentRunnerTest(experiment)
runner.start()
rclpy.spin_until_future_complete(runner, runner.future)
runner.stop()
runner.destroy_node()
rclpy.shutdown()
