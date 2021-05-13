from report_utils.experiments import ExperimentRunnerBase

from geometry_msgs.msg import PoseStamped
import rclpy


class SingleTurnExperiment(ExperimentRunnerBase):
    def __init__(self):
        cmd = ["ros2",
               "launch",
               "launch/all.launch.py",
               "gui:=false",
               "use_single_turn:=true",
               "bag:=true"]

        super().__init__(cmd)

        self._sub = self.create_subscription(
            PoseStamped, "/vehicle/pose", self._pose_callback, 1)

    def _pose_callback(self, msg: PoseStamped):
        if abs(msg.pose.position.x - 30) + abs(msg.pose.position.y - 30) < 5.0:
            self._set_done()


rclpy.init()

e = SingleTurnExperiment()
e.run()

rclpy.shutdown()
