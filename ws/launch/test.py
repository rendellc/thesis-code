

import time
import signal
import subprocess

from report_utils.experiments import start_experiment, stop_experiment


exp = start_experiment(
    ["ros2", "launch", "launch/all.launch.py", "gui:=false", "use_single_turn:=true"])


time.sleep(10)

stop_experiment(exp)
