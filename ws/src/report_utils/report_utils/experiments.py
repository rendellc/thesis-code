import subprocess
import signal

from dataclasses import dataclass


@dataclass
class Experiment:
    process: subprocess.Popen


def start_experiment(cmd):
    process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL)
    return Experiment(process)


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
