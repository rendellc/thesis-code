import yaml

import sys
import pickle
import time

import subprocess

import report_utils.plotlib as plotlib
import report_utils.bagsaver as bagsaver
import report_utils.datalib as datalib
import numpy as np
from pathlib import Path
from geometry_msgs.msg import PoseStamped
from report_utils.experiments import ExperimentMonitor, ExperimentRunnerBase
import shlex
from pathlib import Path
import shutil
import signal

from report_utils.launch_parameters import WP_SIMPLE_LAP, WP_SINGLE_TURN
from report_utils.chattering_analysis import chatter_metric, chatter_signal


class CommandExperimentRunner(ExperimentRunnerBase):
    def __init__(self, cmd):
        # cmd = ["ros2",
        #        "launch",
        #        "launch/all.launch.py",
        #        "gui:=true",
        #        "use_single_turn:=true",
        #        "bag:=true"]

        super().__init__(cmd)

        self._sub = self.create_subscription(
            PoseStamped, "/vehicle/pose", self._pose_callback, 1)

    def _pose_callback(self, msg: PoseStamped):
        xfinal, yfinal = WP_SINGLE_TURN[~0]
        if abs(msg.pose.position.x - xfinal) + abs(msg.pose.position.y - yfinal) < 4.0:
            self._set_done()


class SingleTurnMonitor(ExperimentMonitor):
    def __init__(self):
        super().__init__()
        self._sub = self.create_subscription(
            PoseStamped, "/vehicle/pose", self._pose_callback, 1)

    def _pose_callback(self, msg: PoseStamped):
        xfinal, yfinal = WP_SINGLE_TURN[~0]
        if abs(msg.pose.position.x - xfinal) + abs(msg.pose.position.y - yfinal) < 4.0:
            self._set_done()


class SimpleLapMonitor(ExperimentMonitor):
    def __init__(self):
        super().__init__()
        self._sub = self.create_subscription(
            PoseStamped, "/vehicle/pose", self._pose_callback, 1)

        self._left_start = False

    def _pose_callback(self, msg: PoseStamped):
        xstart, ystart = WP_SIMPLE_LAP[0]
        distance_to_start = abs(msg.pose.position.x - xstart) + \
            abs(msg.pose.position.y - ystart)
        if distance_to_start > 10:
            self._left_start = True

        xfinal, yfinal = WP_SIMPLE_LAP[~0]
        if self._left_start and abs(msg.pose.position.x - xfinal) + abs(msg.pose.position.y - yfinal) < 4.0:
            self._set_done()


class TimeLimit20SecMonitor(ExperimentMonitor):
    def __init__(self):
        super().__init__()
        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(1.0, self._timer_callback)

    def _timer_callback(self):
        t = self.get_clock().now()
        d = t - self._start_time

        if d.nanoseconds > 20*10**9:
            self._set_done()


class TimeLimit60SecMonitor(ExperimentMonitor):
    def __init__(self):
        super().__init__()
        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(1.0, self._timer_callback)

    def _timer_callback(self):
        t = self.get_clock().now()
        d = t - self._start_time

        if d.nanoseconds > 60*10**9:
            self._set_done()


def _start_processes(commands):
    processes = []
    for cmd in commands:
        processes.append(
            subprocess.Popen(cmd)
        )
        #subprocess.Popen(cmd, stdout=subprocess.DEVNULL)
    return processes


def _get_masked_metric_data(data, options):
    xs = data[options["data"]["topic"]][options["data"]["attribute"]]
    t = data[options["data"]["topic"]]["elapsed_time"]
    tlim = options["data"].get("tlim", [t[0], t[~0]])
    mask = (tlim[0] <= t) * (t <= tlim[1])
    return xs[mask]


def meanabs(data, options):
    xs = _get_masked_metric_data(data, options)
    return np.mean(np.abs(xs))


def mean(data, options):
    xs = _get_masked_metric_data(data, options)
    return np.mean(xs)


def finalabs(data, options):
    xs = _get_masked_metric_data(data, options)
    return np.abs(xs[~0])


def peak(data, options):
    xs = _get_masked_metric_data(data, options)
    i = np.argmax(np.abs(xs))
    return np.abs(xs[i])


def chatter(data, options):
    xs = _get_masked_metric_data(data, options)
    ch = chatter_signal(xs)
    return np.std(ch)
    # return chatter_metric(xs)


def std(data, options):
    xs = _get_masked_metric_data(data, options)
    return np.std(xs)


def chattering_signal_plotter(name, options):
    print("Making", name)
    fig = options["fig"]
    tlim = options["tlim"]

    f = plotlib.open_figure(fig)
    t = f.get_axes()[0].lines[0].get_xdata()
    torque = f.get_axes()[0].lines[0].get_ydata()
    torque_smoothened = chatter_signal(torque)

    fig, ax = plotlib.subplots(num=name)
    plotlib.plot_timeseries(t, torque, label=r"$\tau_i$", ax=ax)
    # plotlib.plot_timeseries(t, torque_smoothened,
    #                         label=r"$\tau_i^s$", alpha=0.4, ax=ax)

    ax.set_xlim(*tlim)

    plotlib.savefig(fig)


def _stop_processes(processes):
    for process in processes:
        process.send_signal(signal.SIGINT)


def _kill_processes(processes):
    _stop_processes(processes)
    escalate = False
    try:
        for p in processes:
            p.wait(5)
    except subprocess.TimeoutExpired:
        escalate = True
        pass

    if escalate:
        pidstokill = []
        for p in processes:
            p.poll()
            if p.returncode is not None:
                try:
                    pids = subprocess.check_output(
                        ["pgrep", "-P", str(p.pid)]).strip().split(b'\n')
                    pids = [int(pid) for pid in pids]

                    pidstokill.extend(pids)
                except subprocess.CalledProcessError:
                    print(f"pgrep -P {p.pid} failed")
                    pass

        for pid in pidstokill:
            subprocess.run(["kill", str(pid)])

        subprocess.run(["pkill", "gzserver"])
        subprocess.run(["pkill", "gzclient"])
        subprocess.run(["pkill", "static_transfor"])
        subprocess.run(["pkill", "component_conta"])
        subprocess.run(["pkill", "component_conta"])

        # sometimes we need to force kill ros2
        subprocess.run(["pkill", "-1", "ros2"])


def check_bags():
    return set(Path("bagfolder").glob("*.db3"))


def run_command(options):
    cmd_str = options["command"]
    bagsbefore = check_bags()
    cmd = shlex.split(cmd_str)
    import rclpy
    rclpy.init()
    e = CommandExperimentRunner(cmd)
    e.run()
    rclpy.shutdown()

    bagsafter = check_bags()
    createdbag = next(iter(bagsafter - bagsbefore))
    bagfile = Path(options["bag"]["file"])

    shutil.copy(createdbag, bagfile)
    # sanity check to ensure we don't delete the entire project
    assert createdbag.parent.absolute() == Path(
        "/home/cale/thesis-code/ws/bagfolder")
    shutil.rmtree(createdbag.parent)


def run_commands(options):
    # print(options["commands"])
    shutil.rmtree("/home/cale/thesis-code/ws/bagfolder", ignore_errors=True)

    bagsbefore = check_bags()
    commands = list(map(shlex.split, options["commands"]))
    processes = _start_processes(commands)
    Monitor = globals()[options["monitor"]]

    import rclpy
    rclpy.init()

    monitor = Monitor()
    rclpy.spin_until_future_complete(monitor, monitor.future)

    rclpy.shutdown()
    _kill_processes(processes)

    for process in processes:
        process.wait(5)
        # print(process.pid, "done")

    bagsafter = check_bags()
    bagdifference = bagsafter - bagsbefore
    if len(bagdifference) == 1:
        createdbag = next(iter(bagsafter - bagsbefore))
        bagfile = Path(options["bag"]["file"])

        shutil.copy(createdbag, bagfile)
        # sanity check to ensure we don't delete the entire project
        assert createdbag.parent.absolute() == Path(
            "/home/cale/thesis-code/ws/bagfolder")
    elif len(bagdifference) > 1:
        print(
            f"{len(bagsafter-bagsbefore)} created bags in bagfolder. Don't know what to do")


def _point_to_numpy(point_str, numsep):
    point = np.empty((2,))
    line = point_str.strip()
    line = line.split(numsep)
    point[0] = float(line[0])
    point[1] = float(line[1])

    return point


def _point_list_to_numpy(points_string, linesep, numsep):
    points_string = points_string.strip()
    point_strings = points_string.split(linesep)
    points = np.empty((len(point_strings), 2))
    for i, line in enumerate(point_strings):
        points[i] = _point_to_numpy(line, numsep)

    return points


def make_path_plots(name, options):
    print(f"Making path plot for {name}")

    curvatures = options["curvatures"]
    if "smoothings" in options:
        smoothings = options["smoothings"]
        if not len(curvatures) == 1:
            print("Too many curvatures supplied, expected 1")
            return
        # make curvatures exactly as long as number of smoothings
        curvatures = [curvatures[0]]*len(smoothings)
    else:
        smoothings = [options["smoothing"]]*len(curvatures)

    fig, ax = plotlib.subplots(num=name)

    labels = options.get("labels", [])
    for i, (smoothing, curvature) in enumerate(zip(smoothings, curvatures)):
        pathgenerator_cmd = [
            "./build/control/pathlib_testing",
            "--waypoints", *options["waypoints"],
            "--curvature", str(curvature),
            "--radius", str(1.0/curvature),
            "--smoothing", smoothing,
            "--num-samples", str(options["num-samples"])]
        # print(pathgenerator_cmd)
        smoothed_path = subprocess.check_output(pathgenerator_cmd)
        smoothed_path = _point_list_to_numpy(smoothed_path, b'\n', b',')

        if labels:
            label = labels[i]
        else:
            label = ""
        plotlib.plot_xy(smoothed_path[:, 0], smoothed_path[:, 1],
                        label=label, ax=ax)

    waypoints = np.array([_point_to_numpy(wp, ",")
                         for wp in options["waypoints"]])
    plotlib.plot_waypoints(waypoints, "", ax=ax)

    waypoints = np.array(list(
        map(lambda s: _point_to_numpy(s, ','), options["waypoints"])))

    if labels:
        legend_loc = options.get("legend_loc", "best")
        ax.legend(loc=legend_loc)

    # ax.grid(False)

    plotlib.savefig(fig)


def make_bag_plots(name, options):
    print(f"Making bag plots for {name} using {options['bag']['file']}")

    datafile = (datalib.SAVE_DIR/name).with_suffix(".pickle")
    bagfile = Path(options["bag"]["file"])
    bagsaver_configfile = Path(options["bagsaver_config"])

    build_datafile = options["bag"].get("force", False)
    if not datafile.exists():
        build_datafile = True
    else:
        bagfile_updated = bagfile.stat().st_mtime > datafile.stat().st_mtime
        bagsaver_config_updated = bagsaver_configfile.stat(
        ).st_mtime > datafile.stat().st_mtime
        if bagfile_updated or bagsaver_config_updated:
            build_datafile = True

    if build_datafile:
        bagsaver_ps = subprocess.Popen([
            "ros2",
            "run",
            "report_utils",
            "bagsaver",
            "--config",
            options['bagsaver_config'],
            "--datafile",
            str(datafile)],
            # stdout=subprocess.STDOUT,
            # stderr=subprocess.DEVNULL
        )

        # wait for bagsaver to be ready
        time.sleep(5.0)

        bag_ps = subprocess.Popen([
            "ros2",
            "bag",
            "play",
            "--rate", str(options["bag"]["rate"]),
            options["bag"]["file"]],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        bag_ps.wait()
        bagsaver_ps.wait()
    else:
        print("using cached datafile")

    data = pickle.load(datafile.open("rb"))

    save_dirs_prev = (plotlib.RAW_FIG_DIR, plotlib.PDF_FIG_DIR)
    plotlib.set_save_directories(
        plotlib.RAW_FIG_DIR / name,
        plotlib.PDF_FIG_DIR / name
    )

    plots = options["plots"]
    for plotname, plotoptions in plots.items():
        fig, ax = plotlib.subplots(num=plotname)
        if plotoptions["type"] == "timeseries":
            ds = plotoptions["datas"]
            for d in ds:
                t = data[d["topic"]]["elapsed_time"]
                x = data[d["topic"]][d["attribute"]]

                scale = d.get("scale", None)
                if scale == "rad2deg":
                    x = np.rad2deg(x)
                if scale == "abs":
                    x = np.abs(x)
                if type(scale) == float or type(scale) == int:
                    x = scale*x

                plotlib.plot_timeseries(t, x, d["label"], ax=ax)
        if plotoptions["type"] == "xy":
            ds = plotoptions["datas"]
            for d in ds:
                x = data[d["topic"]][d["xattribute"]]
                y = data[d["topic"]][d["yattribute"]]
                label = d.get("label", "")
                alpha = d.get("alpha", 1.0)
                plotlib.plot_xy(x, y, label, ax=ax, alpha=alpha)

        if plotoptions.get("legend", False):
            ax.legend(loc="upper right")

        ylim = plotoptions.get("ylim", None)
        if not ylim is None:
            ax.set_ylim(ylim[0], ylim[1])
        xlim = plotoptions.get("xlim", None) or plotoptions.get("tlim", None)
        if not xlim is None:
            ax.set_xlim(xlim[0], xlim[1])

        ax.set_xlabel(plotoptions.get("xlabel", ""))
        ax.set_ylabel(plotoptions.get("ylabel", ""))

        plotlib.savefig(fig)

    metricdir = plotlib.PDF_FIG_DIR
    metricdir.mkdir(parents=True, exist_ok=True)
    metrics = options.get("metrics", dict())
    metricvalues = dict()
    for metricname, metricoptions in metrics.items():
        method = globals()[metricoptions["method"]]
        metric = method(data, metricoptions)
        scale = metricoptions.get("scale", None)
        if scale == "abs":
            metric = np.abs(metric)
        if scale == "rad2deg":
            metric = np.rad2deg(metric)
        if type(scale) == float or type(scale) == int:
            metric = scale*metric
        metricvalues[metricname] = metric

        with (metricdir / metricname).open("w") as f:
            # f.write(str(metric))
            f.write(f"{metric:.3f}")

    print("metrics", metricvalues)

    # restore save directories
    plotlib.set_save_directories(*save_dirs_prev)


def chattering_comparison(name, options):
    print("Making", name)
    figs = options["figs"]
    labels = options["labels"]
    alpha = options["alpha"]
    tlim = options["tlim"]

    fig, ax = plotlib.subplots(num=name+"_zoomed")
    for i in range(len(figs)):
        f = plotlib.open_figure(figs[i])
        t = f.get_axes()[0].lines[0].get_xdata()
        torque = f.get_axes()[0].lines[0].get_ydata()
        chatter = chatter_signal(torque, alpha)

        # tlim = plotregions[i]
        # idx = (tlim[0] <= t)*(t <= tlim[1])
        plotlib.plot_timeseries(t, chatter, label=labels[i], ax=ax)

    ax.legend()
    ax.set_ylabel("chatter [Nm]")
    plotlib.savefig(fig)

    ax.set_xlim(*tlim)
    ax.autoscale(True)
    plotlib.savefig(fig, name+"_zoomed")


def main():
    if len(sys.argv) != 2:
        print("Need a config file")
        print(sys.argv[0], "experiments.yaml")
        return

    experiment_config_file = sys.argv[1]
    with open(experiment_config_file, "r") as file:
        config = yaml.safe_load(file)

    settings = config["settings"]
    experiments = config["experiments"]

    rerunlist = settings.get("rerun", [])

    def should_run(experiment_name):
        return True
    if settings.get("onlyrun", None) != None:
        def should_run(experiment_name):
            if isinstance(settings["onlyrun"], list):
                onlyrunlist = settings["onlyrun"]
            else:
                onlyrunlist = [settings["onlyrun"]]

            return experiment_name in onlyrunlist

    plotlib.set_save_directories(
        config["settings"]["rawdir"],
        config["settings"]["pdfdir"])
    datalib.set_save_directories(config["settings"]["datadir"])

    for expname, expconfig in experiments.items():
        if not should_run(expname):
            continue

        if "experimenter" in expconfig:
            experimenter = globals()[expconfig["experimenter"]]

            if len(rerunlist) == 0 or expname in rerunlist:
                # if expname in rerunlist or rerunlist
                # if expconfig["options"].get("rerun", True):
                experimenter(expconfig["options"])

        plotter = globals()[expconfig["plotter"]]
        plotter(expname, expconfig["options"])


if __name__ == "__main__":
    main()
