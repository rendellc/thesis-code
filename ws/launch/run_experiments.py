from matplotlib.pyplot import plot
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
from report_utils.experiments import ExperimentRunnerBase
import shlex
from pathlib import Path
import shutil


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
        if abs(msg.pose.position.x - 30) + abs(msg.pose.position.y - 30) < 2.0:
            self._set_done()


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
        ax.legend()

    ax.grid(False)

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
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # wait for bagsaver to be ready
        time.sleep(2.0)

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
                if type(scale) == float or type(scale) == int:
                    x = scale*x

                plotlib.plot_timeseries(t, x, d["label"], ax=ax)

        if plotoptions.get("legend", False):
            ax.legend()

        ylim = plotoptions.get("ylim", None)
        if not ylim is None:
            ax.set_ylim(ylim[0], ylim[1])
        xlim = plotoptions.get("xlim", None)
        if not xlim is None:
            ax.set_xlim(xlim[0], xlim[1])

        ax.set_xlabel(plotoptions.get("xlabel", ""))
        ax.set_ylabel(plotoptions.get("ylabel", ""))

        plotlib.savefig(fig)

    # restore save directories
    plotlib.set_save_directories(*save_dirs_prev)


def main():
    if len(sys.argv) != 2:
        print("Need a config file")
        print(sys.argv[0], "experiments.yaml")
        return

    experiment_config_file = sys.argv[1]
    with open(experiment_config_file, "r") as file:
        config = yaml.safe_load(file)

    experiments = config["experiments"]

    plotlib.set_save_directories(
        config["settings"]["rawdir"],
        config["settings"]["pdfdir"])
    datalib.set_save_directories(config["settings"]["datadir"])

    for expname, expconfig in experiments.items():
        if "runner" in expconfig:
            runner = globals()[expconfig["runner"]]

            if expconfig["options"].get("rerun", True):
                runner(expconfig["options"])

        plotter = globals()[expconfig["plotter"]]
        plotter(expname, expconfig["options"])


if __name__ == "__main__":
    main()
