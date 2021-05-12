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
    curvatures = options["curvatures"]

    fig, ax = plotlib.subplots(num=name)
    for curvature in curvatures:
        pathgenerator_cmd = [
            "./build/control/pathlib_testing",
            "--waypoints", *options["waypoints"],
            "--curvature", str(curvature),
            "--smoothing", options["smoothing"],
            "--num-samples", str(options["num-samples"])]
        # print(pathgenerator_cmd)
        smoothed_path = subprocess.check_output(pathgenerator_cmd)
        smoothed_path = _point_list_to_numpy(smoothed_path, b'\n', b',')

        if len(curvatures) > 1:
            label = rf"$\kappa={curvature}$"
        else:
            label = ""
        plotlib.plot_xy(smoothed_path[:, 0], smoothed_path[:, 1],
                        label=label, ax=ax)

    waypoints = np.array(list(
        map(lambda s: _point_to_numpy(s, ','), options["waypoints"])))

    if len(curvatures) > 1:
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
        method = globals()[expconfig["method"]]
        method(expname, expconfig["options"])


if __name__ == "__main__":
    main()
