import yaml

import sys

import subprocess

import report_utils.plotlib as plotlib

import numpy as np


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


def make_path_plots(name, options, experiment_config):
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

    plotlib.set_save_directories("../figs/raw", "../figs")

    plotlib.savefig(fig)


def main():
    if len(sys.argv) != 2:
        print("Need a config file")
        print(sys.argv[0], "experiments.yaml")
        return

    experiment_config_file = sys.argv[1]
    with open(experiment_config_file, "r") as file:
        config = yaml.safe_load(file)

    experiments = config["experiments"]

    for expname, expconfig in experiments.items():
        method = globals()[expconfig["method"]]
        method(expname, expconfig["options"], config)


if __name__ == "__main__":
    main()
