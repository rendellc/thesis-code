import sys
from matplotlib.pyplot import get_plot_commands, plot
import numpy as np

from report_utils.plotlib import plot_xy


def read_from_pipe():
    try:
        data = sys.stdin.readlines()
    except KeyboardInterrupt:
        sys.stdout.flush()
        pass

    return data


def parse_waypoints(data):
    waypoints = np.empty((len(data), 2))
    for i, line in enumerate(data):
        line = line.strip()
        line = line.split(",")
        waypoints[i, 0] = float(line[0])
        waypoints[i, 1] = float(line[1])

    return waypoints


def main(args=None):
    data = read_from_pipe()
    waypoints = parse_waypoints(data)
    print(waypoints)
    ax = plot_xy(waypoints[:, 0], waypoints[:, 1], "path")

    from matplotlib.pyplot import show
    show()


if __name__ == "__main__":
    main()
