import sys
from matplotlib.pyplot import get_plot_commands, plot
import numpy as np

import report_utils.plotlib as plotlib


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
    from sys import argv
    last_command = ""
    for i in range(len(argv)):
        if argv[i][:2] == "--":
            last_command = argv[i]

        if argv[i] == "--pdffigdir":
            pdfdir = argv[i+1]
        if argv[i] == "--rawfigdir":
            rawdir = argv[i+1]
        if argv[i] == "--figname":
            figname = argv[i+1]
            
        if last_command == "--smoothed":



    plotlib.set_save_directories(rawdir, pdfdir)

    #data = read_from_pipe()
    waypoints = parse_waypoints(data)

    fig, ax = plotlib.subplots(num=figname)
    plotlib.plot_xy(waypoints[:, 0], waypoints[:, 1], "path", ax=ax)

    plotlib.savefig(fig)


if __name__ == "__main__":
    main()
