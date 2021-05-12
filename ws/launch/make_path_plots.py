

import subprocess

pathgenerator = subprocess.Popen([
    "./build/control/pathlib_testing",
    "--waypoints",
    "0,0",
    "10,0",
    "10,5",
    "5,5",
    "0,4",
    "--curvature",
    "1",
    "--smoothing",
    "fermat"],
    stdout=subprocess.PIPE)
pathplotter = subprocess.Popen([
    "ros2",
    "run",
    "report_utils",
    "path_plotter",
    "--pdffigdir", "../figs",
    "--rawfigdir", "../figs/raw",
    "--figname", "fermat_smoothing_1"],
    stdin=pathgenerator.stdout)
pathgenerator.stdout.close()
