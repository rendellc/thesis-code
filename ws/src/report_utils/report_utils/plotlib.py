import numpy as np
import matplotlib.pyplot as plt
import pickle
from pathlib import Path
import matplotlib

# plt.style.use(["science", "grid", "ieee", "no-latex"])
plt.style.use(["science", "grid", "no-latex"])

RAW_FIG_DIR = Path(".")
PDF_FIG_DIR = Path(".")

FULL_SIZE = (7.0, 4.2)
COL_SIZE = (3.5, 2.2)


def subplots(*args, **kwargs):
    # clear by default
    kwargs['clear'] = kwargs.get('clear', True)
    fig, ax = plt.subplots(*args, **kwargs)
    # make_full_size(fig)

    return fig, ax


def make_full_size(fig: matplotlib.figure.Figure):
    fig.set_size_inches(FULL_SIZE)


def make_col_size(fig: matplotlib.figure.Figure):
    fig.set_size_inches(COL_SIZE)


def create_save_directories():
    if not PDF_FIG_DIR.exists():
        print("Creating figure directory")
        PDF_FIG_DIR.mkdir(parents=True)
    if not RAW_FIG_DIR.exists():
        print("Creating raw figure directory")
        RAW_FIG_DIR.mkdir(parents=True)


def set_save_directories(rawdir: str, pdfdir: str):
    global RAW_FIG_DIR, PDF_FIG_DIR
    RAW_FIG_DIR = Path(rawdir)
    PDF_FIG_DIR = Path(pdfdir)
    create_save_directories()


def plot_timeseries(t, x, label, ax=None):
    if ax is None:
        _, ax = plt.subplots()

    ax.plot(t, x, label=label)

    ax.set_xlim(t[0], t[~0])

    return ax


def plot_xy(x, y, label, ax=None):
    if ax is None:
        _, ax = plt.subplots()

    ax.plot(x, y, label=label)
    ax.set_aspect("equal")
    return ax


def plot_waypoints(waypoints, label, ax=None):
    if ax is None:
        _, ax = plt.subplots()

    ax.scatter(waypoints[:, 0], waypoints[:, 1], 20,
               marker='x', color="r", label=label, zorder=3)
    return ax


def savefig(fig: plt.figure, figurename=None):
    """
    Save figure as .pdf and .fig
    """
    if figurename is None:
        figurename = fig.properties()["label"]

    try:
        pickle.dump(
            fig, (RAW_FIG_DIR/figurename).with_suffix(".fig").open(mode="wb"))
    except TypeError as e:
        print(f"failed to pickle figure {fig}:", e)

    fig.savefig((PDF_FIG_DIR/figurename).with_suffix(".pdf"), bbox_inches="tight")


def plot_from_bagsaver(config, data, display=False):
    """
    Plot data from bagsaver. Not safe to run in parallel as
    it modifies the state of the module
    """
    plotconfig = config["plots"]
    rawfigdir = config.get("figure_directory_raw", ".")
    pdffigdir = config.get("figure_directory", ".")
    set_save_directories(rawfigdir, pdffigdir)

    for plotname, plotconfig in plotconfig.items():
        try:
            pc = plotconfig  # shorter name
            fig, ax = plt.subplots(num=plotname)

            topic = pc["topic"]

            if pc["type"] == "timeseries":
                t = data[topic]["elapsed_time"]
                for attribute in pc["attributes"]:
                    x = data[topic][attribute]
                    plot_timeseries(t, x, label=attribute, ax=ax)

                xlow, xupp = pc.get("xlim", [t[0], t[~0]])
                ax.set_xlim(xlow, xupp)

                ax.set_xlabel(pc.get("xlabel", "time [s]"))
                ax.set_ylabel(pc.get("ylabel", plotname))
            if pc["type"] == "xy":
                x = data[topic][pc["xdata"]]
                y = data[topic][pc["ydata"]]
                label = pc["label"]

                plot_xy(x, y, label=label, ax=ax)

                ax.set_xlabel(pc.get("xlabel", ""))
                ax.set_ylabel(pc.get("ylabel", ""))

            savefig(fig, plotname)

        except KeyError as e:
            print("error in", plotname)
            print(plotconfig)
            raise e

    if display:
        plt.show()
