import numpy as np


def alpha_filter(xs, alpha):
    # weights = [1 - alpha, alpha]
    # return np.convolve(xs, weights, "same")
    xfs = np.empty_like(xs)

    xfs[0] = xs[0]

    for i in range(1, len(xs)):
        xfs[i] = alpha*xfs[i-1] + (1-alpha)*xs[i]

    return xfs


def smoothening(xs):
    w = np.hanning(5)
    w = w/np.sum(w)

    return np.convolve(xs, w, "same")


def chatter_signal(xs, alpha=0.5):
    # xs_filtered = alpha_filter(xs, alpha)
    xs_smoothed = smoothening(xs)
    return xs - xs_smoothed


if __name__ == "__main__":
    import report_utils.plotlib as pl
    import pathlib
    import matplotlib.pyplot as plt

    rawdir = pathlib.Path("/home/cale/thesis-code/figs/raw")
    figpath_smc = rawdir / "sliding_mode_results" / "steering_torque.fig"
    figpath_rr = rawdir / "robust_rate_results" / "steering_torque.fig"

    fig_smc = pl.open_figure(figpath_smc)
    fig_rr = pl.open_figure(figpath_rr)

    torque_smc = fig_smc.get_axes()[0].lines[0].get_ydata()
    time_smc = fig_smc.get_axes()[0].lines[0].get_xdata()
    T_smc = time_smc[~0] - time_smc[0]
    torque_rr = fig_rr.get_axes()[0].lines[0].get_ydata()
    time_rr = fig_rr.get_axes()[0].lines[0].get_xdata()
    T_rr = time_rr[~0] - time_rr[0]

    alpha = 0.5
    chatter_smc = chatter_signal(torque_smc)
    chatter_rr = chatter_signal(torque_rr)

    print(np.mean(chatter_smc), np.std(chatter_smc))
    print(np.mean(chatter_rr), np.std(chatter_rr))

    print("max smc", np.max(np.abs(torque_smc)))
    print("max rr", np.max(np.abs(torque_rr)))

    fig, ax = plt.subplots(1, 1)
    ax.plot(time_smc, chatter_smc, label="smc")
    ax.plot(time_rr, chatter_rr, label="rr")
    ax.legend()

    t1 = torque_smc
    t2 = torque_rr

    plt.show(block=False)
