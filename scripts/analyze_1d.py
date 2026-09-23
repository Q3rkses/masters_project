#!/usr/bin/env python3
"""Plot the filter and smoother results and check their consistency.

usage: analyze_1d.py [data_dir]
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import chi2, norm

TRUTH = "#3d3d3a"
MEASURED = "#c3c2b7"
FILTER = "#2a78d6"
SMOOTHER = "#eb6834"
BAND = "#8a8984"

STATE_DIM = 1        # n, the dof of NEES
MEASUREMENT_DIM = 1  # m, the dof of NIS
CONFIDENCE = 0.95
WINDOWS = (150, 500, 1000)  # trajectories are unreadable at full length
AVERAGING_WINDOW = 50  # raw per-step NIS/NEES is noise; the running mean is readable


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def chi2_interval(dof, samples=1):
    """Two-sided interval for a chi-square statistic averaged over `samples`."""
    tail = (1 - CONFIDENCE) / 2
    low, high = chi2.ppf([tail, 1 - tail], dof * samples)
    return low / samples, high / samples


def new_figure(title, ylabel="position"):
    figure, axis = plt.subplots(figsize=(11, 5))
    axis.set_title(title, loc="left")
    axis.set_xlabel("timestep")
    axis.set_ylabel(ylabel)
    axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
    axis.set_axisbelow(True)
    axis.spines[["top", "right"]].set_visible(False)
    return figure, axis


def draw_measurements(axis, k, z):
    axis.scatter(k, z, s=8, color=MEASURED, label="measurements", zorder=1)


def draw_truth(axis, k, x_true):
    axis.plot(k, x_true, color=TRUTH, linewidth=1.2, label="ground truth", zorder=2)


def draw_estimate(axis, k, x, P, color, label):
    """An estimate as a line with its 95% confidence band."""
    sigma = norm.ppf(0.5 + CONFIDENCE / 2) * np.sqrt(P)
    axis.fill_between(k, x - sigma, x + sigma, color=color, alpha=0.18,
                      linewidth=0, label=f"{label} 95%")
    axis.plot(k, x, color=color, linewidth=1.5, label=label, zorder=3)


def draw_variance(axis, k, P_filter, P_smoother):
    """Filter vs smoother variance, as in Sarkka figure 12.2."""
    axis.plot(k, P_filter, color=FILTER, linewidth=1.5, label="filter variance")
    axis.plot(k, P_smoother, color=SMOOTHER, linewidth=1.5, label="smoother variance")
    axis.set_title("Filter vs smoother variance", loc="left")
    axis.set_ylabel("variance")


def finish(axis):
    axis.legend(loc="best", frameon=False, fontsize=9)


def running_mean(values, window):
    return np.convolve(values, np.ones(window) / window, mode="valid")


def effective_samples(values, count):
    """How many independent samples `count` correlated ones are worth.

    NIS is white so this returns `count`, but NEES errors are autocorrelated
    and a naive count would give a band that is far too narrow.
    """
    rho = np.corrcoef(values[:-1], values[1:])[0, 1]
    return count * (1 - rho) / (1 + rho)


def plot_consistency(axis, k, series, dof, name):
    """Running mean of a NIS or NEES sequence against its acceptance band."""
    samples = min(effective_samples(v, AVERAGING_WINDOW) for v, _ in series.values())
    low, high = chi2_interval(dof, samples=samples)
    axis.axhspan(low, high, color=BAND, alpha=0.15, linewidth=0,
                 label=f"95% band [{low:.2f}, {high:.2f}]")
    axis.axhline(dof, color=BAND, linewidth=1, linestyle="--",
                 label=f"expected value ({dof})")
    for label, (values, color) in series.items():
        axis.plot(k[AVERAGING_WINDOW - 1:], running_mean(values, AVERAGING_WINDOW),
                  color=color, linewidth=1.5, label=label)
    axis.set_title(name, loc="left")
    axis.set_ylabel(f"{name}, {AVERAGING_WINDOW}-step running mean")


def summarise(name, values, dof):
    """Time-averaged consistency, which is a far tighter test than per-timestep."""
    low, high = chi2_interval(dof, samples=effective_samples(values, len(values)))
    step_low, step_high = chi2_interval(dof)
    inside = np.mean((values >= step_low) & (values <= step_high))
    average = values.mean()
    verdict = "consistent" if low <= average <= high else "INCONSISTENT"
    print(f"  {name:<16} {average:6.4f}  band [{low:.4f}, {high:.4f}]  "
          f"{inside:5.1%} of steps in band  -> {verdict}")


def trajectory_figures(window, k, z, x_true, x_filter, P_filter, x_smoother, P_smoother):
    """The four trajectory views over the first `window` timesteps."""
    w = slice(0, window)
    span = f"first {window} timesteps" if window < len(k) else f"all {len(k)} timesteps"
    figures = {}

    figure, axis = new_figure(f"Simulation, {span}")
    draw_measurements(axis, k[w], z[w])
    draw_truth(axis, k[w], x_true[w])
    finish(axis)
    figures[f"1_simulation_{window}.png"] = figure

    figure, axis = new_figure(f"Kalman filter, {span}")
    draw_measurements(axis, k[w], z[w])
    draw_truth(axis, k[w], x_true[w])
    draw_estimate(axis, k[w], x_filter[w], P_filter[w], FILTER, "filter")
    finish(axis)
    figures[f"2_filter_{window}.png"] = figure

    figure, axis = new_figure(f"RTS smoother, {span}")
    draw_measurements(axis, k[w], z[w])
    draw_truth(axis, k[w], x_true[w])
    draw_estimate(axis, k[w], x_smoother[w], P_smoother[w], SMOOTHER, "smoother")
    finish(axis)
    figures[f"3_smoother_{window}.png"] = figure

    figure, axis = new_figure(f"Filter vs smoother, {span}")
    draw_truth(axis, k[w], x_true[w])
    draw_estimate(axis, k[w], x_filter[w], P_filter[w], FILTER, "filter")
    draw_estimate(axis, k[w], x_smoother[w], P_smoother[w], SMOOTHER, "smoother")
    finish(axis)
    figures[f"4_filter_vs_smoother_{window}.png"] = figure

    return figures


def main():
    data_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent.parent / "data"
    figures_dir = data_dir.parent / "figures"
    figures_dir.mkdir(exist_ok=True)
    truth = load(data_dir / "truth.csv")
    filtered = load(data_dir / "filter.csv")
    smoothed = load(data_dir / "smoother.csv")

    k = truth["timestep"]
    x_true, z = truth["x_true_0"], truth["z_0"]
    x_filter, P_filter = filtered["x_updated_0"], filtered["P_updated_0_0"]
    x_smoother, P_smoother = smoothed["x_smoothed_0"], smoothed["P_smoothed_0_0"]

    nis = filtered["innovation_0"] ** 2 / filtered["S_0_0"]
    nees_filter = (x_true - x_filter) ** 2 / P_filter
    nees_smoother = (x_true - x_smoother) ** 2 / P_smoother

    rmse = lambda estimate: np.sqrt(np.mean((x_true - estimate) ** 2))
    print(f"{len(k)} timesteps\n")
    print("consistency (time-averaged):")
    summarise("ANIS", nis, MEASUREMENT_DIM)
    summarise("ANEES filter", nees_filter, STATE_DIM)
    summarise("ANEES smoother", nees_smoother, STATE_DIM)
    print("\naccuracy:")
    print(f"  RMSE filter      {rmse(x_filter):6.4f}   mean P {P_filter.mean():.4f}")
    print(f"  RMSE smoother    {rmse(x_smoother):6.4f}   mean P {P_smoother.mean():.4f}")

    figures = {}
    for window in WINDOWS:
        figures.update(trajectory_figures(
            window, k, z, x_true, x_filter, P_filter, x_smoother, P_smoother))

    figure, axes = plt.subplots(2, 1, figsize=(11, 8))
    plot_consistency(axes[0], k, {"filter": (nis, FILTER)}, MEASUREMENT_DIM, "NIS")
    plot_consistency(axes[1], k, {"filter": (nees_filter, FILTER),
                                  "smoother": (nees_smoother, SMOOTHER)},
                     STATE_DIM, "NEES")
    for axis in axes:
        axis.set_xlabel("timestep")
        axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
        axis.set_axisbelow(True)
        axis.spines[["top", "right"]].set_visible(False)
        finish(axis)
    figures["5_consistency.png"] = figure

    figure, axis = new_figure("Filter vs smoother variance", ylabel="variance")
    draw_variance(axis, k, P_filter, P_smoother)
    finish(axis)
    figures["6_variance.png"] = figure

    always_smaller = np.all(P_smoother <= P_filter + 1e-12)
    final_equal = np.isclose(P_smoother[-1], P_filter[-1])
    print(f"\nsmoother variance <= filter variance at every step: {always_smaller}")
    print(f"equal at the final step: {final_equal}")

    print()
    for name, figure in figures.items():
        figure.tight_layout()
        figure.savefig(figures_dir / name, dpi=150)
        print(f"wrote {figures_dir / name}")
    plt.show()


if __name__ == "__main__":
    main()
