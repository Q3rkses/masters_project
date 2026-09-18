#!/usr/bin/env python3
"""Plot the 2D filter and smoother results and check their consistency.

Assumes state components 0 and 1 are the x and y position.

usage: analyze_2d.py [data_dir]
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse
from scipy.stats import chi2

TRUTH = "#3d3d3a"
MEASURED = "#c3c2b7"
FILTER = "#2a78d6"
SMOOTHER = "#eb6834"
BAND = "#8a8984"

STATE_DIM = 2        # n, the dof of NEES
MEASUREMENT_DIM = 2  # m, the dof of NIS
CONFIDENCE = 0.95
WINDOWS = (50, 100, 200)  # trajectories are unreadable at full length
ELLIPSES = 6  # confidence ellipses drawn along each estimated trajectory
AVERAGING_WINDOW = 50  # raw per-step NIS/NEES is noise; the running mean is readable


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def vectors(table, name, dim):
    """(timesteps, dim) array from the columns name_0 .. name_{dim-1}."""
    return np.column_stack([table[f"{name}_{i}"] for i in range(dim)])


def matrices(table, name, dim):
    """(timesteps, dim, dim) array from the row-major columns name_i_j."""
    columns = [table[f"{name}_{i}_{j}"] for i in range(dim) for j in range(dim)]
    return np.column_stack(columns).reshape(-1, dim, dim)


def quadratic_form(error, P):
    """error^T P^-1 error at every timestep, which is NEES or NIS."""
    solved = np.linalg.solve(P, error[..., None])[..., 0]
    return np.einsum("ti,ti->t", error, solved)


def chi2_interval(dof, samples=1):
    """Two-sided interval for a chi-square statistic averaged over `samples`."""
    tail = (1 - CONFIDENCE) / 2
    low, high = chi2.ppf([tail, 1 - tail], dof * samples)
    return low / samples, high / samples


def ellipse_shape(P):
    """Width, height and angle in degrees of the confidence ellipse of a 2x2 P."""
    eigenvalues, eigenvectors = np.linalg.eigh(P)  # ascending order
    scale = chi2.ppf(CONFIDENCE, 2)
    width, height = 2 * np.sqrt(scale * eigenvalues[::-1])
    major = eigenvectors[:, -1]
    return width, height, np.degrees(np.arctan2(major[1], major[0]))


def new_figure(title):
    figure, axis = plt.subplots(figsize=(9, 8))
    axis.set_title(title, loc="left")
    axis.set_xlabel("x position")
    axis.set_ylabel("y position")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
    axis.set_axisbelow(True)
    axis.spines[["top", "right"]].set_visible(False)
    return figure, axis


def draw_measurements(axis, z):
    axis.scatter(z[:, 0], z[:, 1], s=8, color=MEASURED, label="measurements", zorder=1)


def draw_truth(axis, x_true):
    axis.plot(x_true[:, 0], x_true[:, 1], color=TRUTH, linewidth=1.2,
              label="ground truth", zorder=2)
    axis.scatter(x_true[0, 0], x_true[0, 1], s=30, color=TRUTH, zorder=4, label="start")


def draw_estimate(axis, indices, x, P, color, label):
    """An estimate as a path with 95% confidence ellipses at a few timesteps."""
    axis.plot(x[:, 0], x[:, 1], color=color, linewidth=1.5, label=label, zorder=3)
    for count, k in enumerate(indices):
        width, height, angle = ellipse_shape(P[k, :2, :2])
        axis.add_patch(Ellipse(
            x[k, :2], width, height, angle=angle, facecolor=color, edgecolor=color,
            alpha=0.18, linewidth=1, zorder=2,
            label=f"{label} 95%" if count == 0 else None))


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


def trajectory_figures(window, z, x_true, x_filter, P_filter, x_smoother, P_smoother):
    """The four trajectory views over the first `window` timesteps."""
    shown = min(window, len(z))
    w = slice(0, shown)
    indices = np.linspace(0, shown - 1, ELLIPSES).astype(int)
    span = f"first {shown} timesteps" if shown < len(z) else f"all {len(z)} timesteps"
    figures = {}

    figure, axis = new_figure(f"Simulation, {span}")
    draw_measurements(axis, z[w])
    draw_truth(axis, x_true[w])
    finish(axis)
    figures[f"2d_1_simulation_{window}.png"] = figure

    figure, axis = new_figure(f"Kalman filter, {span}")
    draw_measurements(axis, z[w])
    draw_truth(axis, x_true[w])
    draw_estimate(axis, indices, x_filter[w], P_filter[w], FILTER, "filter")
    finish(axis)
    figures[f"2d_2_filter_{window}.png"] = figure

    figure, axis = new_figure(f"RTS smoother, {span}")
    draw_measurements(axis, z[w])
    draw_truth(axis, x_true[w])
    draw_estimate(axis, indices, x_smoother[w], P_smoother[w], SMOOTHER, "smoother")
    finish(axis)
    figures[f"2d_3_smoother_{window}.png"] = figure

    figure, axis = new_figure(f"Filter vs smoother, {span}")
    draw_truth(axis, x_true[w])
    draw_estimate(axis, indices, x_filter[w], P_filter[w], FILTER, "filter")
    draw_estimate(axis, indices, x_smoother[w], P_smoother[w], SMOOTHER, "smoother")
    finish(axis)
    figures[f"2d_4_filter_vs_smoother_{window}.png"] = figure

    return figures


def main():
    data_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent.parent / "data"
    figures_dir = data_dir.parent / "figures"
    figures_dir.mkdir(exist_ok=True)
    truth = load(data_dir / "truth.csv")
    filtered = load(data_dir / "filter.csv")
    smoothed = load(data_dir / "smoother.csv")

    k = truth["timestep"]
    x_true = vectors(truth, "x_true", STATE_DIM)
    z = vectors(truth, "z", MEASUREMENT_DIM)
    x_filter = vectors(filtered, "x_updated", STATE_DIM)
    P_filter = matrices(filtered, "P_updated", STATE_DIM)
    innovation = vectors(filtered, "innovation", MEASUREMENT_DIM)
    S = matrices(filtered, "S", MEASUREMENT_DIM)
    x_smoother = vectors(smoothed, "x_smoothed", STATE_DIM)
    P_smoother = matrices(smoothed, "P_smoothed", STATE_DIM)

    nis = quadratic_form(innovation, S)
    nees_filter = quadratic_form(x_true - x_filter, P_filter)
    nees_smoother = quadratic_form(x_true - x_smoother, P_smoother)

    rmse = lambda estimate: np.sqrt(np.mean(np.sum((x_true - estimate) ** 2, axis=1)))
    mean_trace = lambda P: np.trace(P, axis1=1, axis2=2).mean()
    print(f"{len(k)} timesteps\n")
    print("consistency (time-averaged):")
    summarise("ANIS", nis, MEASUREMENT_DIM)
    summarise("ANEES filter", nees_filter, STATE_DIM)
    summarise("ANEES smoother", nees_smoother, STATE_DIM)
    print("\naccuracy:")
    print(f"  RMSE filter      {rmse(x_filter):6.4f}   mean trace P {mean_trace(P_filter):.4f}")
    print(f"  RMSE smoother    {rmse(x_smoother):6.4f}   mean trace P {mean_trace(P_smoother):.4f}")

    figures = {}
    for window in WINDOWS:
        figures.update(trajectory_figures(
            window, z, x_true, x_filter, P_filter, x_smoother, P_smoother))

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
    figures["2d_5_consistency.png"] = figure

    print()
    for name, figure in figures.items():
        figure.tight_layout()
        figure.savefig(figures_dir / name, dpi=150)
        print(f"wrote {figures_dir / name}")
    plt.show()


if __name__ == "__main__":
    main()
