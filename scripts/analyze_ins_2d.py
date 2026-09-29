#!/usr/bin/env python3
"""Plot the 2D strapdown INS filter and smoother results and check their
consistency.

State is [x, y, psi, u, v, b_ax, b_ay, b_gyro]. GNSS arrives at irregular
intervals (see gnss_fixes.csv), not every timestep, so NIS and the
"measurements" shown on a trajectory only ever use the real fix timesteps,
never every row of truth.csv's z column.

usage: analyze_ins_2d.py [data_dir]
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
FIX = "#c0392b"

STATE_DIM = 8
MEASUREMENT_DIM = 2  # GNSS position only
CONFIDENCE = 0.95
WINDOWS = (6000, 20000, 60000)  # ~1 lap, ~3 laps, the full 10-minute run
ELLIPSES = 6  # confidence ellipses drawn along each estimated trajectory
AVERAGING_WINDOW = 50  # raw per-step NIS/NEES is noise; the running mean is readable

# where each part sits in the state [x, y, psi, u, v, b_ax, b_ay, b_gyro]
POSITION = slice(0, 2)
HEADING = 2
VELOCITY = slice(3, 5)
BIAS = slice(5, 8)


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def load_fix_steps(path):
    """The timesteps that got a real GNSS update, as an int array."""
    steps = np.genfromtxt(path, delimiter=",", names=True, dtype=int)
    return np.atleast_1d(steps["timestep"])


def vectors(table, name, dim):
    """(timesteps, dim) array from the columns name_0 .. name_{dim-1}."""
    return np.column_stack([table[f"{name}_{i}"] for i in range(dim)])


def matrices(table, name, dim):
    """(timesteps, dim, dim) array from the row-major columns name_i_j."""
    columns = [table[f"{name}_{i}_{j}"] for i in range(dim) for j in range(dim)]
    return np.column_stack(columns).reshape(-1, dim, dim)


def angle_diff(a, b):
    """a - b, wrapped into [-pi, pi], matching the C++ ssa() convention."""
    return np.arctan2(np.sin(a - b), np.cos(a - b))


def quadratic_form(error, P):
    """error^T P^-1 error at every timestep, which is NEES or NIS."""
    solved = np.linalg.solve(P, error[..., None])[..., 0]
    return np.einsum("ti,ti->t", error, solved)


def masked_quadratic_form(error, P):
    """quadratic_form where P is positive semidefinite, NaN elsewhere.

    The RTS/ERTS smoother's P has no algebraic PSD guarantee (see
    ellipse_shape), so a straight quadratic_form can go negative there,
    which isn't a real NEES value.
    """
    mask = psd_mask(P)
    values = np.full(len(error), np.nan)
    values[mask] = quadratic_form(error[mask], P[mask])
    return values


def chi2_interval(dof, samples=1):
    """Two-sided interval for a chi-square statistic averaged over `samples`."""
    tail = (1 - CONFIDENCE) / 2
    low, high = chi2.ppf([tail, 1 - tail], dof * samples)
    return low / samples, high / samples


def ellipse_shape(P):
    """Width, height and angle in degrees of the confidence ellipse of a 2x2 P.

    The RTS/ERTS smoother's covariance update has no algebraic guarantee of
    staying positive semidefinite (unlike the filter's Joseph-form update),
    so a slightly negative eigenvalue here isn't unusual; clip it to zero
    rather than let it produce a NaN width/height.
    """
    eigenvalues, eigenvectors = np.linalg.eigh(P)  # ascending order
    eigenvalues = np.clip(eigenvalues, 0.0, None)
    scale = chi2.ppf(CONFIDENCE, 2)
    width, height = 2 * np.sqrt(scale * eigenvalues[::-1])
    major = eigenvectors[:, -1]
    return width, height, np.degrees(np.arctan2(major[1], major[0]))


def psd_mask(P):
    """True where a batch of matrices (T, n, n) is positive semidefinite."""
    return np.linalg.eigvalsh(P).min(axis=-1) >= 0


def new_figure(title, xlabel="x position", ylabel="y position", equal=True):
    figure, axis = plt.subplots(figsize=(9, 8) if equal else (11, 5))
    axis.set_title(title, loc="left")
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    if equal:
        axis.set_aspect("equal", adjustable="datalim")
    axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
    axis.set_axisbelow(True)
    axis.spines[["top", "right"]].set_visible(False)
    return figure, axis


def draw_gnss_fixes(axis, z_fix):
    axis.scatter(z_fix[:, 0], z_fix[:, 1], s=18, color=FIX, zorder=4,
                label="GNSS fix", marker="x")


def draw_truth(axis, x_true):
    axis.plot(x_true[:, 0], x_true[:, 1], color=TRUTH, linewidth=1.2,
              label="ground truth", zorder=2)
    axis.scatter(x_true[0, 0], x_true[0, 1], s=30, color=TRUTH, zorder=5, label="start")


def draw_estimate(axis, indices, x, P, color, label):
    """An estimate as a path with 95% position confidence ellipses at a few timesteps."""
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
    """How many independent samples `count` correlated ones are worth."""
    rho = np.corrcoef(values[:-1], values[1:])[0, 1]
    return count * (1 - rho) / (1 + rho)


def plot_consistency(axis, k, series, dof, name):
    """Running mean of a NIS or NEES sequence against its acceptance band."""
    samples = min(effective_samples(v, min(AVERAGING_WINDOW, len(v))) for v, _ in series.values())
    low, high = chi2_interval(dof, samples=samples)
    axis.axhspan(low, high, color=BAND, alpha=0.15, linewidth=0,
                 label=f"95% band [{low:.2f}, {high:.2f}]")
    axis.axhline(dof, color=BAND, linewidth=1, linestyle="--",
                 label=f"expected value ({dof})")
    for label, (values, color) in series.items():
        window = min(AVERAGING_WINDOW, len(values))
        axis.plot(k[window - 1:], running_mean(values, window),
                  color=color, linewidth=1.5, label=label)
    axis.set_title(name, loc="left")
    axis.set_ylabel(f"{name}, {AVERAGING_WINDOW}-step running mean")


def summarise(name, values, dof):
    """Time-averaged consistency, which is a far tighter test than per-timestep.

    values may contain NaN where P wasn't positive semidefinite (see
    masked_quadratic_form); those timesteps are dropped, not counted as
    inconsistent, and how many were dropped is reported.
    """
    valid = values[~np.isnan(values)]
    dropped = len(values) - len(valid)
    low, high = chi2_interval(dof, samples=effective_samples(valid, len(valid)))
    step_low, step_high = chi2_interval(dof)
    inside = np.mean((valid >= step_low) & (valid <= step_high))
    average = valid.mean()
    verdict = "consistent" if low <= average <= high else "INCONSISTENT"
    dropped_note = f"  ({dropped} steps dropped, P not PSD)" if dropped else ""
    print(f"  {name:<20} {average:9.4f}  band [{low:.4f}, {high:.4f}]  "
          f"{inside:5.1%} of steps in band  -> {verdict}{dropped_note}")


def trajectory_figures(window, x_true, z_fix, fix_in_window, x_filter, P_filter,
                       x_smoother, P_smoother):
    """The four trajectory views over the first `window` timesteps."""
    shown = min(window, len(x_true))
    w = slice(0, shown)
    indices = np.linspace(0, shown - 1, ELLIPSES).astype(int)
    span = f"first {shown} timesteps" if shown < len(x_true) else f"all {len(x_true)} timesteps"
    z_w = z_fix[fix_in_window < shown]
    figures = {}

    figure, axis = new_figure(f"Simulation, {span}")
    draw_gnss_fixes(axis, z_w)
    draw_truth(axis, x_true[w])
    finish(axis)
    figures[f"ins_1_simulation_{window}.png"] = figure

    figure, axis = new_figure(f"EKF, {span}")
    draw_gnss_fixes(axis, z_w)
    draw_truth(axis, x_true[w])
    draw_estimate(axis, indices, x_filter[w], P_filter[w], FILTER, "filter")
    finish(axis)
    figures[f"ins_2_filter_{window}.png"] = figure

    figure, axis = new_figure(f"ERTS smoother, {span}")
    draw_gnss_fixes(axis, z_w)
    draw_truth(axis, x_true[w])
    draw_estimate(axis, indices, x_smoother[w], P_smoother[w], SMOOTHER, "smoother")
    finish(axis)
    figures[f"ins_3_smoother_{window}.png"] = figure

    figure, axis = new_figure(f"Filter vs smoother, {span}")
    draw_truth(axis, x_true[w])
    draw_estimate(axis, indices, x_filter[w], P_filter[w], FILTER, "filter")
    draw_estimate(axis, indices, x_smoother[w], P_smoother[w], SMOOTHER, "smoother")
    finish(axis)
    figures[f"ins_4_filter_vs_smoother_{window}.png"] = figure

    return figures


def growth_between_fixes_figure(fix_steps, x_true, x_filter, P_filter, dt):
    """Zooms into the single longest gap between two GNSS fixes, and draws the
    filter's position ellipse growing at several points through it -- the
    filter free-runs on the IMU alone for that whole stretch."""
    gaps = np.diff(fix_steps)
    longest = np.argmax(gaps)
    start, end = fix_steps[longest], fix_steps[longest + 1]
    span = slice(start, end + 1)
    indices = np.linspace(0, end - start, ELLIPSES).astype(int) + start

    figure, axis = new_figure(
        f"Covariance growth between two GNSS fixes ({(end - start) * dt:.1f}s apart)")
    draw_truth(axis, x_true[span])
    draw_estimate(axis, indices - start, x_filter[span], P_filter[span], FILTER, "filter")
    finish(axis)
    return figure


def bias_figure(k, x_true, x_filter, x_smoother):
    """True vs estimated sensor bias over time -- can the filter learn it?"""
    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    labels = ("accelerometer bias x [m/s^2]", "accelerometer bias y [m/s^2]",
             "gyro bias [rad/s]")
    for i, (axis, label) in enumerate(zip(axes, labels)):
        axis.plot(k, x_true[:, 5 + i], color=TRUTH, linewidth=1.2, label="true bias")
        axis.plot(k, x_filter[:, 5 + i], color=FILTER, linewidth=1.2, label="filter")
        axis.plot(k, x_smoother[:, 5 + i], color=SMOOTHER, linewidth=1.2, label="smoother")
        axis.set_ylabel(label)
        axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
        axis.set_axisbelow(True)
        axis.spines[["top", "right"]].set_visible(False)
    axes[0].set_title("Sensor bias: truth vs estimate", loc="left")
    axes[-1].set_xlabel("timestep")
    finish(axes[0])
    return figure


def heading_error_figure(k, heading_error_filter, heading_error_smoother):
    figure, axis = new_figure("Heading error", xlabel="timestep",
                              ylabel="heading error [rad]", equal=False)
    axis.axhline(0, color=BAND, linewidth=1, linestyle="--")
    axis.plot(k, heading_error_filter, color=FILTER, linewidth=1, label="filter")
    axis.plot(k, heading_error_smoother, color=SMOOTHER, linewidth=1, label="smoother")
    finish(axis)
    return figure


def position_error_figure(k, position_error_filter, position_error_smoother, fix_steps):
    figure, axis = new_figure("Position error, with GNSS fixes marked",
                              xlabel="timestep", ylabel="position error [m]", equal=False)
    for count, step in enumerate(fix_steps):
        axis.axvline(step, color=FIX, alpha=0.25, linewidth=0.8,
                     label="GNSS fix" if count == 0 else None)
    axis.plot(k, position_error_filter, color=FILTER, linewidth=1, label="filter")
    axis.plot(k, position_error_smoother, color=SMOOTHER, linewidth=1, label="smoother")
    finish(axis)
    return figure


def main():
    data_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent.parent / "data"
    figures_dir = data_dir.parent / "figures"
    figures_dir.mkdir(exist_ok=True)
    truth = load(data_dir / "truth.csv")
    filtered = load(data_dir / "filter.csv")
    smoothed = load(data_dir / "smoother.csv")
    fix_steps = load_fix_steps(data_dir / "gnss_fixes.csv")

    k = truth["timestep"].astype(int)
    x_true = vectors(truth, "x_true", STATE_DIM)
    z = vectors(truth, "z", MEASUREMENT_DIM)
    z_fix = z[fix_steps]

    x_filter = vectors(filtered, "x_updated", STATE_DIM)
    P_filter = matrices(filtered, "P_updated", STATE_DIM)
    innovation = vectors(filtered, "innovation", MEASUREMENT_DIM)
    S = matrices(filtered, "S", MEASUREMENT_DIM)
    x_smoother = vectors(smoothed, "x_smoothed", STATE_DIM)
    P_smoother = matrices(smoothed, "P_smoothed", STATE_DIM)

    # NIS only means anything at the timesteps a measurement actually arrived;
    # S is exactly singular everywhere else (see gnss_fixes.csv docstring above)
    nis = quadratic_form(innovation[fix_steps], S[fix_steps])

    def position_nees(x_est, P_est):
        return masked_quadratic_form(x_true[:, POSITION] - x_est[:, POSITION], P_est[:, :2, :2])

    def heading_nees(x_est, P_est):
        error = angle_diff(x_true[:, HEADING], x_est[:, HEADING])
        variance = P_est[:, 2, 2]
        return np.where(variance >= 0, error ** 2 / variance, np.nan)

    def velocity_nees(x_est, P_est):
        return masked_quadratic_form(x_true[:, VELOCITY] - x_est[:, VELOCITY], P_est[:, 3:5, 3:5])

    def bias_nees(x_est, P_est):
        return masked_quadratic_form(x_true[:, BIAS] - x_est[:, BIAS], P_est[:, 5:8, 5:8])

    position_error_filter = np.linalg.norm(x_true[:, POSITION] - x_filter[:, POSITION], axis=1)
    position_error_smoother = np.linalg.norm(x_true[:, POSITION] - x_smoother[:, POSITION], axis=1)
    heading_error_filter = angle_diff(x_true[:, HEADING], x_filter[:, HEADING])
    heading_error_smoother = angle_diff(x_true[:, HEADING], x_smoother[:, HEADING])

    print(f"{len(k)} timesteps, {len(fix_steps)} GNSS fixes\n")
    print("consistency (time-averaged):")
    summarise("ANIS", nis, MEASUREMENT_DIM)
    for name, dof, fn in (("position", 2, position_nees), ("heading", 1, heading_nees),
                          ("velocity", 2, velocity_nees), ("bias", 3, bias_nees)):
        summarise(f"ANEES {name} filter", fn(x_filter, P_filter), dof)
        summarise(f"ANEES {name} smoother", fn(x_smoother, P_smoother), dof)
    print("\naccuracy:")
    print(f"  RMSE position filter    {np.sqrt(np.mean(position_error_filter ** 2)):6.4f} m")
    print(f"  RMSE position smoother  {np.sqrt(np.mean(position_error_smoother ** 2)):6.4f} m")

    figures = {}
    for window in WINDOWS:
        figures.update(trajectory_figures(
            window, x_true, z_fix, fix_steps, x_filter, P_filter, x_smoother, P_smoother))

    dt = 0.01  # not in any CSV; keep in sync with the config's dt, only used for a label
    figures["ins_5_covariance_growth.png"] = growth_between_fixes_figure(
        fix_steps, x_true, x_filter, P_filter, dt)
    figures["ins_6_bias.png"] = bias_figure(k, x_true, x_filter, x_smoother)
    figures["ins_7_heading_error.png"] = heading_error_figure(
        k, heading_error_filter, heading_error_smoother)
    figures["ins_8_position_error.png"] = position_error_figure(
        k, position_error_filter, position_error_smoother, fix_steps)

    # running-mean plots can't handle NaN (see masked_quadratic_form); zero is
    # a display compromise, the printed summary above is the honest number
    figure, axes = plt.subplots(2, 1, figsize=(11, 8))
    plot_consistency(axes[0], k[fix_steps], {"filter": (nis, FILTER)}, MEASUREMENT_DIM, "NIS")
    plot_consistency(axes[1], k, {
        "filter": (np.nan_to_num(position_nees(x_filter, P_filter)), FILTER),
        "smoother": (np.nan_to_num(position_nees(x_smoother, P_smoother)), SMOOTHER)},
        2, "position NEES")
    for axis in axes:
        axis.set_xlabel("timestep")
        axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
        axis.set_axisbelow(True)
        axis.spines[["top", "right"]].set_visible(False)
        finish(axis)
    figures["ins_9_consistency.png"] = figure

    print()
    for name, figure in figures.items():
        figure.tight_layout()
        figure.savefig(figures_dir / name, dpi=150)
        print(f"wrote {figures_dir / name}")
    plt.show()


if __name__ == "__main__":
    main()
