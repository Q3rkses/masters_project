#!/usr/bin/env python3
"""Figures and consistency statistics for one 2-D strapdown INS run.

usage: analyze_ins_2d.py <case_dir>/data

Reads truth.csv, filter.csv, smoother.csv and the three *_fixes.csv files, and
config.yaml from the case folder for the time step and the GNSS outage window.
The printed statistics are also written to <case_dir>/stats.txt and the
figures to <case_dir>/figures. Each case folder gets six figures for the EKF
and its ERTS smoother:

    1_scenario.png      the true path and when each sensor fired
    2_path.png          path with both estimates and a zoom on the turn with the largest error
    3_errors.png        error with 95% bands
    4_uncertainty.png   the position spread the estimators report
    5_biases.png        true and estimated sensor biases
    6_consistency.png   NIS per sensor and the running NEES

A statistic that falls above its 95% interval means the errors are larger than
the reported covariance admits, the estimator is overconfident. Below the
interval it is underconfident, the covariance is larger than it needs to be.

In the folders of the ukf and iekf experiments the UKF/URTSS and IEKF/ERTS
results are analysed next to the EKF/ERTS. Those folders get figures 1 to 4
and two more instead of 5 and 6, since the biases and the consistency are
compared in the statistics:

    5_variants.png        errors, gyro bias and distances of all variants
    6_errors_variant.png  error figure of the experiment's own variant

State: [x, y, psi, u, v, b_ax, b_ay, b_gyro].
"""

import re
import sys
from collections import namedtuple
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, Rectangle
from scipy.stats import chi2, norm

TRUTH = "#3d3d3a"
BAND = "#8a8984"
EKF = "#2a78d6"
ERTS = "#eb6834"
UKF = "#2e9e5b"
URTSS = "#b8860b"
IEKF = "#7f5539"
IERTS = "#d98c5f"
GNSS = "#c0392b"
MAGNETOMETER = "#8e44ad"
DVL = "#16a085"
DENIED = "#f2c48d"

STATE_DIM = 8
POSITION = slice(0, 2)
HEADING = 2
SENSOR_DIMS = {"GNSS": 2, "magnetometer": 1, "DVL": 2}
SENSOR_COLORS = {"GNSS": GNSS, "magnetometer": MAGNETOMETER, "DVL": DVL}
CONFIDENCE = 0.95
AVERAGING_WINDOW = 150
ELLIPSE_COUNT = 5
SKIP_STEPS = 100  # initial transient left out when choosing axis limits
TURN_FRACTION = 0.5  # a turn is where the yaw rate is above this share of the largest one
ZOOM_PAD_SECONDS = 3.0  # path shown before and after a turn in a zoom
START_SECONDS = 10.0  # turns before this are left out of the zoom choice, the prior still dominates
ARROW_COUNT = 10  # direction arrows drawn along the true path

Estimator = namedtuple("Estimator", "name color x P")
Turn = namedtuple("Turn", "first last direction centre")  # direction: +1 left, -1 right

VARIANT_FOLDERS = {"ukf": "UKF", "iekf": "IEKF"}


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def vectors(table, name, dim):
    return np.column_stack([table[f"{name}_{i}"] for i in range(dim)])


def matrices(table, name, dim):
    columns = [table[f"{name}_{i}_{j}"] for i in range(dim) for j in range(dim)]
    return np.column_stack(columns).reshape(-1, dim, dim)


def angle_diff(a, b):
    return np.arctan2(np.sin(a - b), np.cos(a - b))


def quadratic_form(error, P):
    solved = np.linalg.solve(P, error[..., None])[..., 0]
    return np.einsum("ti,ti->t", error, solved)


def psd_mask(P):
    return np.linalg.eigvalsh(P).min(axis=-1) >= 0


def masked_quadratic_form(error, P):
    """NEES where P is positive semidefinite and NaN elsewhere."""
    mask = psd_mask(P)
    values = np.full(len(error), np.nan)
    values[mask] = quadratic_form(error[mask], P[mask])
    return values


def chi2_interval(dof, samples=1.0):
    tail = (1 - CONFIDENCE) / 2
    low, high = chi2.ppf([tail, 1 - tail], dof * samples)
    return low / samples, high / samples


def ellipse_shape(P):
    eigenvalues, eigenvectors = np.linalg.eigh(P)
    eigenvalues = np.clip(eigenvalues, 0.0, None)
    scale = chi2.ppf(CONFIDENCE, 2)
    width, height = 2 * np.sqrt(scale * eigenvalues[::-1])
    major = eigenvectors[:, -1]
    return width, height, np.degrees(np.arctan2(major[1], major[0]))


def running_mean(values, window):
    return np.convolve(values, np.ones(window) / window, mode="valid")


def effective_samples(values):
    rho = np.corrcoef(values[:-1], values[1:])[0, 1]
    return len(values) * (1 - rho) / (1 + rho)


def time_averaged(values, dof):
    """Mean of a NEES or NIS series and the 95% interval it is held against.

    The interval uses the number of independent samples the series is worth,
    estimated from its lag-one autocorrelation and never less than one.
    """
    valid = values[~np.isnan(values)]
    if len(valid) < 3:
        return None
    samples = max(1.0, effective_samples(valid))
    low, high = chi2_interval(dof, samples)
    step_low, step_high = chi2_interval(dof)
    average = valid.mean()
    return {
        "average": average,
        "low": low,
        "high": high,
        "inside": np.mean((valid >= step_low) & (valid <= step_high)),
        "dropped": len(values) - len(valid),
        "verdict": ("underconfident" if average < low
                    else "overconfident" if average > high else "consistent"),
    }


def read_run_settings(case_dir):
    """Time step and GNSS outage window from the case's config.yaml."""
    settings = {"dt": 0.01, "denied": None}
    path = case_dir / "config.yaml"
    if not path.exists():
        return settings
    text = path.read_text()

    def number(key):
        match = re.search(rf"^{key}:\s*([-+0-9.eE]+)", text, flags=re.M)
        return float(match.group(1)) if match else None

    dt = number("dt")
    start, end = number("gnss_denied_start"), number("gnss_denied_end")
    if dt:
        settings["dt"] = dt
    if start is not None and end is not None and end > start and start < 1e8:
        settings["denied"] = (start, end)
    return settings


def style_axis(axis):
    axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
    axis.set_axisbelow(True)
    axis.spines[["top", "right"]].set_visible(False)


def shade_denied(axis, denied):
    if denied:
        axis.axvspan(*denied, color=DENIED, alpha=0.2, linewidth=0, zorder=0)


def limit(axis, curves, pad=1.25):
    """Symmetric y limits from the values after the initial transient."""
    peak = max(np.nanmax(np.abs(c[SKIP_STEPS:])) for c in curves)
    axis.set_ylim(-pad * peak, pad * peak)


def sensor_times(fixes, dt):
    return fixes["timestep"].astype(int) * dt


def fit_equal_axes(axis, curves, margin=3.0):
    """Equal axis scaling with the limits just around the curves, so the plot
    box takes the shape of the path instead of leaving empty bands."""
    points = np.vstack(curves)
    low, high = points.min(axis=0) - margin, points.max(axis=0) + margin
    axis.set_xlim(low[0], high[0])
    axis.set_ylim(low[1], high[1])
    axis.set_aspect("equal", adjustable="box")
    axis.set_anchor("N")


def find_turns(x_true, dt):
    """The stretches of the true path where the vehicle is turning, from the
    yaw rate of the true heading. A turn is where the rate is above
    TURN_FRACTION of its largest value."""
    rate = angle_diff(x_true[1:, HEADING], x_true[:-1, HEADING]) / dt
    rate = np.append(rate, rate[-1])
    turning = np.abs(rate) > TURN_FRACTION * np.abs(rate).max()
    turns, first = [], None
    for k in range(len(turning) + 1):
        inside = k < len(turning) and turning[k]
        if inside and first is None:
            first = k
        elif not inside and first is not None:
            last = k - 1
            if (last - first) * dt >= 0.5:
                direction = int(np.sign(rate[first:last + 1].mean()))
                turns.append(Turn(first, last, direction,
                                  x_true[first:last + 1, POSITION].mean(axis=0)))
            first = None
    return turns


def choose_turn(turns, error, dt):
    """The turn with the largest filter error, leaving out the first seconds
    where the prior still dominates. None if the path has no such turn."""
    candidates = [turn for turn in turns if turn.first * dt >= START_SECONDS]
    if not candidates:
        return None
    return max(candidates, key=lambda turn: error[turn.first:turn.last + 1].max())


def draw_zoom(axis, turn, filt, smooth, x_true, dt):
    """One turn with a little path before and after, both estimates and their
    95% position ellipses. Returns the rectangle the zoom covers."""
    pad = int(ZOOM_PAD_SECONDS / dt)
    lo, hi = max(turn.first - pad, 0), min(turn.last + pad, len(x_true) - 1) + 1
    extent = [x_true[lo:hi, POSITION]]
    ellipses = []
    for estimator in (filt, smooth):
        for k in np.linspace(lo, hi - 1, ELLIPSE_COUNT).astype(int):
            width, height, angle = ellipse_shape(estimator.P[k, :2, :2])
            ellipses.append((estimator, k, width, height, angle))
            reach = max(width, height) / 2
            extent.append(estimator.x[k, :2] + np.array([[reach, reach], [-reach, -reach]]))
        extent.append(estimator.x[lo:hi, POSITION])
    corners = np.vstack(extent)
    low, high = corners.min(axis=0) - 1.0, corners.max(axis=0) + 1.0

    axis.plot(*x_true[lo:hi, POSITION].T, color=TRUTH, linewidth=1.8, label="truth", zorder=3)
    for estimator in (filt, smooth):
        axis.plot(*estimator.x[lo:hi, POSITION].T, color=estimator.color, linewidth=1.4,
                  label=estimator.name, zorder=4)
    for estimator, k, width, height, angle in ellipses:
        axis.add_patch(Ellipse(estimator.x[k, :2], width, height, angle=angle,
                               facecolor=estimator.color, edgecolor=estimator.color,
                               alpha=0.2, linewidth=1, zorder=2))
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlim(low[0], high[0])
    axis.set_ylim(low[1], high[1])
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    kind = "left turn" if turn.direction > 0 else "right turn"
    axis.set_title(f"Zoom on a {kind}, t = {lo * dt:.0f} to {hi * dt:.0f} s, 95% ellipses", loc="left")
    axis.legend(loc="best", frameon=False, fontsize=8)
    style_axis(axis)
    return low, high


def path_figure(x_true, filt, smooth, turns, dt):
    """The path with both estimates, and a large zoom on the turn where the
    filter's position error is largest."""
    figure = plt.figure(figsize=(14, 7.5))
    grid = figure.add_gridspec(1, 2, width_ratios=[1.1, 1], wspace=0.15)
    overview = figure.add_subplot(grid[0, 0])
    zoom = figure.add_subplot(grid[0, 1])

    overview.plot(*x_true[:, POSITION].T, color=TRUTH, linewidth=1.8, label="truth", zorder=3)
    overview.plot(*filt.x[:, POSITION].T, color=filt.color, linewidth=1.0, label=filt.name, zorder=4)
    overview.plot(*smooth.x[:, POSITION].T, color=smooth.color, linewidth=1.0, label=smooth.name, zorder=4)
    fit_equal_axes(overview, [x_true[:, POSITION], filt.x[SKIP_STEPS:, POSITION], smooth.x[SKIP_STEPS:, POSITION]])
    overview.scatter(*x_true[0, POSITION], s=45, color=TRUTH, zorder=6)
    overview.annotate("", xy=x_true[200, POSITION], xytext=x_true[0, POSITION],
                      arrowprops=dict(arrowstyle="-|>", color=TRUTH, lw=1.5), zorder=6)
    overview.set_xlabel("x [m]")
    overview.set_ylabel("y [m]")
    overview.set_title("Path and estimates", loc="left")
    overview.legend(loc="best", frameon=False, fontsize=9)
    style_axis(overview)

    error = np.linalg.norm(filt.x[:, POSITION] - x_true[:, POSITION], axis=1)
    turn = choose_turn(turns, error, dt)
    if turn is None:
        zoom.remove()
        return figure
    low, high = draw_zoom(zoom, turn, filt, smooth, x_true, dt)
    overview.add_patch(Rectangle(low, *(high - low), fill=False, edgecolor=BAND,
                                 linewidth=1.2, zorder=1))
    return figure


def scenario_figure(t, x_true, schedule, dt):
    """The true path alone, with its direction, and when each sensor fired."""
    figure = plt.figure(figsize=(13, 9))
    grid = figure.add_gridspec(2, 1, height_ratios=[3, 1.1], hspace=0.3)
    path = figure.add_subplot(grid[0])
    timeline = figure.add_subplot(grid[1])

    path.plot(*x_true[:, POSITION].T, color=TRUTH, linewidth=1.8, label="true path", zorder=3)
    steps = np.linspace(0, len(x_true) - 30, ARROW_COUNT).astype(int)
    for k in steps:
        path.annotate("", xy=x_true[k + 25, POSITION], xytext=x_true[k, POSITION],
                      arrowprops=dict(arrowstyle="-|>", color=TRUTH, lw=1.2, mutation_scale=14),
                      zorder=5)
    path.scatter(*x_true[0, POSITION], s=55, color=TRUTH, zorder=6, label="start")
    fit_equal_axes(path, [x_true[:, POSITION]])
    path.set_xlabel("x [m]")
    path.set_ylabel("y [m]")
    path.set_title("True path", loc="left")
    path.legend(loc="best", frameon=False, fontsize=9)
    style_axis(path)

    for row, (name, due) in enumerate(schedule.items()):
        times = np.flatnonzero(due) * dt
        timeline.eventplot(times, lineoffsets=row, linelengths=0.7, linewidths=0.7,
                           color=SENSOR_COLORS[name])
        timeline.text(t[-1] * 1.005, row, f"{len(times)} fixes", va="center", fontsize=8, color=BAND)
    timeline.set_yticks(range(len(schedule)))
    timeline.set_yticklabels(list(schedule))
    timeline.set_ylim(-0.6, len(schedule) - 0.4)
    timeline.invert_yaxis()
    timeline.set_xlim(0, t[-1])
    timeline.set_xlabel("time [s]")
    timeline.set_title("When each sensor fired", loc="left")
    style_axis(timeline)
    return figure


def errors_figure(t, x_true, filt, smooth, denied):
    """Error against truth with each estimator's own 95% band."""
    scale = norm.ppf(0.5 + CONFIDENCE / 2)
    rows = (("x error [m]", 0, 1.0), ("y error [m]", 1, 1.0),
            ("heading error [deg]", HEADING, 180 / np.pi))
    figure, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
    for axis, (label, i, unit) in zip(axes, rows):
        curves = []
        for estimator in (filt, smooth):
            diff = (angle_diff(estimator.x[:, i], x_true[:, i]) if i == HEADING
                    else estimator.x[:, i] - x_true[:, i]) * unit
            sigma = scale * np.sqrt(np.clip(estimator.P[:, i, i], 0, None)) * unit
            axis.fill_between(t, -sigma, sigma, color=estimator.color, alpha=0.15, linewidth=0)
            axis.plot(t, diff, color=estimator.color, linewidth=1, label=estimator.name)
            curves += [diff, sigma]
        axis.axhline(0, color=BAND, linewidth=0.8, linestyle="--")
        shade_denied(axis, denied)
        limit(axis, curves)
        axis.set_ylabel(label)
        style_axis(axis)
    axes[-1].set_xlabel("time [s]")
    axes[0].set_title("Error with 95% confidence band, shaded where GNSS is denied", loc="left")
    axes[0].legend(loc="upper right", frameon=False, ncol=2, fontsize=9)
    figure.tight_layout()
    return figure


def uncertainty_figure(t, filt, smooth, denied):
    """The position spread the estimators report about themselves."""
    figure, axis = plt.subplots(figsize=(11, 4.5))
    for estimator in (filt, smooth):
        axis.plot(t, np.sqrt(np.trace(estimator.P[:, :2, :2], axis1=1, axis2=2)),
                  color=estimator.color, linewidth=1.4, label=estimator.name)
    shade_denied(axis, denied)
    axis.set_yscale("log")
    axis.set_ylabel(r"$\sqrt{\mathrm{tr}\,P_{xy}}$ [m]")
    axis.set_xlabel("time [s]")
    axis.set_title("Reported position spread, shaded where GNSS is denied", loc="left")
    axis.legend(loc="upper right", frameon=False, ncol=2, fontsize=9)
    style_axis(axis)
    figure.tight_layout()
    return figure


def biases_figure(t, x_true, estimators, denied):
    figure, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    labels = ("accelerometer bias $b_{a,x}$ [m/s$^2$]", "accelerometer bias $b_{a,y}$ [m/s$^2$]",
              "gyro bias $b_g$ [rad/s]")
    for i, (axis, label) in enumerate(zip(axes, labels)):
        axis.plot(t, x_true[:, 5 + i], color=TRUTH, linewidth=1.4, label="truth")
        for estimator in estimators:
            axis.plot(t, estimator.x[:, 5 + i], color=estimator.color, linewidth=1.1,
                      label=estimator.name)
        shade_denied(axis, denied)
        axis.set_ylabel(label)
        style_axis(axis)
    axes[0].set_title("Bias estimates against truth", loc="left")
    axes[0].legend(loc="best", frameon=False, ncol=len(estimators) + 1, fontsize=9)
    axes[-1].set_xlabel("time [s]")
    figure.tight_layout()
    return figure


def consistency_figure(t, nis, nees, summaries):
    """NIS of every sensor that fired, and the running position NEES. The
    time-averaged value and its verdict are written into each panel."""
    rows = len(nis) + 1
    figure, axes = plt.subplots(rows, 1, figsize=(11, 2.6 * rows + 0.6), sharex=True)
    for axis, (name, (times, values, color)) in zip(axes, nis.items()):
        dof = SENSOR_DIMS[name]
        low, high = chi2_interval(dof)
        axis.axhspan(low, high, color=BAND, alpha=0.15, linewidth=0)
        axis.axhline(dof, color=BAND, linewidth=1, linestyle="--")
        axis.scatter(times, values, s=7, color=color, alpha=0.7, linewidths=0)
        axis.set_ylim(0, max(high * 1.6, np.percentile(values, 99.5)))
        axis.set_ylabel(f"{name} NIS")
        axis.text(0.01, 0.95, summaries[name], transform=axis.transAxes, va="top", fontsize=9)
        style_axis(axis)
    axis = axes[-1]
    low, high = chi2_interval(2)
    axis.axhspan(low, high, color=BAND, alpha=0.15, linewidth=0)
    axis.axhline(2, color=BAND, linewidth=1, linestyle="--")
    for label, color, values in nees:
        values = np.nan_to_num(values)
        axis.plot(t[AVERAGING_WINDOW - 1:], running_mean(values, AVERAGING_WINDOW),
                  color=color, linewidth=1.3, label=f"{label}, {summaries[label]}")
    axis.set_ylabel(f"position NEES\n({AVERAGING_WINDOW}-step mean)")
    axis.set_xlabel("time [s]")
    axis.legend(loc="best", frameon=False, fontsize=9)
    style_axis(axis)
    axes[0].set_title("NIS per fix and running position NEES; shaded: 95% interval of one sample, "
                      "dashed: expected value", loc="left")
    figure.tight_layout()
    return figure


def variants_figure(t, x_true, filters, smoothers, denied):
    """Filters and smoothers of all variants against the EKF and ERTS."""
    figure, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    position_error = lambda e: np.linalg.norm(e.x[:, POSITION] - x_true[:, POSITION], axis=1)

    def draw(axis, group, series):
        """The variants lie on top of the reference almost exactly, so the
        reference is drawn wide underneath and the others dashed over it."""
        for index, estimator in enumerate(group):
            if index == 0:
                axis.plot(t, series(estimator), color=estimator.color, linewidth=3.5, alpha=0.6,
                          label=estimator.name, zorder=2)
            else:
                axis.plot(t, series(estimator), color=estimator.color, linewidth=1.1,
                          linestyle="--", label=estimator.name, zorder=3)

    draw(axes[0], filters, position_error)
    draw(axes[1], smoothers, position_error)
    axes[2].plot(t, x_true[:, 7], color=TRUTH, linewidth=1.4, label="truth", zorder=4)
    draw(axes[2], filters, lambda e: e.x[:, 7])
    for reference, others in ((filters[0], filters[1:]), (smoothers[0], smoothers[1:])):
        for other in others:
            difference = np.linalg.norm(other.x[:, POSITION] - reference.x[:, POSITION], axis=1)
            axes[3].plot(t, np.where(difference > 0, difference, np.nan), color=other.color,
                         linewidth=1, label=f"{other.name} vs {reference.name}")
    axes[0].set_ylabel("filter position\nerror [m]")
    axes[1].set_ylabel("smoother position\nerror [m]")
    axes[2].set_ylabel("gyro bias [rad/s]")
    axes[3].set_ylabel("distance to the EKF-based\nestimate [m]")
    axes[3].set_yscale("log")
    positive = [d for line in axes[3].get_lines() for d in line.get_ydata()[SKIP_STEPS:] if d > 0]
    if positive:
        axes[3].set_ylim(bottom=0.5 * min(positive))
    axes[3].set_xlabel("time [s]")
    for axis in axes:
        shade_denied(axis, denied)
        axis.legend(loc="upper right", frameon=False, fontsize=8, ncol=3)
        style_axis(axis)
    axes[0].set_title("Estimator variants", loc="left")
    figure.tight_layout()
    return figure


def sensor_fix_data(data_dir, prefix):
    names = ("gnss_fixes", "magnetometer_fixes", "dvl_fixes")
    paths = [data_dir / f"{prefix}{name}.csv" for name in names]
    if not all(path.exists() for path in paths):
        return None
    return [load(path) for path in paths]


def nis_of(fixes):
    gnss, magnetometer, dvl = fixes
    return {
        "GNSS": quadratic_form(vectors(gnss, "innovation", 2), matrices(gnss, "S", 2)),
        "magnetometer": quadratic_form(vectors(magnetometer, "innovation", 1), matrices(magnetometer, "S", 1)),
        "DVL": quadratic_form(vectors(dvl, "innovation", 2), matrices(dvl, "S", 2)),
    }


def load_estimators(data_dir, with_variants):
    filtered = load(data_dir / "filter.csv")
    smoothed = load(data_dir / "smoother.csv")
    filters = [Estimator("EKF", EKF, vectors(filtered, "x_updated", STATE_DIM),
                         matrices(filtered, "P_updated", STATE_DIM))]
    smoothers = [Estimator("ERTS", ERTS, vectors(smoothed, "x_smoothed", STATE_DIM),
                           matrices(smoothed, "P_smoothed", STATE_DIM))]
    prefixes = {"EKF": ""}
    if with_variants:
        for prefix, filter_name, smoother_name, f_color, s_color in (
                ("ukf_", "UKF", "URTSS", UKF, URTSS), ("iekf_", "IEKF", "ERTS (IEKF)", IEKF, IERTS)):
            if not (data_dir / f"{prefix}filter.csv").exists():
                continue
            f, s = load(data_dir / f"{prefix}filter.csv"), load(data_dir / f"{prefix}smoother.csv")
            filters.append(Estimator(filter_name, f_color, vectors(f, "x_updated", STATE_DIM),
                                     matrices(f, "P_updated", STATE_DIM)))
            smoothers.append(Estimator(smoother_name, s_color, vectors(s, "x_smoothed", STATE_DIM),
                                       matrices(s, "P_smoothed", STATE_DIM)))
            prefixes[filter_name] = prefix
    return filters, smoothers, prefixes


def report(lines, text=""):
    lines.append(text)


def main():
    data_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent.parent / "data"
    case_dir = data_dir.resolve().parent
    figures_dir = case_dir / "figures"
    figures_dir.mkdir(exist_ok=True)
    settings = read_run_settings(case_dir)
    dt, denied = settings["dt"], settings["denied"]
    with_variants = case_dir.parent.name in VARIANT_FOLDERS

    truth = load(data_dir / "truth.csv")
    x_true = vectors(truth, "x_true", STATE_DIM)
    k = truth["timestep"].astype(int)
    t = k * dt
    filters, smoothers, prefixes = load_estimators(data_dir, with_variants)
    estimators = filters + smoothers
    ekf, erts = filters[0], smoothers[0]

    fixes = sensor_fix_data(data_dir, "")
    gnss, magnetometer, dvl = fixes
    schedule = {"GNSS": np.zeros(len(k), bool), "magnetometer": np.zeros(len(k), bool),
                "DVL": np.zeros(len(k), bool)}
    for name, table in zip(schedule, fixes):
        schedule[name][table["timestep"].astype(int)] = True
    fix_counts = {name: int(due.sum()) for name, due in schedule.items()}

    nis_by_filter = {"EKF": nis_of(fixes)}
    for filter_ in filters[1:]:
        variant_fixes = sensor_fix_data(data_dir, prefixes[filter_.name])
        if variant_fixes is not None:
            nis_by_filter[filter_.name] = nis_of(variant_fixes)

    position_error = lambda e: np.linalg.norm(e.x[:, POSITION] - x_true[:, POSITION], axis=1)
    position_nees = lambda e: masked_quadratic_form(x_true[:, POSITION] - e.x[:, POSITION],
                                                    e.P[:, :2, :2])

    out = []
    report(out, f"{len(k)} timesteps, dt {dt} s, fixes: GNSS {fix_counts['GNSS']}, "
                f"magnetometer {fix_counts['magnetometer']}, DVL {fix_counts['DVL']}")
    report(out)
    report(out, "consistency (time-averaged; above the band the estimator is overconfident, below it underconfident):")
    for name, by_sensor in nis_by_filter.items():
        for sensor, values in by_sensor.items():
            result = time_averaged(values, SENSOR_DIMS[sensor])
            label = f"ANIS {sensor} {name}"
            if result is None:
                report(out, f"  {label:<28} no fixes")
                continue
            report(out, f"  {label:<28} {result['average']:9.4f}  band [{result['low']:.4f}, "
                        f"{result['high']:.4f}]  {result['inside']:5.1%} of steps in band  -> {result['verdict']}")
    nees_values = {e.name: position_nees(e) for e in estimators}
    for estimator in estimators:
        result = time_averaged(nees_values[estimator.name], 2)
        label = f"ANEES position {estimator.name}"
        note = f"  ({result['dropped']} steps dropped, P not PSD)" if result["dropped"] else ""
        report(out, f"  {label:<28} {result['average']:9.4f}  band [{result['low']:.4f}, "
                    f"{result['high']:.4f}]  {result['inside']:5.1%} of steps in band  "
                    f"-> {result['verdict']}{note}")

    report(out)
    report(out, "accuracy (position error in m, heading error in deg):")
    report(out, f"  {'':<14}{'RMSE pos':>9}{'median':>9}{'95th':>9}{'max':>9}{'RMSE head':>11}")
    for estimator in estimators:
        error = position_error(estimator)
        heading = np.degrees(angle_diff(estimator.x[:, HEADING], x_true[:, HEADING]))
        report(out, f"  {estimator.name:<14}{np.sqrt(np.mean(error ** 2)):9.4f}{np.median(error):9.4f}"
                    f"{np.percentile(error, 95):9.4f}{error.max():9.4f}{np.sqrt(np.mean(heading ** 2)):11.4f}")

    report(out)
    report(out, "bias RMSE (accel x [m/s^2], accel y [m/s^2], gyro [rad/s]) and peak gyro error:")
    for estimator in estimators:
        bias_error = estimator.x[:, 5:8] - x_true[:, 5:8]
        rmse = np.sqrt(np.mean(bias_error ** 2, axis=0))
        report(out, f"  {estimator.name:<14}{rmse[0]:9.4f}{rmse[1]:9.4f}{rmse[2]:11.5f}"
                    f"{np.abs(bias_error[:, 2]).max():11.4f}")

    report(out)
    report(out, "share of steps with the true position inside the reported 95% ellipse:")
    limit95 = chi2.ppf(CONFIDENCE, 2)
    for estimator in estimators:
        values = nees_values[estimator.name]
        report(out, f"  {estimator.name:<14}{np.mean(values[~np.isnan(values)] <= limit95):6.1%}")

    report(out)
    report(out, "smallest eigenvalue of the full P:")
    for estimator in estimators:
        smallest = np.linalg.eigvalsh(estimator.P).min(axis=-1)
        report(out, f"  {estimator.name:<14}min {smallest.min():10.3e}  "
                    f"{np.sum(smallest < 0)} of {len(smallest)} steps below zero")

    report(out)
    for filter_, smoother in zip(filters, smoothers):
        trace_f = np.trace(filter_.P[:, :2, :2], axis1=1, axis2=2)
        trace_s = np.trace(smoother.P[:, :2, :2], axis1=1, axis2=2)
        always = np.all(trace_s <= trace_f * (1 + 1e-6))
        report(out, f"{smoother.name} trace(P) <= {filter_.name} trace(P) at every step: {always}")

    if len(filters) > 1:
        report(out)
        report(out, "distance to the EKF-based estimate (position in mm, heading in deg), "
                    "and the range of the position trace(P) ratio:")
        for reference, others in ((filters[0], filters[1:]), (smoothers[0], smoothers[1:])):
            for other in others:
                distance = np.linalg.norm(other.x[:, POSITION] - reference.x[:, POSITION], axis=1)
                heading = np.degrees(np.abs(angle_diff(other.x[:, HEADING], reference.x[:, HEADING])))
                ratio = (np.trace(other.P[:, :2, :2], axis1=1, axis2=2)
                         / np.trace(reference.P[:, :2, :2], axis1=1, axis2=2))
                report(out, f"  {other.name} vs {reference.name}: median {np.median(distance) * 1e3:.2f} "
                            f"max {distance.max() * 1e3:.1f}, heading max {heading.max():.3f}, "
                            f"ratio [{ratio.min():.4f}, {ratio.max():.4f}]")

    text = "\n".join(out).rstrip() + "\n"
    print(text, end="")
    (case_dir / "stats.txt").write_text(text)

    figures = {
        "1_scenario.png": scenario_figure(t, x_true, schedule, dt),
        "2_path.png": path_figure(x_true, ekf, erts, find_turns(x_true, dt), dt),
        "3_errors.png": errors_figure(t, x_true, ekf, erts, denied),
        "4_uncertainty.png": uncertainty_figure(t, ekf, erts, denied),
    }
    if not with_variants:
        figures["5_biases.png"] = biases_figure(t, x_true, [ekf, erts], denied)
    nis = {}
    for sensor, table in zip(SENSOR_DIMS, fixes):
        if len(table["timestep"]):
            nis[sensor] = (table["timestep"].astype(int) * dt, nis_by_filter["EKF"][sensor],
                           SENSOR_COLORS[sensor])
    summaries = {}
    for sensor in nis:
        result = time_averaged(nis_by_filter["EKF"][sensor], SENSOR_DIMS[sensor])
        summaries[sensor] = (f"ANIS {result['average']:.3f}, band [{result['low']:.3f}, "
                             f"{result['high']:.3f}], {result['verdict']}")
    for estimator in (ekf, erts):
        result = time_averaged(nees_values[estimator.name], 2)
        summaries[estimator.name] = f"ANEES {result['average']:.3f} ({result['verdict']})"
    if not with_variants:
        figures["6_consistency.png"] = consistency_figure(
            t, nis, [(ekf.name, ekf.color, nees_values["EKF"]), (erts.name, erts.color, nees_values["ERTS"])],
            summaries)
    if len(filters) > 1:
        figures["5_variants.png"] = variants_figure(t, x_true, filters, smoothers, denied)
        focus = VARIANT_FOLDERS[case_dir.parent.name]
        index = [f.name for f in filters].index(focus)
        figures["6_errors_variant.png"] = errors_figure(t, x_true, filters[index], smoothers[index], denied)

    for name, figure in figures.items():
        figure.savefig(figures_dir / name, dpi=150)
        plt.close(figure)
    for stale in figures_dir.glob("*.png"):
        if stale.name not in figures:
            stale.unlink()


if __name__ == "__main__":
    main()
