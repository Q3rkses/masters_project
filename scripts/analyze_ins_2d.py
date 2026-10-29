#!/usr/bin/env python3
"""Figures and consistency statistics for one 2-D strapdown INS run.

usage: analyze_ins_2d.py <case_dir>/data

Reads truth.csv, filter.csv, smoother.csv and the three *_fixes.csv files, and
config.yaml from the case folder for the time step and the GNSS outage window.
The printed statistics are also written to <case_dir>/stats.txt and the
figures to <case_dir>/figures. Each case folder gets four figures for the EKF
and its ERTS smoother:

    1_scenario.png     path, estimates, fixes and when each sensor fired
    2_errors.png       error with 95% bands and the reported position spread
    3_biases.png       true and estimated sensor biases
    4_consistency.png  NIS per sensor and the running NEES

In the folders of the ukf and iekf experiments the UKF/URTSS and IEKF/ERTS
results are analysed next to the EKF/ERTS. Those folders get figures 1 and 2
and two more instead of 3 and 4, since the biases and the consistency are
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
ELLIPSE_COUNT = 4
ZOOM_SECONDS = 15.0
SKIP_STEPS = 100  # initial transient left out when choosing axis limits

Estimator = namedtuple("Estimator", "name color x P")

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
        "verdict": "consistent" if low <= average <= high else "INCONSISTENT",
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


def trajectory_figure(t, x_true, filt, smooth, gnss, schedule, denied, dt):
    """Path with GNSS fixes and both estimates, a zoom on the worst filter
    error, and when each sensor fired."""
    figure = plt.figure(figsize=(13, 9))
    grid = figure.add_gridspec(2, 2, height_ratios=[3, 1.1], width_ratios=[1.25, 1],
                               hspace=0.3, wspace=0.18)
    overview = figure.add_subplot(grid[0, 0])
    zoom = figure.add_subplot(grid[0, 1])
    timeline = figure.add_subplot(grid[1, :])

    error = np.linalg.norm(filt.x[:, POSITION] - x_true[:, POSITION], axis=1)
    half = int(ZOOM_SECONDS / dt / 2)
    centre = int(np.argmax(error[2 * half:-half])) + 2 * half
    lo, hi = centre - half, centre + half

    gnss_position = vectors(gnss, "z", 2)
    gnss_steps = gnss["timestep"].astype(int)

    overview.set_aspect("equal", adjustable="datalim")
    zoom.set_aspect("equal", adjustable="box")
    for axis in (overview, zoom):
        axis.plot(*x_true[:, POSITION].T, color=TRUTH, linewidth=1.6, label="truth", zorder=3)
        style_axis(axis)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
    if denied:
        inside = (t >= denied[0]) & (t < denied[1])
        overview.plot(*x_true[inside, POSITION].T, color=DENIED, linewidth=5, alpha=0.7,
                      zorder=2, solid_capstyle="butt", label="GNSS denied")
    overview.scatter(*gnss_position.T, s=14, color=GNSS, marker="x", zorder=5,
                     label=f"GNSS fix ({len(gnss_steps)})")
    overview.plot(*filt.x[:, POSITION].T, color=filt.color, linewidth=0.9, label=filt.name, zorder=4)
    overview.plot(*smooth.x[:, POSITION].T, color=smooth.color, linewidth=0.9, label=smooth.name, zorder=4)
    overview.scatter(*x_true[0, POSITION], s=45, color=TRUTH, zorder=6)
    overview.annotate("", xy=x_true[200, POSITION], xytext=x_true[0, POSITION],
                      arrowprops=dict(arrowstyle="-|>", color=TRUTH, lw=1.5), zorder=6)
    overview.set_title("Path and estimates", loc="left")
    overview.legend(loc="best", frameon=False, fontsize=8)

    ellipses = []
    extent = [x_true[lo:hi, POSITION]]
    for estimator in (filt, smooth):
        for k in np.linspace(lo, hi - 1, ELLIPSE_COUNT).astype(int):
            width, height, angle = ellipse_shape(estimator.P[k, :2, :2])
            ellipses.append((estimator, k, width, height, angle))
            reach = max(width, height) / 2
            extent.append(estimator.x[k, :2] + np.array([[reach, reach], [-reach, -reach]]))
    corners = np.vstack(extent)
    low, high = corners.min(axis=0) - 1.0, corners.max(axis=0) + 1.0
    overview.add_patch(Rectangle(low, *(high - low), fill=False, edgecolor=BAND,
                                 linewidth=1.2, zorder=1))

    zoom.plot(*filt.x[lo:hi, POSITION].T, color=filt.color, linewidth=1.3, label=filt.name, zorder=4)
    zoom.plot(*smooth.x[lo:hi, POSITION].T, color=smooth.color, linewidth=1.3, label=smooth.name, zorder=4)
    for estimator, k, width, height, angle in ellipses:
        zoom.add_patch(Ellipse(estimator.x[k, :2], width, height, angle=angle,
                               facecolor=estimator.color, edgecolor=estimator.color,
                               alpha=0.2, linewidth=1, zorder=2))
    in_window = (gnss_steps >= lo) & (gnss_steps < hi)
    if in_window.any():
        zoom.scatter(*gnss_position[in_window].T, s=40, color=GNSS, marker="x", zorder=5, label="GNSS fix")
    zoom.set_xlim(low[0], high[0])
    zoom.set_ylim(low[1], high[1])
    zoom.set_title(f"Zoom, t = {lo * dt:.0f} to {hi * dt:.0f} s, with 95% ellipses", loc="left")
    zoom.legend(loc="best", frameon=False, fontsize=8)

    for row, (name, due) in enumerate(schedule.items()):
        times = np.flatnonzero(due) * dt
        timeline.eventplot(times, lineoffsets=row, linelengths=0.7, linewidths=0.7,
                           color=SENSOR_COLORS[name])
        timeline.text(t[-1] * 1.005, row, f"{len(times)} fixes", va="center", fontsize=8, color=BAND)
    shade_denied(timeline, denied)
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
    """Error against truth with each estimator's own 95% band, and the
    position spread the estimators report."""
    scale = norm.ppf(0.5 + CONFIDENCE / 2)
    rows = (("x error [m]", 0, 1.0), ("y error [m]", 1, 1.0),
            ("heading error [deg]", HEADING, 180 / np.pi))
    figure, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
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
    spread = axes[3]
    for estimator in (filt, smooth):
        spread.plot(t, np.sqrt(np.trace(estimator.P[:, :2, :2], axis1=1, axis2=2)),
                    color=estimator.color, linewidth=1.3, label=estimator.name)
    shade_denied(spread, denied)
    spread.set_yscale("log")
    spread.set_ylabel(r"$\sqrt{\mathrm{tr}\,P_{xy}}$ [m]")
    spread.set_xlabel("time [s]")
    style_axis(spread)
    axes[0].set_title("Error with 95% confidence band, shaded where GNSS is denied", loc="left")
    axes[0].legend(loc="upper right", frameon=False, ncol=2, fontsize=9)
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
    for estimator in filters:
        axes[0].plot(t, position_error(estimator), color=estimator.color, linewidth=1, label=estimator.name)
    for estimator in smoothers:
        axes[1].plot(t, position_error(estimator), color=estimator.color, linewidth=1, label=estimator.name)
    axes[2].plot(t, x_true[:, 7], color=TRUTH, linewidth=1.4, label="truth")
    for estimator in filters:
        axes[2].plot(t, estimator.x[:, 7], color=estimator.color, linewidth=1, label=estimator.name)
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
    report(out, "consistency (time-averaged):")
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
        "1_scenario.png": trajectory_figure(t, x_true, ekf, erts, gnss, schedule, denied, dt),
        "2_errors.png": errors_figure(t, x_true, ekf, erts, denied),
    }
    if not with_variants:
        figures["3_biases.png"] = biases_figure(t, x_true, [ekf, erts], denied)
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
        figures["4_consistency.png"] = consistency_figure(
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
