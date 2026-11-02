#!/usr/bin/env python3
"""Figures and consistency statistics for one 3-D strapdown INS run.

usage: analyze_ins_3d.py <case_dir>/data

Reads truth.csv, filter.csv, smoother.csv and the four *_fixes.csv files, and
config.yaml from the case folder for the time step and the GNSS outage window.
The printed statistics are also written to <case_dir>/stats.txt and the
figures to <case_dir>/figures. Each case folder gets six figures for the EKF
and its ERTS smoother:

    1_scenario.png      the true 3D path with the vehicle's attitude, and when each sensor fired
    2_path.png          the 3D path with both estimates, and the depth against time
    3_errors.png        position and attitude error with 95% bands
    4_uncertainty.png   the position and attitude spread the estimators report
    5_biases.png        true and estimated sensor biases
    6_consistency.png   NIS per sensor and the running NEES

A statistic that falls above its 95% interval means the errors are larger than
the reported covariance admits, the estimator is overconfident. Below the
interval it is underconfident, the covariance is larger than it needs to be.

In the folders of the ukf and iekf experiments the UKF/URTSS or IEKF/ERTS
results are the subject, analysed next to the EKF/ERTS, and the fourth figure
compares the variants instead of showing the uncertainty.

Nominal state: [p(3), v(3), q(4), b_a(3), b_g(3)] with q = [w, x, y, z] rotating
body to navigation (NED, z down). The covariance is on the 15D error state
[dp, dv, dtheta, db_a, db_g], where the attitude error is a rotation vector in
the body frame, q_true = q_est * Exp(dtheta).
"""

import re
import sys
from collections import namedtuple
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator
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
ALTIMETER = "#2c3e8f"
DENIED = "#f2c48d"

STATE_DIM = 16
ERROR_DIM = 15
POSITION = slice(0, 3)
VELOCITY = slice(3, 6)
QUATERNION = slice(6, 10)
ACCEL_BIAS = slice(10, 13)
GYRO_BIAS = slice(13, 16)
# the same parts in the 15D error state the covariance lives on
POSITION_ERR = slice(0, 3)
ATTITUDE_ERR = slice(6, 9)
SENSOR_DIMS = {"GNSS": 3, "magnetometer": 1, "DVL": 3, "altimeter": 1}
SENSOR_FILES = {"GNSS": "gnss", "magnetometer": "magnetometer", "DVL": "dvl", "altimeter": "altimeter"}
SENSOR_COLORS = {"GNSS": GNSS, "magnetometer": MAGNETOMETER, "DVL": DVL, "altimeter": ALTIMETER}
CONFIDENCE = 0.95
AVERAGING_WINDOW = 150
SKIP_STEPS = 100  # initial transient left out when choosing axis limits
ATTITUDE_COUNT = 14  # attitude triads drawn along the true path

Estimator = namedtuple("Estimator", "name color x P")

VARIANT_FOLDERS = {"ukf": "UKF", "iekf": "IEKF"}


def load(path):
    return np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))


def vectors(table, name, dim):
    return np.column_stack([table[f"{name}_{i}"] for i in range(dim)])


def matrices(table, name, dim):
    columns = [table[f"{name}_{i}_{j}"] for i in range(dim) for j in range(dim)]
    return np.column_stack(columns).reshape(-1, dim, dim)


def quaternion_product(a, b):
    """Hamilton product of quaternions [w, x, y, z], row by row."""
    aw, av = a[:, :1], a[:, 1:]
    bw, bv = b[:, :1], b[:, 1:]
    return np.hstack([aw * bw - np.sum(av * bv, axis=1, keepdims=True),
                      aw * bv + bw * av + np.cross(av, bv)])


def quaternion_conjugate(q):
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def attitude_error(q_true, q_est):
    """Rotation vector Log(q_est^* q_true), the body-frame attitude error the
    covariance describes. q and -q are the same rotation, so the shorter of
    the two is taken."""
    delta = quaternion_product(quaternion_conjugate(q_est), q_true)
    delta = np.where(delta[:, :1] < 0, -delta, delta)
    vector_norm = np.linalg.norm(delta[:, 1:], axis=1, keepdims=True)
    angle = 2 * np.arctan2(vector_norm, delta[:, :1])
    return angle * delta[:, 1:] / np.maximum(vector_norm, 1e-15)


def error_state(x_true, x):
    """The 15D error [dp, dv, dtheta, db_a, db_g] of an estimate."""
    return np.hstack([x_true[:, POSITION] - x[:, POSITION],
                      x_true[:, VELOCITY] - x[:, VELOCITY],
                      attitude_error(x_true[:, QUATERNION], x[:, QUATERNION]),
                      x_true[:, ACCEL_BIAS] - x[:, ACCEL_BIAS],
                      x_true[:, GYRO_BIAS] - x[:, GYRO_BIAS]])


def rotation_matrices(q):
    """Body-to-navigation rotation matrices of quaternions [w, x, y, z]."""
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], axis=-1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], axis=-1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], axis=-1),
    ], axis=1)


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


def equal_axes_3d(axis, curves, margin=3.0):
    """Equal scaling of a 3D axis with the limits just around the curves, and
    depth pointing down as in the navigation frame."""
    points = np.vstack(curves)
    low, high = points.min(axis=0) - margin, points.max(axis=0) + margin
    axis.set_xlim(low[0], high[0])
    axis.set_ylim(low[1], high[1])
    axis.set_zlim(low[2], high[2])
    axis.set_box_aspect(high - low)
    axis.invert_zaxis()
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_zlabel("z, down [m]", labelpad=8)
    axis.zaxis.set_major_locator(MaxNLocator(3))


def style_axis_3d(axis):
    axis.grid(True)
    for pane in (axis.xaxis, axis.yaxis, axis.zaxis):
        pane.pane.set_facecolor((1, 1, 1, 0))
        pane.pane.set_edgecolor(BAND)


def scenario_figure(t, x_true, schedule, dt):
    """The true path with the vehicle's attitude along it, and when each
    sensor fired."""
    figure = plt.figure(figsize=(13, 10))
    grid = figure.add_gridspec(2, 1, height_ratios=[3, 1.1], hspace=0.2)
    path = figure.add_subplot(grid[0], projection="3d")
    timeline = figure.add_subplot(grid[1])

    position = x_true[:, POSITION]
    path.plot(*position.T, color=TRUTH, linewidth=1.5, label="true path")
    steps = np.linspace(0, len(x_true) - 1, ATTITUDE_COUNT).astype(int)
    rotation = rotation_matrices(x_true[steps, QUATERNION])
    length = 0.08 * np.ptp(position, axis=0).max()
    # the body x axis points forward, the body z axis down through the hull
    for column, color, label in ((0, EKF, "body x (forward)"), (2, ERTS, "body z (down)")):
        path.quiver(*position[steps].T, *rotation[:, :, column].T, length=length,
                    color=color, linewidth=1.4, arrow_length_ratio=0.25, label=label)
    path.scatter(*position[0], s=45, color=TRUTH, label="start")
    equal_axes_3d(path, [position])
    style_axis_3d(path)
    path.set_title("True path with the vehicle attitude", loc="left")
    path.legend(loc="upper right", frameon=False, fontsize=9)

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


def path_figure(t, x_true, filt, smooth, denied, reference=None):
    """The 3D path with the subject's estimates, and the depth against time.
    With a reference, the EKF-based estimates are drawn wide underneath."""
    figure = plt.figure(figsize=(14, 7.5))
    grid = figure.add_gridspec(1, 2, width_ratios=[1.3, 1], wspace=0.12)
    overview = figure.add_subplot(grid[0], projection="3d")
    depth = figure.add_subplot(grid[1])

    overview.plot(*x_true[:, POSITION].T, color=TRUTH, linewidth=1.8, label="truth", zorder=3)
    for estimator in reference or ():
        overview.plot(*estimator.x[:, POSITION].T, color=estimator.color, linewidth=3.0,
                      alpha=0.35, label=estimator.name)
    for estimator in (filt, smooth):
        overview.plot(*estimator.x[:, POSITION].T, color=estimator.color, linewidth=1.0,
                      label=estimator.name, zorder=4)
    overview.scatter(*x_true[0, POSITION], s=45, color=TRUTH)
    equal_axes_3d(overview, [x_true[:, POSITION]] + [e.x[SKIP_STEPS:, POSITION] for e in (filt, smooth)])
    style_axis_3d(overview)
    overview.set_title("Path and estimates", loc="left")
    overview.legend(loc="upper right", frameon=False, fontsize=9)

    depth.plot(t, x_true[:, 2], color=TRUTH, linewidth=1.8, label="truth", zorder=3)
    for estimator in (filt, smooth):
        depth.plot(t, estimator.x[:, 2], color=estimator.color, linewidth=1.0, label=estimator.name)
    shade_denied(depth, denied)
    depth.invert_yaxis()
    depth.set_xlabel("time [s]")
    depth.set_ylabel("z, down [m]")
    depth.set_title("Depth, shaded where GNSS is denied", loc="left")
    depth.legend(loc="best", frameon=False, fontsize=9)
    style_axis(depth)
    return figure


def errors_figure(t, x_true, filt, smooth, denied):
    """Position and attitude error against truth with each estimator's own
    95% band. The attitude error is the body-frame rotation vector."""
    scale = norm.ppf(0.5 + CONFIDENCE / 2)
    figure, axes = plt.subplots(3, 2, figsize=(14, 8.5), sharex=True)
    errors = {estimator.name: error_state(x_true, estimator.x) for estimator in (filt, smooth)}
    columns = (("Position error with 95% band, shaded where GNSS is denied", "m", POSITION_ERR, 1.0,
                ("x", "y", "z")),
               ("Attitude error about the body axes with 95% band", "deg", ATTITUDE_ERR, 180 / np.pi,
                ("roll", "pitch", "yaw")))
    for column, (title, unit, block, scaling, axis_names) in enumerate(columns):
        for row in range(3):
            axis = axes[row, column]
            index = block.start + row
            curves = []
            for estimator in (filt, smooth):
                diff = errors[estimator.name][:, index] * scaling
                sigma = scale * np.sqrt(np.clip(estimator.P[:, index, index], 0, None)) * scaling
                axis.fill_between(t, -sigma, sigma, color=estimator.color, alpha=0.15, linewidth=0)
                axis.plot(t, diff, color=estimator.color, linewidth=1, label=estimator.name)
                curves += [diff, sigma]
            axis.axhline(0, color=BAND, linewidth=0.8, linestyle="--")
            shade_denied(axis, denied)
            limit(axis, curves)
            axis.set_ylabel(f"{axis_names[row]} [{unit}]")
            style_axis(axis)
        axes[0, column].set_title(title, loc="left")
        axes[-1, column].set_xlabel("time [s]")
    axes[0, 0].legend(loc="upper right", frameon=False, ncol=2, fontsize=9)
    figure.tight_layout()
    return figure


def uncertainty_figure(t, filt, smooth, denied):
    """The position and attitude spread the estimators report about themselves."""
    figure, axes = plt.subplots(1, 2, figsize=(14, 4.5), sharex=True)
    for axis, block, label in ((axes[0], POSITION_ERR, r"$\sqrt{\mathrm{tr}\,P_{p}}$ [m]"),
                               (axes[1], ATTITUDE_ERR, r"$\sqrt{\mathrm{tr}\,P_{\theta}}$ [deg]")):
        scaling = 1.0 if block is POSITION_ERR else 180 / np.pi
        for estimator in (filt, smooth):
            axis.plot(t, scaling * np.sqrt(np.trace(estimator.P[:, block, block], axis1=1, axis2=2)),
                      color=estimator.color, linewidth=1.4, label=estimator.name)
        shade_denied(axis, denied)
        axis.set_yscale("log")
        axis.set_ylabel(label)
        axis.set_xlabel("time [s]")
        style_axis(axis)
    axes[0].set_title("Reported position spread, shaded where GNSS is denied", loc="left")
    axes[1].set_title("Reported attitude spread", loc="left")
    axes[0].legend(loc="upper right", frameon=False, ncol=2, fontsize=9)
    figure.tight_layout()
    return figure


def biases_figure(t, x_true, estimators, denied):
    figure, axes = plt.subplots(3, 2, figsize=(14, 8), sharex=True)
    names = ("x", "y", "z")
    for column, (block, title, unit) in enumerate(
            ((ACCEL_BIAS, "Accelerometer bias", "m/s$^2$"), (GYRO_BIAS, "Gyro bias", "rad/s"))):
        for row in range(3):
            axis = axes[row, column]
            axis.plot(t, x_true[:, block.start + row], color=TRUTH, linewidth=1.4, label="truth")
            for estimator in estimators:
                axis.plot(t, estimator.x[:, block.start + row], color=estimator.color,
                          linewidth=1.1, label=estimator.name)
            shade_denied(axis, denied)
            axis.set_ylabel(f"{names[row]} [{unit}]")
            style_axis(axis)
        axes[0, column].set_title(f"{title} against truth", loc="left")
        axes[-1, column].set_xlabel("time [s]")
    axes[0, 0].legend(loc="best", frameon=False, ncol=len(estimators) + 1, fontsize=9)
    figure.tight_layout()
    return figure


def consistency_figure(t, nis, nees, summaries):
    """NIS of every sensor that fired, and the running NEES of the whole
    error state. The time-averaged value and its verdict are written into
    each panel."""
    rows = len(nis) + 1
    figure, axes = plt.subplots(rows, 1, figsize=(11, 2.4 * rows + 0.6), sharex=True)
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
    low, high = chi2_interval(ERROR_DIM)
    axis.axhspan(low, high, color=BAND, alpha=0.15, linewidth=0)
    axis.axhline(ERROR_DIM, color=BAND, linewidth=1, linestyle="--")
    for label, color, values in nees:
        values = np.nan_to_num(values)
        axis.plot(t[AVERAGING_WINDOW - 1:], running_mean(values, AVERAGING_WINDOW),
                  color=color, linewidth=1.3, label=f"{label}, {summaries[label]}")
    axis.set_ylabel(f"NEES, 15 dof\n({AVERAGING_WINDOW}-step mean)")
    axis.set_xlabel("time [s]")
    axis.legend(loc="best", frameon=False, fontsize=9)
    style_axis(axis)
    axes[0].set_title("NIS per fix and running NEES; shaded: 95% interval of one sample, "
                      "dashed: expected value", loc="left")
    figure.tight_layout()
    return figure


def variants_figure(t, x_true, filters, smoothers, denied):
    """Position and attitude error of all variants, filters and smoothers."""
    figure, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    position_error = lambda e: np.linalg.norm(e.x[:, POSITION] - x_true[:, POSITION], axis=1)
    attitude_norm = lambda e: np.degrees(np.linalg.norm(attitude_error(x_true[:, QUATERNION],
                                                                       e.x[:, QUATERNION]), axis=1))
    panels = ((filters, position_error, "filter position error [m]"),
              (smoothers, position_error, "smoother position error [m]"),
              (filters, attitude_norm, "filter attitude error [deg]"),
              (smoothers, attitude_norm, "smoother attitude error [deg]"))
    for axis, (group, series, label) in zip(axes, panels):
        for estimator in group:
            axis.plot(t, series(estimator), color=estimator.color, linewidth=1.1, label=estimator.name)
        shade_denied(axis, denied)
        axis.set_ylabel(label)
        axis.legend(loc="upper right", frameon=False, ncol=len(group), fontsize=9)
        style_axis(axis)
    axes[-1].set_xlabel("time [s]")
    axes[0].set_title("Estimator variants, shaded where GNSS is denied", loc="left")
    figure.tight_layout()
    return figure


def sensor_fix_data(data_dir, prefix):
    paths = [data_dir / f"{prefix}{name}_fixes.csv" for name in SENSOR_FILES.values()]
    if not all(path.exists() for path in paths):
        return None
    return [load(path) for path in paths]


def nis_of(fixes):
    return {name: quadratic_form(vectors(table, "innovation", dim), matrices(table, "S", dim))
            if len(table["timestep"]) else np.array([])
            for (name, dim), table in zip(SENSOR_DIMS.items(), fixes)}


def load_estimators(data_dir, with_variants):
    filtered = load(data_dir / "filter.csv")
    smoothed = load(data_dir / "smoother.csv")
    filters = [Estimator("EKF", EKF, vectors(filtered, "x_updated", STATE_DIM),
                         matrices(filtered, "P_updated", ERROR_DIM))]
    smoothers = [Estimator("ERTS", ERTS, vectors(smoothed, "x_smoothed", STATE_DIM),
                           matrices(smoothed, "P_smoothed", ERROR_DIM))]
    prefixes = {"EKF": ""}
    if with_variants:
        for prefix, filter_name, smoother_name, f_color, s_color in (
                ("ukf_", "UKF", "URTSS", UKF, URTSS), ("iekf_", "IEKF", "ERTS (IEKF)", IEKF, IERTS)):
            if not (data_dir / f"{prefix}filter.csv").exists():
                continue
            f, s = load(data_dir / f"{prefix}filter.csv"), load(data_dir / f"{prefix}smoother.csv")
            filters.append(Estimator(filter_name, f_color, vectors(f, "x_updated", STATE_DIM),
                                     matrices(f, "P_updated", ERROR_DIM)))
            smoothers.append(Estimator(smoother_name, s_color, vectors(s, "x_smoothed", STATE_DIM),
                                       matrices(s, "P_smoothed", ERROR_DIM)))
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
    schedule = {name: np.zeros(len(k), bool) for name in SENSOR_DIMS}
    for name, table in zip(schedule, fixes):
        schedule[name][table["timestep"].astype(int)] = True
    fix_counts = {name: int(due.sum()) for name, due in schedule.items()}

    nis_by_filter = {"EKF": nis_of(fixes)}
    for filter_ in filters[1:]:
        variant_fixes = sensor_fix_data(data_dir, prefixes[filter_.name])
        if variant_fixes is not None:
            nis_by_filter[filter_.name] = nis_of(variant_fixes)

    errors = {e.name: error_state(x_true, e.x) for e in estimators}
    position_error = lambda e: np.linalg.norm(errors[e.name][:, POSITION_ERR], axis=1)
    attitude_norm = lambda e: np.linalg.norm(errors[e.name][:, ATTITUDE_ERR], axis=1)
    nees_blocks = {"full": (slice(0, ERROR_DIM), ERROR_DIM),
                   "position": (POSITION_ERR, 3),
                   "attitude": (ATTITUDE_ERR, 3)}
    nees_values = {label: {e.name: masked_quadratic_form(errors[e.name][:, block], e.P[:, block, block])
                           for e in estimators}
                   for label, (block, _) in nees_blocks.items()}

    out = []
    report(out, f"{len(k)} timesteps, dt {dt} s, fixes: " +
           ", ".join(f"{name} {count}" for name, count in fix_counts.items()))
    report(out)
    report(out, "consistency (time-averaged; above the band the estimator is overconfident, below it underconfident):")
    for name, by_sensor in nis_by_filter.items():
        for sensor, values in by_sensor.items():
            result = time_averaged(values, SENSOR_DIMS[sensor]) if len(values) else None
            label = f"ANIS {sensor} {name}"
            if result is None:
                report(out, f"  {label:<30} no fixes")
                continue
            report(out, f"  {label:<30} {result['average']:9.4f}  band [{result['low']:.4f}, "
                        f"{result['high']:.4f}]  {result['inside']:5.1%} of steps in band  -> {result['verdict']}")
    for part, (block, dof) in nees_blocks.items():
        for estimator in estimators:
            result = time_averaged(nees_values[part][estimator.name], dof)
            label = f"ANEES {part} {estimator.name}"
            note = f"  ({result['dropped']} steps dropped, P not PSD)" if result["dropped"] else ""
            report(out, f"  {label:<30} {result['average']:9.4f}  band [{result['low']:.4f}, "
                        f"{result['high']:.4f}]  {result['inside']:5.1%} of steps in band  "
                        f"-> {result['verdict']}{note}")

    report(out)
    report(out, "accuracy (position error in m, attitude error in deg):")
    report(out, f"  {'':<14}{'RMSE pos':>9}{'median':>9}{'95th':>9}{'max':>9}{'RMSE att':>10}{'max att':>9}")
    for estimator in estimators:
        error = position_error(estimator)
        attitude = np.degrees(attitude_norm(estimator))
        report(out, f"  {estimator.name:<14}{np.sqrt(np.mean(error ** 2)):9.4f}{np.median(error):9.4f}"
                    f"{np.percentile(error, 95):9.4f}{error.max():9.4f}"
                    f"{np.sqrt(np.mean(attitude ** 2)):10.4f}{attitude.max():9.4f}")

    report(out)
    report(out, "bias RMSE (accel x, y, z [m/s^2], gyro x, y, z [rad/s]):")
    for estimator in estimators:
        rmse = np.sqrt(np.mean(errors[estimator.name][:, 9:15] ** 2, axis=0))
        report(out, f"  {estimator.name:<14}" + "".join(f"{value:10.5f}" for value in rmse))

    report(out)
    report(out, "share of steps with the true position inside the reported 95% ellipsoid:")
    limit95 = chi2.ppf(CONFIDENCE, 3)
    for estimator in estimators:
        values = nees_values["position"][estimator.name]
        report(out, f"  {estimator.name:<14}{np.mean(values[~np.isnan(values)] <= limit95):6.1%}")

    report(out)
    report(out, "smallest eigenvalue of the full P:")
    for estimator in estimators:
        smallest = np.linalg.eigvalsh(estimator.P).min(axis=-1)
        report(out, f"  {estimator.name:<14}min {smallest.min():10.3e}  "
                    f"{np.sum(smallest < 0)} of {len(smallest)} steps below zero")

    report(out)
    for filter_, smoother in zip(filters, smoothers):
        trace_f = np.trace(filter_.P[:, :3, :3], axis1=1, axis2=2)
        trace_s = np.trace(smoother.P[:, :3, :3], axis1=1, axis2=2)
        always = np.all(trace_s <= trace_f * (1 + 1e-6))
        report(out, f"{smoother.name} trace(P_p) <= {filter_.name} trace(P_p) at every step: {always}")

    if len(filters) > 1:
        report(out)
        report(out, "distance to the EKF-based estimate (position in mm, attitude in deg), "
                    "and the range of the position trace(P) ratio:")
        for reference, others in ((filters[0], filters[1:]), (smoothers[0], smoothers[1:])):
            for other in others:
                distance = np.linalg.norm(other.x[:, POSITION] - reference.x[:, POSITION], axis=1)
                attitude = np.degrees(np.linalg.norm(attitude_error(reference.x[:, QUATERNION],
                                                                    other.x[:, QUATERNION]), axis=1))
                ratio = (np.trace(other.P[:, :3, :3], axis1=1, axis2=2)
                         / np.trace(reference.P[:, :3, :3], axis1=1, axis2=2))
                report(out, f"  {other.name} vs {reference.name}: median {np.median(distance) * 1e3:.2f} "
                            f"max {distance.max() * 1e3:.1f}, attitude max {attitude.max():.3f}, "
                            f"ratio [{ratio.min():.4f}, {ratio.max():.4f}]")

    text = "\n".join(out).rstrip() + "\n"
    print(text, end="")
    (case_dir / "stats.txt").write_text(text)

    figures = {"1_scenario.png": scenario_figure(t, x_true, schedule, dt)}
    if with_variants:
        focus = VARIANT_FOLDERS[case_dir.parent.name]
        index = [f.name for f in filters].index(focus)
        subject_filter, subject_smoother = filters[index], smoothers[index]
        figures["2_path.png"] = path_figure(t, x_true, subject_filter, subject_smoother, denied,
                                            reference=(ekf, erts))
        figures["3_errors.png"] = errors_figure(t, x_true, subject_filter, subject_smoother, denied)
        figures["4_variants.png"] = variants_figure(t, x_true, filters, smoothers, denied)
    else:
        figures["2_path.png"] = path_figure(t, x_true, ekf, erts, denied)
        figures["3_errors.png"] = errors_figure(t, x_true, ekf, erts, denied)
        figures["4_uncertainty.png"] = uncertainty_figure(t, ekf, erts, denied)
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
            result = time_averaged(nees_values["full"][estimator.name], ERROR_DIM)
            summaries[estimator.name] = f"ANEES {result['average']:.3f} ({result['verdict']})"
        figures["6_consistency.png"] = consistency_figure(
            t, nis, [(ekf.name, ekf.color, nees_values["full"]["EKF"]),
                     (erts.name, erts.color, nees_values["full"]["ERTS"])], summaries)

    for name, figure in figures.items():
        figure.savefig(figures_dir / name, dpi=150)
        plt.close(figure)
    for stale in figures_dir.glob("*.png"):
        if stale.name not in figures:
            stale.unlink()


if __name__ == "__main__":
    main()
