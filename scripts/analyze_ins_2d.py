#!/usr/bin/env python3
"""Plot the 2D strapdown INS filter and smoother results and check their
consistency. The EKF/ERTS results (filter.csv, smoother.csv, *_fixes.csv) are
always expected, the UKF/URTSS ones (ukf_*.csv) get the same set of figures,
prefixed ukf_, when they exist in the same directory, plus a few figures that
compare the two variants.

State is [x, y, psi, u, v, b_ax, b_ay, b_gyro]. Each sensor (GNSS,
magnetometer, DVL) arrives at irregular intervals, not every timestep, and
has its own fix CSV (timestep, raw measurement, innovation, S), one row per
real fix, never every row of truth.csv.

usage: analyze_ins_2d.py [data_dir]
"""

import sys
from collections import namedtuple
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse
from scipy.stats import chi2, norm

TRUTH = "#3d3d3a"
FILTER = "#2a78d6"
SMOOTHER = "#eb6834"
BAND = "#8a8984"
GNSS = "#c0392b"
MAGNETOMETER = "#8e44ad"
DVL = "#16a085"
UKF = "#2e9e5b"
URTSS = "#b8860b"

STATE_DIM = 8
MEASUREMENT_DIM = 2  # GNSS position only
MAGNETOMETER_DIM = 1  # heading only
DVL_DIM = 2  # body-frame velocity only
MAGNETOMETER_ARROW_LENGTH = 3.0  # meters, purely for visibility
DVL_ARROW_SCALE = 1.5  # meters per m/s, purely for visibility
FIX_STRIDE = 20  # magnetometer/DVL fire far more often than GNSS; plot every Nth
CONFIDENCE = 0.95
ELLIPSES = 6  # confidence ellipses drawn along each estimated trajectory
AVERAGING_WINDOW = 150  # raw per-step NEES is noise; the running mean is readable

# one estimator (EKF, ERTS, UKF or URTSS) as the figures draw it
Estimator = namedtuple("Estimator", "name color x P")


def names(estimators):
    """EKF and ERTS, for figure titles."""
    return " and ".join(e.name for e in estimators)


# where each part sits in the state [x, y, psi, u, v, b_ax, b_ay, b_gyro]
POSITION = slice(0, 2)
HEADING = 2


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


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
    axis.scatter(z_fix[:, 0], z_fix[:, 1], s=30, color=GNSS, zorder=5,
                label="GNSS fix", marker="x")


def draw_magnetometer_fixes(axis, position_fix, heading_fix):
    """Arrows at the true position when a magnetometer fix arrived, pointing
    in the measured (noisy) heading. Shows both where and what it read."""
    dx = MAGNETOMETER_ARROW_LENGTH * np.cos(heading_fix)
    dy = MAGNETOMETER_ARROW_LENGTH * np.sin(heading_fix)
    axis.quiver(position_fix[:, 0], position_fix[:, 1], dx, dy, color=MAGNETOMETER,
               angles="xy", scale_units="xy", scale=1, width=0.005, zorder=5,
               label="magnetometer fix")


def draw_dvl_fixes(axis, position_fix, velocity_body_fix, heading_fix):
    """Arrows at the true position when a DVL fix arrived, showing the
    measured body-frame velocity rotated into the nav frame by the true
    heading. The DVL measures velocity only, not position."""
    cos_psi, sin_psi = np.cos(heading_fix), np.sin(heading_fix)
    u, v = velocity_body_fix[:, 0], velocity_body_fix[:, 1]
    dx = DVL_ARROW_SCALE * (cos_psi * u - sin_psi * v)
    dy = DVL_ARROW_SCALE * (sin_psi * u + cos_psi * v)
    axis.quiver(position_fix[:, 0], position_fix[:, 1], dx, dy, color=DVL,
               angles="xy", scale_units="xy", scale=1, width=0.005, zorder=5,
               label="DVL fix")


def draw_truth(axis, x_true):
    axis.plot(x_true[:, 0], x_true[:, 1], color=TRUTH, linewidth=1.2,
              label="ground truth", zorder=3)
    axis.scatter(x_true[0, 0], x_true[0, 1], s=30, color=TRUTH, zorder=6, label="start")


def draw_estimate(axis, indices, estimator):
    """An estimator as a path with 95% position confidence ellipses at a few timesteps."""
    x, P = estimator.x, estimator.P
    axis.plot(x[:, 0], x[:, 1], color=estimator.color, linewidth=1.5,
              label=estimator.name, zorder=3)
    for count, k in enumerate(indices):
        width, height, angle = ellipse_shape(P[k, :2, :2])
        axis.add_patch(Ellipse(
            x[k, :2], width, height, angle=angle, facecolor=estimator.color,
            edgecolor=estimator.color, alpha=0.18, linewidth=1, zorder=2,
            label=f"{estimator.name} 95%" if count == 0 else None))


def finish(axis):
    axis.legend(loc="best", frameon=False, fontsize=9)


def running_mean(values, window):
    return np.convolve(values, np.ones(window) / window, mode="valid")


def effective_samples(values, count):
    """How many independent samples `count` correlated ones are worth."""
    rho = np.corrcoef(values[:-1], values[1:])[0, 1]
    return count * (1 - rho) / (1 + rho)


def summarise(name, values, dof):
    """Time-averaged consistency, which is a far tighter test than per-timestep.

    values may contain NaN where P wasn't positive semidefinite (see
    masked_quadratic_form); those timesteps are dropped, not counted as
    inconsistent, and how many were dropped is reported.
    """
    valid = values[~np.isnan(values)]
    if len(valid) == 0:
        print(f"  {name:<20} no fixes recorded, nothing to compute")
        return
    dropped = len(values) - len(valid)
    # effective_samples can come out below 1 when the signal is almost
    # perfectly autocorrelated (e.g. position NEES over a run with no GNSS
    # fixes to decorrelate it); clamped to 1 since "fewer than one
    # independent sample" would make the averaged band tighter than the
    # always-valid per-sample band, which is nonsensical
    samples = max(1.0, effective_samples(valid, len(valid)))
    low, high = chi2_interval(dof, samples=samples)
    step_low, step_high = chi2_interval(dof)
    inside = np.mean((valid >= step_low) & (valid <= step_high))
    average = valid.mean()
    verdict = "consistent" if low <= average <= high else "INCONSISTENT"
    dropped_note = f"  ({dropped} steps dropped, P not PSD)" if dropped else ""
    print(f"  {name:<20} {average:9.4f}  band [{low:.4f}, {high:.4f}]  "
          f"{inside:5.1%} of steps in band  -> {verdict}{dropped_note}")


def filter_vs_smoother_figure(x_true, estimators):
    """2+3. Ground truth against a filter and its smoother, each with its own
    95% confidence ellipses, in one window."""
    indices = np.linspace(0, len(x_true) - 1, ELLIPSES).astype(int)
    figure, axis = new_figure(f"{names(estimators)}, all {len(x_true)} timesteps")
    draw_truth(axis, x_true)
    for estimator in estimators:
        draw_estimate(axis, indices, estimator)
    finish(axis)
    return figure


def sensor_fixes_figure(x_true, z_gnss, magnetometer_position, magnetometer_heading,
                        dvl_position, dvl_velocity_body, dvl_heading):
    """1. Ground truth against all three sensors' fixes, overlaid. Magnetometer
    and DVL fire far more often than GNSS, so only every FIX_STRIDE-th one is
    drawn."""
    figure, axis = new_figure("Sensor fixes")
    draw_truth(axis, x_true)
    draw_gnss_fixes(axis, z_gnss)
    draw_magnetometer_fixes(axis, magnetometer_position[::FIX_STRIDE],
                            magnetometer_heading[::FIX_STRIDE])
    draw_dvl_fixes(axis, dvl_position[::FIX_STRIDE], dvl_velocity_body[::FIX_STRIDE],
                   dvl_heading[::FIX_STRIDE])
    finish(axis)
    return figure


def bias_figure(k, x_true, estimators):
    """True vs estimated sensor bias over time, one window, three stacked
    plots. Can the filter learn it?"""
    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    labels = ("accelerometer bias x [m/s^2]", "accelerometer bias y [m/s^2]",
             "gyro bias [rad/s]")
    for i, (axis, label) in enumerate(zip(axes, labels)):
        axis.plot(k, x_true[:, 5 + i], color=TRUTH, linewidth=1.2, label="true bias")
        for estimator in estimators:
            axis.plot(k, estimator.x[:, 5 + i], color=estimator.color,
                      linewidth=1.2, label=estimator.name)
        axis.set_ylabel(label)
        axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
        axis.set_axisbelow(True)
        axis.spines[["top", "right"]].set_visible(False)
    axes[0].set_title(f"Sensor bias, {names(estimators)}: truth vs estimate", loc="left")
    axes[-1].set_xlabel("timestep")
    finish(axes[0])
    return figure


def confidence_interval_figure(k, x_true, estimators):
    """4. Position and heading error against each estimator's own 95%
    confidence interval, a filter and its smoother overlaid so they're
    directly comparable."""
    sigma_scale = norm.ppf(0.5 + CONFIDENCE / 2)
    specs = (("x error [m]", 0), ("y error [m]", 1), ("heading error [rad]", 2))
    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for axis, (label, i) in zip(axes, specs):
        axis.axhline(0, color=BAND, linewidth=1, linestyle="--")
        for estimator in estimators:
            x_est, P_est = estimator.x, estimator.P
            error = angle_diff(x_true[:, i], x_est[:, i]) if i == HEADING else x_true[:, i] - x_est[:, i]
            # clip: the smoother's P has no PSD guarantee, see masked_quadratic_form
            sigma = sigma_scale * np.sqrt(np.clip(P_est[:, i, i], 0, None))
            axis.fill_between(k, -sigma, sigma, color=estimator.color, alpha=0.15,
                             linewidth=0, label=f"{estimator.name} 95%")
            axis.plot(k, error, color=estimator.color, linewidth=1,
                      label=estimator.name)
        axis.set_ylabel(label)
        axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
        axis.set_axisbelow(True)
        axis.spines[["top", "right"]].set_visible(False)
    axes[0].set_title(f"{names(estimators)}: error with 95% confidence interval", loc="left")
    axes[-1].set_xlabel("timestep")
    finish(axes[0])
    return figure


def nis_figure(specs):
    """5a. NIS at each fix, against the per-sample 95% band, one subplot per
    sensor since each has its own dimension (and so its own expected value
    and band). There are too few fixes for a running-mean plot to mean
    anything, each point here is one real measurement.
    specs: (sensor name, fix timesteps, filter name, color, nis, dof)"""
    figure, axes = plt.subplots(len(specs), 1, figsize=(11, 3 * len(specs)), sharex=True)
    for axis, (name, k_fix, label, color, nis, dof) in zip(axes, specs):
        low, high = chi2_interval(dof)
        axis.axhspan(low, high, color=BAND, alpha=0.15, linewidth=0,
                    label=f"95% band [{low:.2f}, {high:.2f}]")
        axis.axhline(dof, color=BAND, linewidth=1, linestyle="--",
                    label=f"expected value ({dof})")
        axis.plot(k_fix, nis, color=color, linewidth=0.8, alpha=0.6)
        axis.scatter(k_fix, nis, color=color, s=20, zorder=3, label=label)
        axis.set_title(f"{name}, {label}", loc="left")
        axis.set_ylabel("NIS")
        axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
        axis.set_axisbelow(True)
        axis.spines[["top", "right"]].set_visible(False)
        axis.legend(loc="best", frameon=False, fontsize=8)
    axes[-1].set_xlabel("timestep")
    return figure


def nees_figure(k, series):
    """5b. Running mean of position NEES for a filter and its smoother, against the
    genuine per-sample 95% region (same one NIS uses). Position NEES here
    stays autocorrelated out past 600 steps, there's no AVERAGING_WINDOW
    small enough to smooth usefully but large enough to treat as "many
    independent samples", so the line is smoothed for readability only, not
    compared against a band that assumes averaging power it doesn't have.
    NaN (P not PSD, see masked_quadratic_form) is zeroed only for this plot;
    the printed summary above is the honest number.
    series: [(label, color, nees)]"""
    low, high = chi2_interval(2)  # per-sample, dof=2, no averaging assumed

    title = "position NEES, " + " and ".join(label for label, _, _ in series)
    figure, axis = new_figure(title, xlabel="timestep",
                              ylabel=f"position NEES ({AVERAGING_WINDOW}-step running mean, "
                                    "per-sample 95% region)",
                              equal=False)
    axis.axhspan(low, high, color=BAND, alpha=0.15, linewidth=0,
                label=f"95% band [{low:.2f}, {high:.2f}]")
    axis.axhline(2, color=BAND, linewidth=1, linestyle="--", label="expected value (2)")
    for label, color, values in series:
        values = np.nan_to_num(values)
        window = min(AVERAGING_WINDOW, len(values))
        axis.plot(k[window - 1:], running_mean(values, window), color=color,
                 linewidth=1.5, label=label)
    finish(axis)
    return figure


def variance_figure(k, estimators):
    """6. Position variance over time for a filter and its smoother, the INS analogue
    of the random walk case's variance plot (Sarkka figure 12.2). Log scale,
    since an outage can make a filter's variance span several orders of
    magnitude more than a smoother's."""
    trace = lambda P: np.trace(P[:, :2, :2], axis1=1, axis2=2)
    figure, axis = new_figure(f"{names(estimators)} variance", xlabel="timestep",
                              ylabel="trace(P) = x variance + y variance", equal=False)
    axis.set_yscale("log")
    for estimator in estimators:
        axis.plot(k, trace(estimator.P), color=estimator.color, linewidth=1.5,
                  label=f"{estimator.name} trace(P)")
    finish(axis)
    return figure


def ekf_vs_ukf_figure(k, pairs):
    """8. How far apart the EKF and the UKF are, and the ERTS and the URTSS,
    over time. Where the two linearise differently (turns, the end of a GNSS
    outage) they separate, so this shows where the nonlinearity matters.
    pairs: [(EKF-like, UKF-like)] as Estimators, log scale on the differences
    since they are orders of magnitude below the errors themselves."""
    specs = (("position difference [m]", True), ("heading difference [rad]", True),
             ("position trace(P) ratio, UKF-like / EKF-like", False))
    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    trace = lambda P: np.trace(P[:, :2, :2], axis1=1, axis2=2)
    for axis, (label, log) in zip(axes, specs):
        axis.set_ylabel(label)
        axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
        axis.set_axisbelow(True)
        axis.spines[["top", "right"]].set_visible(False)
        if log:
            axis.set_yscale("log")
    axes[2].axhline(1, color=BAND, linewidth=1, linestyle="--")
    for reference, other in pairs:
        name = f"{other.name} vs {reference.name}"
        position = np.linalg.norm(other.x[:, POSITION] - reference.x[:, POSITION], axis=1)
        heading = np.abs(angle_diff(other.x[:, HEADING], reference.x[:, HEADING]))
        # an exact zero has no place on a log axis
        axes[0].plot(k, np.where(position > 0, position, np.nan), color=other.color,
                     linewidth=1, label=name)
        axes[1].plot(k, np.where(heading > 0, heading, np.nan), color=other.color,
                     linewidth=1, label=name)
        axes[2].plot(k, trace(other.P) / trace(reference.P), color=other.color,
                     linewidth=1, label=name)
    axes[0].set_title("EKF vs UKF", loc="left")
    axes[-1].set_xlabel("timestep")
    finish(axes[0])
    return figure


def sigma_point_health_figure(k, estimators):
    """9. Smallest eigenvalue of the full state covariance over time. The
    unscented transform's centre weight can be negative, so unlike the EKF's
    Joseph form nothing algebraically keeps P positive semidefinite, and the
    URTSS has no guarantee either. A line dipping below zero means P stopped
    being a covariance. Symlog scale, since healthy values sit near 1e-6."""
    figure, axis = new_figure("Smallest eigenvalue of P", xlabel="timestep",
                              ylabel="min eigenvalue of P (symlog)", equal=False)
    axis.set_yscale("symlog", linthresh=1e-9)
    axis.axhline(0, color=BAND, linewidth=1, linestyle="--")
    for estimator in estimators:
        smallest = np.linalg.eigvalsh(estimator.P).min(axis=-1)
        axis.plot(k, smallest, color=estimator.color, linewidth=1.2,
                  label=f"{estimator.name} ({np.sum(smallest < 0)} steps < 0)")
    finish(axis)
    return figure


def load_filter_data(data_dir, prefix):
    """The three fix tables one filter produced, or None if they aren't there."""
    names = ("gnss_fixes", "magnetometer_fixes", "dvl_fixes")
    paths = [data_dir / f"{prefix}{name}.csv" for name in names]
    if not all(path.exists() for path in paths):
        return None
    return [load(path) for path in paths]


def main():
    data_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent.parent / "data"
    figures_dir = data_dir.parent / "figures"
    figures_dir.mkdir(exist_ok=True)
    truth = load(data_dir / "truth.csv")
    filtered = load(data_dir / "filter.csv")
    smoothed = load(data_dir / "smoother.csv")
    gnss_fixes, magnetometer_fixes, dvl_fixes = load_filter_data(data_dir, "")

    k = truth["timestep"].astype(int)
    x_true = vectors(truth, "x_true", STATE_DIM)

    gnss_steps = gnss_fixes["timestep"].astype(int)
    z_fix = vectors(gnss_fixes, "z", MEASUREMENT_DIM)

    magnetometer_steps = magnetometer_fixes["timestep"].astype(int)
    magnetometer_heading = magnetometer_fixes["z_0"]
    magnetometer_position = x_true[magnetometer_steps, :2]

    dvl_steps = dvl_fixes["timestep"].astype(int)
    dvl_velocity_body = vectors(dvl_fixes, "z", 2)
    dvl_position = x_true[dvl_steps, :2]  # DVL has no position of its own
    dvl_heading = x_true[dvl_steps, HEADING]

    ekf = Estimator("EKF", FILTER, vectors(filtered, "x_updated", STATE_DIM),
                    matrices(filtered, "P_updated", STATE_DIM))
    erts = Estimator("ERTS", SMOOTHER, vectors(smoothed, "x_smoothed", STATE_DIM),
                     matrices(smoothed, "P_smoothed", STATE_DIM))
    filters, smoothers = [ekf], [erts]
    # the UKF/URTSS files only exist for runs made with a scenario that has them
    if (data_dir / "ukf_filter.csv").exists() and (data_dir / "ukf_smoother.csv").exists():
        ukf_filtered = load(data_dir / "ukf_filter.csv")
        ukf_smoothed = load(data_dir / "ukf_smoother.csv")
        filters.append(Estimator("UKF", UKF, vectors(ukf_filtered, "x_updated", STATE_DIM),
                                 matrices(ukf_filtered, "P_updated", STATE_DIM)))
        smoothers.append(Estimator("URTSS", URTSS, vectors(ukf_smoothed, "x_smoothed", STATE_DIM),
                                   matrices(ukf_smoothed, "P_smoothed", STATE_DIM)))
    estimators = filters + smoothers

    def nis_of(fixes):
        gnss, magnetometer, dvl = fixes
        return (quadratic_form(vectors(gnss, "innovation", MEASUREMENT_DIM),
                               matrices(gnss, "S", MEASUREMENT_DIM)),
                quadratic_form(vectors(magnetometer, "innovation", MAGNETOMETER_DIM),
                               matrices(magnetometer, "S", MAGNETOMETER_DIM)),
                quadratic_form(vectors(dvl, "innovation", DVL_DIM),
                               matrices(dvl, "S", DVL_DIM)))

    # NIS per filter, each filter has its own innovations and S
    nis_by_filter = {"EKF": nis_of((gnss_fixes, magnetometer_fixes, dvl_fixes))}
    ukf_fixes = load_filter_data(data_dir, "ukf_")
    if len(filters) > 1 and ukf_fixes is not None:
        nis_by_filter["UKF"] = nis_of(ukf_fixes)

    def position_nees(estimator):
        return masked_quadratic_form(x_true[:, POSITION] - estimator.x[:, POSITION],
                                     estimator.P[:, :2, :2])

    def position_error(estimator):
        return np.linalg.norm(x_true[:, POSITION] - estimator.x[:, POSITION], axis=1)

    print(f"{len(k)} timesteps, {len(gnss_steps)} GNSS fixes\n")
    print("consistency (time-averaged):")
    for name, (gnss_nis, magnetometer_nis, dvl_nis) in nis_by_filter.items():
        summarise(f"ANIS GNSS {name}", gnss_nis, MEASUREMENT_DIM)
        summarise(f"ANIS magnetometer {name}", magnetometer_nis, MAGNETOMETER_DIM)
        summarise(f"ANIS DVL {name}", dvl_nis, DVL_DIM)
    for estimator in estimators:
        summarise(f"ANEES position {estimator.name}", position_nees(estimator), 2)
    print("\naccuracy:")
    for estimator in estimators:
        rmse = np.sqrt(np.mean(position_error(estimator) ** 2))
        print(f"  RMSE position {estimator.name:<8}{rmse:6.4f} m")
    for estimator in estimators:
        heading_error = angle_diff(x_true[:, HEADING], estimator.x[:, HEADING])
        print(f"  RMSE heading  {estimator.name:<8}{np.degrees(np.sqrt(np.mean(heading_error ** 2))):6.3f} deg")

    print("\nsigma point health (smallest eigenvalue of the full P):")
    for estimator in estimators:
        smallest = np.linalg.eigvalsh(estimator.P).min(axis=-1)
        print(f"  {estimator.name:<8}min {smallest.min():10.3e}  "
              f"{np.sum(smallest < 0)} of {len(smallest)} steps below zero")

    # each smoother's uncertainty against the filter it ran on
    print()
    for filter_, smoother in zip(filters, smoothers):
        trace_filter = np.trace(filter_.P[:, :2, :2], axis1=1, axis2=2)
        trace_smoother = np.trace(smoother.P[:, :2, :2], axis1=1, axis2=2)
        # relative, the csv files only keep about 6 significant digits
        always_smaller = np.all(trace_smoother <= trace_filter * (1 + 1e-6))
        print(f"{smoother.name} trace(P) <= {filter_.name} trace(P) at every step: {always_smaller}")

    # the same set of figures for each variant, so they can be put side by
    # side, the EKF/ERTS ones keep the names the results pages already link to
    figures = {"ins_1_sensor_fixes.png": sensor_fixes_figure(
        x_true, z_fix, magnetometer_position, magnetometer_heading,
        dvl_position, dvl_velocity_body, dvl_heading)}
    variants = [("ukf_" if filter_.name == "UKF" else "", filter_, smoother)
                for filter_, smoother in zip(filters, smoothers)]
    for prefix, filter_, smoother in variants:
        pair = [filter_, smoother]
        figures[f"{prefix}ins_2_filter_vs_smoother.png"] = filter_vs_smoother_figure(x_true, pair)
        figures[f"{prefix}ins_3_confidence_interval.png"] = confidence_interval_figure(k, x_true, pair)
        if filter_.name in nis_by_filter:
            gnss_nis, magnetometer_nis, dvl_nis = nis_by_filter[filter_.name]
            nis_specs = [
                ("GNSS", k[gnss_steps], filter_.name, filter_.color, gnss_nis, MEASUREMENT_DIM),
                ("Magnetometer", magnetometer_steps, filter_.name, filter_.color, magnetometer_nis, MAGNETOMETER_DIM),
                ("DVL", dvl_steps, filter_.name, filter_.color, dvl_nis, DVL_DIM),
            ]
            figures[f"{prefix}ins_4_nis.png"] = nis_figure(nis_specs)
        figures[f"{prefix}ins_5_nees.png"] = nees_figure(k, [
            (e.name, e.color, position_nees(e)) for e in pair])
        figures[f"{prefix}ins_6_variance.png"] = variance_figure(k, pair)
        figures[f"{prefix}ins_7_bias.png"] = bias_figure(k, x_true, pair)

    # figures that put the two variants against each other
    if len(filters) > 1:
        figures["ins_8_ekf_vs_ukf.png"] = ekf_vs_ukf_figure(
            k, [tuple(filters), tuple(smoothers)])
        figures["ins_9_sigma_point_health.png"] = sigma_point_health_figure(k, estimators)

    print()
    for name, figure in figures.items():
        figure.tight_layout()
        figure.savefig(figures_dir / name, dpi=150)
        print(f"wrote {figures_dir / name}")
    plt.show()


if __name__ == "__main__":
    main()
