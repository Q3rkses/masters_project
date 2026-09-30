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
from scipy.stats import chi2, norm

TRUTH = "#3d3d3a"
FILTER = "#2a78d6"
SMOOTHER = "#eb6834"
BAND = "#8a8984"
GNSS = "#c0392b"
MAGNETOMETER = "#8e44ad"
DVL = "#16a085"

STATE_DIM = 8
MEASUREMENT_DIM = 2  # GNSS position only
MAGNETOMETER_ARROW_LENGTH = 3.0  # meters, purely for visibility
FIX_STRIDE = 20  # magnetometer/DVL fire far more often than GNSS; plot every Nth
CONFIDENCE = 0.95
ELLIPSES = 6  # confidence ellipses drawn along each estimated trajectory
AVERAGING_WINDOW = 150  # raw per-step NEES is noise; the running mean is readable

# where each part sits in the state [x, y, psi, u, v, b_ax, b_ay, b_gyro]
POSITION = slice(0, 2)
HEADING = 2


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
    axis.scatter(z_fix[:, 0], z_fix[:, 1], s=30, color=GNSS, zorder=5,
                label="GNSS fix", marker="x")


def draw_magnetometer_fixes(axis, position_fix, heading_fix):
    """Arrows at the true position when a magnetometer fix arrived, pointing
    in the measured (noisy) heading -- shows both where and what it read."""
    dx = MAGNETOMETER_ARROW_LENGTH * np.cos(heading_fix)
    dy = MAGNETOMETER_ARROW_LENGTH * np.sin(heading_fix)
    axis.quiver(position_fix[:, 0], position_fix[:, 1], dx, dy, color=MAGNETOMETER,
               angles="xy", scale_units="xy", scale=1, width=0.005, zorder=5,
               label="magnetometer fix")


def draw_dvl_fixes(axis, z_fix):
    axis.scatter(z_fix[:, 0], z_fix[:, 1], s=30, color=DVL, zorder=5,
                label="DVL fix", marker="^")


def draw_truth(axis, x_true):
    axis.plot(x_true[:, 0], x_true[:, 1], color=TRUTH, linewidth=1.2,
              label="ground truth", zorder=3)
    axis.scatter(x_true[0, 0], x_true[0, 1], s=30, color=TRUTH, zorder=6, label="start")


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


def filter_vs_smoother_figure(x_true, x_filter, P_filter, x_smoother, P_smoother):
    """2+3. Ground truth against both the filter and the smoother, each with
    its own 95% confidence ellipses, in one window."""
    indices = np.linspace(0, len(x_true) - 1, ELLIPSES).astype(int)
    figure, axis = new_figure(f"Filter vs smoother, all {len(x_true)} timesteps")
    draw_truth(axis, x_true)
    draw_estimate(axis, indices, x_filter, P_filter, FILTER, "filter")
    draw_estimate(axis, indices, x_smoother, P_smoother, SMOOTHER, "smoother")
    finish(axis)
    return figure


def gnss_fixes_figure(x_true, z_gnss):
    """1a. Ground truth against the GNSS fixes actually used by the filter."""
    figure, axis = new_figure("GNSS fixes")
    draw_truth(axis, x_true)
    draw_gnss_fixes(axis, z_gnss)
    finish(axis)
    return figure


def magnetometer_fixes_figure(x_true, position_fix, heading_fix):
    """1b. Ground truth against magnetometer fixes. Magnetometer fires far
    more often than GNSS, so only every FIX_STRIDE-th one is drawn."""
    figure, axis = new_figure("Magnetometer fixes")
    draw_truth(axis, x_true)
    draw_magnetometer_fixes(axis, position_fix[::FIX_STRIDE], heading_fix[::FIX_STRIDE])
    finish(axis)
    return figure


def dvl_fixes_figure(x_true, z_dvl):
    """1c. Ground truth against DVL fixes. DVL fires far more often than
    GNSS, so only every FIX_STRIDE-th one is drawn."""
    figure, axis = new_figure("DVL fixes")
    draw_truth(axis, x_true)
    draw_dvl_fixes(axis, z_dvl[::FIX_STRIDE])
    finish(axis)
    return figure


def bias_figure(k, x_true, x_filter, x_smoother):
    """True vs estimated sensor bias over time, one window, three stacked
    plots -- can the filter learn it?"""
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


def confidence_interval_figure(k, x_true, x_filter, P_filter, x_smoother, P_smoother):
    """4. Position and heading error against each estimator's own 95%
    confidence interval, filter and smoother overlaid so they're directly
    comparable."""
    sigma_scale = norm.ppf(0.5 + CONFIDENCE / 2)
    specs = (("x error [m]", 0), ("y error [m]", 1), ("heading error [rad]", 2))
    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for axis, (label, i) in zip(axes, specs):
        axis.axhline(0, color=BAND, linewidth=1, linestyle="--")
        for x_est, P_est, color, name in ((x_filter, P_filter, FILTER, "filter"),
                                          (x_smoother, P_smoother, SMOOTHER, "smoother")):
            error = angle_diff(x_true[:, i], x_est[:, i]) if i == HEADING else x_true[:, i] - x_est[:, i]
            # clip: the smoother's P has no PSD guarantee, see masked_quadratic_form
            sigma = sigma_scale * np.sqrt(np.clip(P_est[:, i, i], 0, None))
            axis.fill_between(k, -sigma, sigma, color=color, alpha=0.15, linewidth=0,
                             label=f"{name} 95%")
            axis.plot(k, error, color=color, linewidth=1, label=name)
        axis.set_ylabel(label)
        axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
        axis.set_axisbelow(True)
        axis.spines[["top", "right"]].set_visible(False)
    axes[0].set_title("Error with 95% confidence interval", loc="left")
    axes[-1].set_xlabel("timestep")
    finish(axes[0])
    return figure


def nis_figure(k_fix, nis):
    """5a. NIS at each of the ~dozen real GNSS fixes, against the per-sample
    95% band. There are too few fixes for a running-mean plot to mean
    anything -- each point here is one real measurement."""
    low, high = chi2_interval(MEASUREMENT_DIM)
    figure, axis = new_figure("NIS per GNSS fix", xlabel="timestep",
                              ylabel="NIS", equal=False)
    axis.axhspan(low, high, color=BAND, alpha=0.15, linewidth=0,
                label=f"95% band [{low:.2f}, {high:.2f}]")
    axis.axhline(MEASUREMENT_DIM, color=BAND, linewidth=1, linestyle="--",
                label=f"expected value ({MEASUREMENT_DIM})")
    axis.plot(k_fix, nis, color=FILTER, linewidth=0.8, alpha=0.6)
    axis.scatter(k_fix, nis, color=FILTER, s=25, zorder=3, label="filter")
    finish(axis)
    return figure


def nees_figure(k, nees_filter, nees_smoother):
    """5b. Running mean of position NEES, filter and smoother, against the
    genuine per-sample 95% region (same one NIS uses). Position NEES here
    stays autocorrelated out past 600 steps -- there's no AVERAGING_WINDOW
    small enough to smooth usefully but large enough to treat as "many
    independent samples", so the line is smoothed for readability only, not
    compared against a band that assumes averaging power it doesn't have.
    NaN (P not PSD, see masked_quadratic_form) is zeroed only for this plot;
    the printed summary above is the honest number."""
    series = {"filter": (np.nan_to_num(nees_filter), FILTER),
             "smoother": (np.nan_to_num(nees_smoother), SMOOTHER)}
    low, high = chi2_interval(2)  # per-sample, dof=2 -- no averaging assumed

    figure, axis = new_figure("position NEES", xlabel="timestep",
                              ylabel=f"position NEES ({AVERAGING_WINDOW}-step running mean, "
                                    "per-sample 95% region)",
                              equal=False)
    axis.axhspan(low, high, color=BAND, alpha=0.15, linewidth=0,
                label=f"95% band [{low:.2f}, {high:.2f}]")
    axis.axhline(2, color=BAND, linewidth=1, linestyle="--", label="expected value (2)")
    for label, (values, color) in series.items():
        window = min(AVERAGING_WINDOW, len(values))
        axis.plot(k[window - 1:], running_mean(values, window), color=color,
                 linewidth=1.5, label=label)
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
    magnetometer_fixes = load(data_dir / "magnetometer_fixes.csv")
    dvl_fixes = load(data_dir / "dvl_fixes.csv")

    k = truth["timestep"].astype(int)
    x_true = vectors(truth, "x_true", STATE_DIM)
    z = vectors(truth, "z", MEASUREMENT_DIM)
    z_fix = z[fix_steps]

    magnetometer_steps = magnetometer_fixes["timestep"].astype(int)
    magnetometer_heading = magnetometer_fixes["z_0"]
    magnetometer_position = x_true[magnetometer_steps, :2]
    z_dvl = vectors(dvl_fixes, "z", 4)[:, :2]  # DVL's own reported position

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

    position_error_filter = np.linalg.norm(x_true[:, POSITION] - x_filter[:, POSITION], axis=1)
    position_error_smoother = np.linalg.norm(x_true[:, POSITION] - x_smoother[:, POSITION], axis=1)

    print(f"{len(k)} timesteps, {len(fix_steps)} GNSS fixes\n")
    print("consistency (time-averaged):")
    summarise("ANIS", nis, MEASUREMENT_DIM)
    summarise("ANEES position filter", position_nees(x_filter, P_filter), 2)
    summarise("ANEES position smoother", position_nees(x_smoother, P_smoother), 2)
    print("\naccuracy:")
    print(f"  RMSE position filter    {np.sqrt(np.mean(position_error_filter ** 2)):6.4f} m")
    print(f"  RMSE position smoother  {np.sqrt(np.mean(position_error_smoother ** 2)):6.4f} m")

    figures = {
        "ins_1a_gnss_fixes.png": gnss_fixes_figure(x_true, z_fix),
        "ins_1b_magnetometer_fixes.png": magnetometer_fixes_figure(
            x_true, magnetometer_position, magnetometer_heading),
        "ins_1c_dvl_fixes.png": dvl_fixes_figure(x_true, z_dvl),
        "ins_2_filter_vs_smoother.png": filter_vs_smoother_figure(
            x_true, x_filter, P_filter, x_smoother, P_smoother),
        "ins_3_confidence_interval.png": confidence_interval_figure(
            k, x_true, x_filter, P_filter, x_smoother, P_smoother),
        "ins_4_nis.png": nis_figure(k[fix_steps], nis),
        "ins_5_nees.png": nees_figure(
            k, position_nees(x_filter, P_filter), position_nees(x_smoother, P_smoother)),
        "ins_7_bias.png": bias_figure(k, x_true, x_filter, x_smoother),
    }

    print()
    for name, figure in figures.items():
        figure.tight_layout()
        figure.savefig(figures_dir / name, dpi=150)
        print(f"wrote {figures_dir / name}")
    plt.show()


if __name__ == "__main__":
    main()
