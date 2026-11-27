#!/usr/bin/env python3
"""Compares the 2-D INS Monte Carlo at bias time constant 1000 s (results/INS2D)
against a shorter one, tau s (results/INS2D_tau<tau>). The figures go to
results/INS2D_tau<tau>/comparison/.

  1_bias_process         simulated truth gyro bias and its 2 sigma envelope for both
                         time constants, the reason for the rerun
  2_filter_bias_sigma    the filter's gyro bias standard deviation over one run, with
                         and without DVL and magnetometer, which explains why only some
                         cases change
  3_rmse_over_time       Monte Carlo position RMSE over time for the cases that change
                         and for one that does not
  4_anees                ANEES with its 95% t-interval per case and estimator
  changes.csv            every RMSE, ANEES and ANIS of both runs side by side

Everything uses the matched white-noise variant. Figure 2 runs the simulator once
per case and time constant (seed 42), the rest reads the summary.npy files.

usage: compare_ins_2d_time_constants.py [tau]   (default 200)
"""

import re
import sys
import subprocess
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import monte_carlo_ins_2d as mc
from legend_placement import place_legend

OLD = mc.PROJECT / "results" / "INS2D"
# the shorter time constant, set from the command line in main()
SHORT = int(sys.argv[1]) if len(sys.argv) > 1 else 200
NEW = mc.PROJECT / "results" / f"INS2D_tau{SHORT}"
OUT = NEW / "comparison"
TAUS = {1000: OLD, SHORT: NEW}
COLORS = {1000: "#2a78d6", SHORT: "#eb6834"}
VARIANT = "matched"
SEED = 42

plt.rcParams.update({"font.size": 13, "axes.titlesize": 14})


def label(tau):
    return rf"$\tau$ = {tau} s"


def summary(root, case):
    return mc.load_summary(root / case / "monte_carlo" / VARIANT / "summary.npy")


def finish(figure, name):
    figure.tight_layout()
    mc.place_legends(figure)
    figure.savefig(OUT / f"{name}.pdf")
    figure.savefig(OUT / f"{name}.png", dpi=150)
    plt.close(figure)


# --- 1: the bias process itself ------------------------------------------------

def bias_process_figure():
    """Truth gyro bias as the simulator draws it, b_k = a b_{k-1} + w_k with
    a = exp(-dt / tau) and var(w) = s2 (1 - a^2), started at zero."""
    text = (OLD / "gnss_outage" / "matched" / "config.yaml").read_text()
    dt, s2 = mc.number(text, "dt"), mc.number(text, "gyro_noise_variance")
    steps = int(mc.number(text, "timesteps"))
    t = np.arange(1, steps + 1) * dt
    rng = np.random.default_rng(SEED)
    figure, axes = plt.subplots(1, 2, figsize=(11, 3.8), sharey=True)
    for axis, tau in zip(axes, TAUS):
        a = np.exp(-dt / tau)
        noise = rng.normal(0.0, np.sqrt(s2 * (1 - a * a)), size=(5, steps))
        paths = np.zeros_like(noise)
        for k in range(steps):
            paths[:, k] = (a * paths[:, k - 1] if k else 0.0) + noise[:, k]
        envelope = 2 * np.sqrt(s2 * (1 - np.exp(-2 * t / tau)))
        axis.fill_between(t, -envelope, envelope, color=COLORS[tau], alpha=0.15,
                          linewidth=0, label=r"$\pm 2\sigma(t)$")
        for path in paths:
            axis.plot(t, path, color=COLORS[tau], linewidth=0.8, alpha=0.8)
        axis.axhline(2 * np.sqrt(s2), color=mc.BAND, linestyle="--", linewidth=1,
                     label=r"stationary $\pm 2\sigma$")
        axis.axhline(-2 * np.sqrt(s2), color=mc.BAND, linestyle="--", linewidth=1)
        mc.style(axis, f"truth gyro bias, {label(tau)}", "bias [rad/s]" if tau == 1000 else "", None)
        axis.set_xlabel("time [s]")
    finish(figure, "1_bias_process")


# --- 2: how well the filter knows the gyro bias -----------------------------------

def filter_bias_sigma(case_dir):
    """The EKF's predicted gyro bias standard deviation over one seed."""
    text = mc.run_config((case_dir / "config.yaml").read_text(), case_dir, VARIANT, SEED)
    with tempfile.TemporaryDirectory(prefix="mc_ins2d_") as scratch:
        run_dir = Path(scratch)
        (run_dir / "config.yaml").write_text(text)
        subprocess.run([mc.EXECUTABLE, run_dir], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        column = mc.read_columns(run_dir / "data" / "ekf_filter.csv", ["P_predicted_7_7"])
    return np.sqrt(column["P_predicted_7_7"])


def filter_bias_sigma_figure():
    cases = {"only_gnss/figure_eight": "GNSS only",
             "multi_sensor_fusion/figure_eight": "GNSS, magnetometer and DVL"}
    text = (OLD / "only_gnss" / "figure_eight" / "config.yaml").read_text()
    dt, s2 = mc.number(text, "dt"), mc.number(text, "gyro_noise_variance")
    figure, axes = plt.subplots(1, 2, figsize=(11, 3.8), sharey=True)
    for axis, (case, title) in zip(axes, cases.items()):
        for tau, root in TAUS.items():
            sigma = filter_bias_sigma(root / case)
            axis.plot(np.arange(len(sigma)) * dt, sigma, color=COLORS[tau],
                      linewidth=1.5, label=label(tau))
        axis.axhline(np.sqrt(s2), color=mc.BAND, linestyle="--", linewidth=1,
                     label=r"stationary $\sigma$")
        axis.set_yscale("log")
        mc.style(axis, f"EKF gyro bias std, {title}", "std [rad/s]" if axis is axes[0] else "", None)
        axis.set_xlabel("time [s]")
    finish(figure, "2_filter_bias_sigma")


# --- 3: position RMSE over time --------------------------------------------------

def rmse_over_time_figure():
    panels = [("only_gnss/figure_eight", ("ekf_", "EKF"), "EKF, GNSS only, figure-eight"),
              ("ukf/heading_stress", ("ukf_", "UKF"), "UKF, heading stress, four-corner"),
              ("gnss_outage/matched", ("ekf_", "EKF"), "EKF, GNSS outage, four-corner")]
    figure, axes = plt.subplots(len(panels), 1, figsize=(11, 3.2 * len(panels)), sharex=True)
    for axis, (case, key, title) in zip(axes, panels):
        denied = None
        for tau, root in TAUS.items():
            s = summary(root, case)
            denied = s["denied"]
            t = np.arange(s["timesteps"]) * s["dt"]
            axis.plot(t, s[key]["rmse_t"], color=COLORS[tau], linewidth=1.2, label=label(tau))
        axis.set_ylim(bottom=0)
        mc.style(axis, f"position RMSE over {s['runs']} runs, {title}", "RMSE [m]", denied)
    axes[-1].set_xlabel("time [s]")
    finish(figure, "3_rmse_over_time")


# --- 4: ANEES per case ---------------------------------------------------------

ANEES_ROWS = [
    ("multi_sensor_fusion/figure_eight", ("ekf_", "EKF"), "fusion, EKF"),
    ("gnss_outage/matched", ("ekf_", "EKF"), "outage, EKF"),
    ("dead_reckoning/figure_eight", ("ekf_", "EKF"), "dead reckoning, EKF"),
    ("only_gnss/figure_eight", ("ekf_", "EKF"), "GNSS only, EKF"),
    ("only_gnss/figure_eight", ("ekf_", "ERTS"), "GNSS only, ERTS"),
    ("ieks/heading_stress", ("ekf_", "EKF"), "heading stress, EKF"),
    ("ieks/heading_stress", ("ukf_", "UKF"), "heading stress, UKF"),
    ("ieks/heading_stress", ("ukf_", "URTSS"), "heading stress, URTSS"),
    ("ieks/heading_stress", ("ieks_", "IEKF"), "heading stress, IEKF"),
    ("ieks/heading_stress", ("ieks_", "IEKS"), "heading stress, IEKS"),
]


def anees_figure():
    figure, axis = plt.subplots(figsize=(11, 0.55 * len(ANEES_ROWS) + 1.4))
    rows = np.arange(len(ANEES_ROWS))[::-1]
    for offset, (tau, root) in zip((0.15, -0.15), TAUS.items()):
        for row, (case, key, _) in zip(rows, ANEES_ROWS):
            mean, _, low, high = mc.t_interval(summary(root, case)[key]["anees"])
            axis.plot([low, high], [row + offset] * 2, color=COLORS[tau], linewidth=2,
                      solid_capstyle="round")
            axis.plot(mean, row + offset, "o", color=COLORS[tau], markersize=8,
                      markeredgecolor="white", markeredgewidth=1.5,
                      label=label(tau) if row == rows[0] else None)
    axis.axvline(2, color=mc.BAND, linestyle="--", linewidth=1, label="expected 2")
    axis.set_xlim(left=0)
    axis.set_yticks(rows, [name for _, _, name in ANEES_ROWS])
    mc.style(axis, "position ANEES, mean and 95% t-interval over 21 runs", "", None)
    axis.grid(axis="y", visible=False)
    axis.set_xlabel("ANEES (dof 2)")
    figure.tight_layout()
    place_legend(axis, **mc.LEGEND)
    figure.savefig(OUT / "4_anees.pdf")
    figure.savefig(OUT / "4_anees.png", dpi=150)
    plt.close(figure)


# --- table -----------------------------------------------------------------------

def changes_table():
    lines = [f"case,estimator,quantity,tau_1000,tau_{SHORT},verdict_1000,verdict_{SHORT}"]
    pattern = re.compile(r"\s+(\S.*?)\s{2,}([-0-9.]+) \+-.*?(?:->\s*(\S+))?$")
    for config in sorted(NEW.rglob("config.yaml")):
        if "monte_carlo" in config.parts:
            continue
        case = config.parent.relative_to(NEW).as_posix()
        parsed = []
        for root in (OLD, NEW):
            values, section = {}, None
            for line in (root / case / "monte_carlo" / VARIANT / "stats.txt").read_text().splitlines():
                if line.startswith("position error ["):
                    section = "RMSE"
                elif line.startswith("position error inside"):
                    section = "outage RMSE"
                elif line.startswith("ANEES"):
                    section = "ANEES"
                elif line.startswith("ANIS"):
                    section = "ANIS"
                elif line and not line.startswith(" "):
                    section = None
                match = pattern.match(line)
                if match and section:
                    values[(match.group(1).strip(), section)] = (match.group(2), match.group(3) or "")
            parsed.append(values)
        for (name, quantity), (old, old_verdict) in parsed[0].items():
            if (name, quantity) in parsed[1]:
                new, new_verdict = parsed[1][(name, quantity)]
                lines.append(f"{case},{name},{quantity},{old},{new},{old_verdict},{new_verdict}")
    (OUT / "changes.csv").write_text("\n".join(lines) + "\n")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    bias_process_figure()
    filter_bias_sigma_figure()
    rmse_over_time_figure()
    anees_figure()
    changes_table()
    print(f"written to {OUT}")


if __name__ == "__main__":
    main()
