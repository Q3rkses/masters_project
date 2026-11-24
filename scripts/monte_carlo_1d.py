#!/usr/bin/env python3
"""Monte Carlo over seeds for the 1-D random walk.

Runs the base config (results/1D/config.yaml) once per seed, each run in a
temporary folder that is deleted afterwards, and keeps only the Monte Carlo
data in <mc_dir>/data/:

  per_run.csv       one row per seed, with that run's time-averaged errors and
                    consistency statistics, so the seeds are recorded too
  per_timestep.csv  one row per timestep, with the statistics averaged over
                    all runs

With M independent runs the NEES averaged over the runs at a single timestep is
chi2(M n) / M, so every timestep gets its own exact band without any
autocorrelation correction.

usage: monte_carlo_1d.py [--skip-run] [--pdf] [mc_dir]
"""

import re
import subprocess
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import chi2, t

PROJECT = Path(__file__).resolve().parent.parent
EXECUTABLE = PROJECT / "build" / "kalman_filtering_and_smoothing"
BASE_CONFIG = PROJECT / "results" / "1D" / "config.yaml"
# 42 as in The Hitchhiker's Guide to the Galaxy, up to 1320, the 22 minutes of
# the Outer Wilds time loop in seconds, both included, so 1279 runs
SEEDS = range(42, 1321)

FILTER = "#2a78d6"
SMOOTHER = "#eb6834"
BAND = "#8a8984"

STATE_DIM = 1
MEASUREMENT_DIM = 1
CONFIDENCE = 0.95

PER_RUN = ["seed", "rmse_filter", "rmse_smoother", "mae_filter", "mae_smoother",
           "anis", "anees_filter", "anees_smoother"]
PER_TIMESTEP = ["timestep", "nis", "nees_filter", "nees_smoother", "rmse_filter",
                "rmse_smoother", "mae_filter", "mae_smoother", "P_filter", "P_smoother"]


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def chi2_interval(dof, samples):
    """Two-sided interval for a chi-square statistic averaged over `samples`."""
    tail = (1 - CONFIDENCE) / 2
    low, high = chi2.ppf([tail, 1 - tail], dof * samples)
    return low / samples, high / samples


def run_one(seed):
    """One run of the base config with the given seed, reduced to its errors."""
    config = re.sub(r"^seed:.*$", f"seed: {seed}", BASE_CONFIG.read_text(), flags=re.M)
    with tempfile.TemporaryDirectory(prefix="mc_1d_") as scratch:
        run_dir = Path(scratch)
        (run_dir / "config.yaml").write_text(config)
        subprocess.run([EXECUTABLE, run_dir], check=True, stdout=subprocess.DEVNULL)
        truth = load(run_dir / "data" / "truth.csv")
        filtered = load(run_dir / "data" / "filter.csv")
        smoothed = load(run_dir / "data" / "smoother.csv")
    x_true = truth["x_true_0"]
    return {
        "k": truth["timestep"],
        "err_filter": x_true - filtered["x_updated_0"],
        "err_smoother": x_true - smoothed["x_smoothed_0"],
        "P_filter": filtered["P_updated_0_0"],
        "P_smoother": smoothed["P_smoothed_0_0"],
        "nis": filtered["innovation_0"] ** 2 / filtered["S_0_0"],
    }


def run_monte_carlo(data_dir):
    """Runs every seed and writes the per-run and per-timestep data."""
    with ProcessPoolExecutor() as pool:
        runs = list(pool.map(run_one, SEEDS, chunksize=16))
    stack = lambda key: np.vstack([run[key] for run in runs])  # runs x timesteps
    err_f, err_s = stack("err_filter"), stack("err_smoother")
    nees_f, nees_s = err_f ** 2 / stack("P_filter"), err_s ** 2 / stack("P_smoother")
    nis = stack("nis")

    per_run = np.column_stack([
        np.array(SEEDS), np.sqrt(np.mean(err_f ** 2, axis=1)), np.sqrt(np.mean(err_s ** 2, axis=1)),
        np.mean(np.abs(err_f), axis=1), np.mean(np.abs(err_s), axis=1),
        nis.mean(axis=1), nees_f.mean(axis=1), nees_s.mean(axis=1)])
    # P does not depend on the data in a linear Kalman filter, so any run's P is every run's P
    per_timestep = np.column_stack([
        runs[0]["k"], nis.mean(axis=0), nees_f.mean(axis=0), nees_s.mean(axis=0),
        np.sqrt(np.mean(err_f ** 2, axis=0)), np.sqrt(np.mean(err_s ** 2, axis=0)),
        np.mean(np.abs(err_f), axis=0), np.mean(np.abs(err_s), axis=0),
        runs[0]["P_filter"], runs[0]["P_smoother"]])

    data_dir.mkdir(parents=True, exist_ok=True)
    np.savetxt(data_dir / "per_run.csv", per_run, delimiter=",", header=",".join(PER_RUN),
               comments="", fmt=["%d"] + ["%.6g"] * (len(PER_RUN) - 1))
    np.savetxt(data_dir / "per_timestep.csv", per_timestep, delimiter=",",
               header=",".join(PER_TIMESTEP), comments="",
               fmt=["%d"] + ["%.6g"] * (len(PER_TIMESTEP) - 1))


def style(axis, title, ylabel):
    axis.set_title(title, loc="left")
    axis.set_xlabel("timestep")
    axis.set_ylabel(ylabel)
    axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
    axis.set_axisbelow(True)
    axis.spines[["top", "right"]].set_visible(False)
    axis.legend(loc="upper right", frameon=False, fontsize=9)


def plot_mc_average(axis, k, series, dof, runs, name):
    """Run-averaged statistic at every timestep against its exact chi2 band."""
    low, high = chi2_interval(dof, runs)
    axis.axhspan(low, high, color=BAND, alpha=0.15, linewidth=0,
                 label=f"95% band [{low:.2f}, {high:.2f}]")
    axis.axhline(dof, color=BAND, linewidth=1, linestyle="--")
    for label, (values, color) in series.items():
        axis.plot(k, values, color=color, linewidth=0.8, label=label)
    style(axis, f"{name} averaged over {runs} runs", name)


def report(per_run, per_timestep):
    seeds = per_run["seed"].astype(int)
    M = len(seeds)
    lines = [f"{M} runs (seeds {seeds[0]} to {seeds[-1]}), "
             f"{len(per_timestep['timestep'])} timesteps each", ""]

    lines.append("accuracy over runs (mean +- std across runs):")
    for label, key in [("RMSE filter", "rmse_filter"), ("RMSE smoother", "rmse_smoother"),
                       ("MAE filter", "mae_filter"), ("MAE smoother", "mae_smoother")]:
        values = per_run[key]
        lines.append(f"  {label:<14} {values.mean():.4f} +- {values.std(ddof=1):.4f}")
    lower = per_run["rmse_smoother"] < per_run["rmse_filter"]
    lines.append(f"  smoother RMSE lower than filter in {lower.sum()}/{M} runs, mean reduction "
                 f"{np.mean(1 - per_run['rmse_smoother'] / per_run['rmse_filter']):.1%}")
    lines.append(f"  smoother MAE mean reduction "
                 f"{np.mean(1 - per_run['mae_smoother'] / per_run['mae_filter']):.1%}")
    lines.append(f"  mean P filter {per_timestep['P_filter'].mean():.4f}, smoother "
                 f"{per_timestep['P_smoother'].mean():.4f} (identical in every run, P does not "
                 f"depend on the data)")

    lines += ["", "consistency, run-averaged at each timestep (band chi2(M)/M):"]
    step_low, step_high = chi2_interval(1, M)
    for label, key in [("NIS", "nis"), ("NEES filter", "nees_filter"),
                       ("NEES smoother", "nees_smoother")]:
        values = per_timestep[key]
        inside = np.mean((values >= step_low) & (values <= step_high))
        lines.append(f"  {label:<14} {inside:5.1%} of timesteps inside [{step_low:.4f}, {step_high:.4f}]")

    # The runs are independent, so the per-run averages are too, and their spread
    # gives an honest interval for the overall average. A chi2 band with an AR(1)
    # effective sample count is far too narrow here, because NEES of a random walk
    # stays correlated much longer than one step.
    lines += ["", "consistency, averaged over all runs and timesteps "
              "(mean +- standard error, 95% t-interval from the spread of the per-run averages):"]
    for label, key in [("ANIS", "anis"), ("ANEES filter", "anees_filter"),
                       ("ANEES smoother", "anees_smoother")]:
        values = per_run[key]
        se = values.std(ddof=1) / np.sqrt(M)
        half = t.ppf(0.5 + CONFIDENCE / 2, M - 1) * se
        low, high = values.mean() - half, values.mean() + half
        verdict = "consistent" if low <= 1 <= high else "INCONSISTENT"
        lines.append(f"  {label:<14} {values.mean():6.4f} +- {se:.4f}  interval [{low:.4f}, {high:.4f}]  "
                     f"range over runs [{values.min():.4f}, {values.max():.4f}]  -> {verdict}")
    return "\n".join(lines)


def main():
    flags = {argument for argument in sys.argv[1:] if argument.startswith("--")}
    arguments = [argument for argument in sys.argv[1:] if not argument.startswith("--")]
    mc_dir = Path(arguments[0]) if arguments else PROJECT / "results" / "1D" / "monte_carlo"
    data_dir = mc_dir / "data"

    if "--skip-run" not in flags:
        run_monte_carlo(data_dir)

    per_run = load(data_dir / "per_run.csv")
    per_timestep = load(data_dir / "per_timestep.csv")
    M = len(per_run)
    k = per_timestep["timestep"]

    text = report(per_run, per_timestep)
    print(text)
    (mc_dir / "stats.txt").write_text(text + "\n")

    figures_dir = mc_dir / "figures"
    figures_dir.mkdir(exist_ok=True)
    figures = {}

    figure, axis = plt.subplots(figsize=(11, 4.5))
    plot_mc_average(axis, k, {"filter": (per_timestep["nees_filter"], FILTER),
                              "smoother": (per_timestep["nees_smoother"], SMOOTHER)},
                    STATE_DIM, M, "NEES")
    figures["mc_nees.png"] = figure

    figure, axis = plt.subplots(figsize=(11, 4.5))
    plot_mc_average(axis, k, {"filter": (per_timestep["nis"], FILTER)},
                    MEASUREMENT_DIM, M, "NIS")
    figures["mc_nis.png"] = figure

    # NIS on top and NEES below, sharing the time axis, as in the single-run analysis
    figure, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    plot_mc_average(axes[0], k, {"filter": (per_timestep["nis"], FILTER)},
                    MEASUREMENT_DIM, M, "NIS")
    plot_mc_average(axes[1], k, {"filter": (per_timestep["nees_filter"], FILTER),
                                 "smoother": (per_timestep["nees_smoother"], SMOOTHER)},
                    STATE_DIM, M, "NEES")
    axes[0].set_xlabel("")
    figures["mc_consistency.png"] = figure

    # error spread across runs against what the filter claims, sqrt(P)
    figure, axis = plt.subplots(figsize=(11, 4.5))
    axis.plot(k, per_timestep["rmse_filter"], color=FILTER, linewidth=0.8,
              alpha=0.7, label="filter RMSE over runs")
    axis.plot(k, np.sqrt(per_timestep["P_filter"]), color=FILTER, linewidth=1.6, linestyle="--",
              label=r"filter $\sqrt{P}$")
    axis.plot(k, per_timestep["rmse_smoother"], color=SMOOTHER, linewidth=0.8,
              alpha=0.7, label="smoother RMSE over runs")
    axis.plot(k, np.sqrt(per_timestep["P_smoother"]), color=SMOOTHER, linewidth=1.6, linestyle="--",
              label=r"smoother $\sqrt{P}$")
    style(axis, f"RMSE across {M} runs against reported standard deviation", "position error")
    figures["mc_rmse_vs_sigma.png"] = figure

    for name, figure in figures.items():
        figure.tight_layout()
        figure.savefig(figures_dir / name, dpi=150)
        if "--pdf" in flags:
            figure.savefig((figures_dir / name).with_suffix(".pdf"))
        print(f"wrote {figures_dir / name}")


if __name__ == "__main__":
    main()
