#!/usr/bin/env python3
"""Monte Carlo over seeds for the 1-D random walk.

Writes one case folder per seed under <mc_dir>/seed_<seed>/, each a copy of the
base config with only the seed changed, runs the executable on all of them,
and then checks consistency across the runs. With M independent runs the
NEES averaged over the runs at a single timestep is chi2(M n) / M, so every
timestep gets its own exact band without any autocorrelation correction.

usage: monte_carlo_1d.py [--skip-run] [--pdf] [mc_dir]
"""

import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.stats import chi2, t

PROJECT = Path(__file__).resolve().parent.parent
EXECUTABLE = PROJECT / "build" / "kalman_filtering_and_smoothing"
BASE_CONFIG = PROJECT / "results" / "1D" / "config.yaml"
SEEDS = range(22, 43)  # 22 up to and including 42, so 21 runs

FILTER = "#2a78d6"
SMOOTHER = "#eb6834"
BAND = "#8a8984"

STATE_DIM = 1
MEASUREMENT_DIM = 1
CONFIDENCE = 0.95


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def chi2_interval(dof, samples):
    """Two-sided interval for a chi-square statistic averaged over `samples`."""
    tail = (1 - CONFIDENCE) / 2
    low, high = chi2.ppf([tail, 1 - tail], dof * samples)
    return low / samples, high / samples


def write_cases(mc_dir):
    base = yaml.safe_load(BASE_CONFIG.read_text())
    for seed in SEEDS:
        case = mc_dir / f"seed_{seed}"
        case.mkdir(parents=True, exist_ok=True)
        config = dict(base, seed=seed)
        (case / "config.yaml").write_text(yaml.safe_dump(config, sort_keys=False))


def load_run(case):
    truth = load(case / "data" / "truth.csv")
    filtered = load(case / "data" / "filter.csv")
    smoothed = load(case / "data" / "smoother.csv")
    x_true = truth["x_true_0"]
    return {
        "k": truth["timestep"],
        "err_filter": x_true - filtered["x_updated_0"],
        "err_smoother": x_true - smoothed["x_smoothed_0"],
        "P_filter": filtered["P_updated_0_0"],
        "P_smoother": smoothed["P_smoothed_0_0"],
        "nis": filtered["innovation_0"] ** 2 / filtered["S_0_0"],
    }


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


def main():
    flags = {argument for argument in sys.argv[1:] if argument.startswith("--")}
    arguments = [argument for argument in sys.argv[1:] if not argument.startswith("--")]
    mc_dir = Path(arguments[0]) if arguments else PROJECT / "results" / "1D" / "monte_carlo"

    if "--skip-run" not in flags:
        write_cases(mc_dir)
        subprocess.run([EXECUTABLE, mc_dir], check=True, stdout=subprocess.DEVNULL)

    runs = [load_run(mc_dir / f"seed_{seed}") for seed in SEEDS]
    M = len(runs)
    k = runs[0]["k"]
    stack = lambda key: np.vstack([run[key] for run in runs])  # runs x timesteps
    err_f, err_s = stack("err_filter"), stack("err_smoother")
    P_f, P_s = stack("P_filter"), stack("P_smoother")
    nis = stack("nis")
    nees_f, nees_s = err_f ** 2 / P_f, err_s ** 2 / P_s

    # per-run numbers, the same ones analyze_1d.py prints for a single run
    rmse_f = np.sqrt(np.mean(err_f ** 2, axis=1))
    rmse_s = np.sqrt(np.mean(err_s ** 2, axis=1))
    mae_f = np.mean(np.abs(err_f), axis=1)
    mae_s = np.mean(np.abs(err_s), axis=1)

    lines = [f"{M} runs (seeds {SEEDS[0]} to {SEEDS[-1]}), {len(k)} timesteps each", ""]
    lines.append("per run:")
    lines.append("  seed   RMSE f   RMSE s    MAE f    MAE s    ANIS  ANEES f  ANEES s")
    for seed, i in zip(SEEDS, range(M)):
        lines.append(f"  {seed:4d}  {rmse_f[i]:7.4f}  {rmse_s[i]:7.4f}  {mae_f[i]:7.4f}  "
                     f"{mae_s[i]:7.4f}  {nis[i].mean():6.4f}  {nees_f[i].mean():7.4f}  "
                     f"{nees_s[i].mean():7.4f}")

    lines += ["", "accuracy over runs (mean +- std across runs):"]
    for name, values in [("RMSE filter", rmse_f), ("RMSE smoother", rmse_s),
                         ("MAE filter", mae_f), ("MAE smoother", mae_s)]:
        lines.append(f"  {name:<14} {values.mean():.4f} +- {values.std(ddof=1):.4f}")
    lines.append(f"  smoother RMSE lower than filter in {np.sum(rmse_s < rmse_f)}/{M} runs, "
                 f"mean reduction {np.mean(1 - rmse_s / rmse_f):.1%}")
    lines.append(f"  mean P filter {P_f.mean():.4f}, smoother {P_s.mean():.4f} "
                 f"(identical in every run, P does not depend on the data)")

    lines += ["", "consistency, run-averaged at each timestep (band chi2(M)/M):"]
    step_low, step_high = chi2_interval(1, M)
    for name, values in [("NIS", nis), ("NEES filter", nees_f), ("NEES smoother", nees_s)]:
        inside = np.mean((values.mean(axis=0) >= step_low) & (values.mean(axis=0) <= step_high))
        lines.append(f"  {name:<14} {inside:5.1%} of timesteps inside [{step_low:.4f}, {step_high:.4f}]")

    # The runs are independent, so the per-run averages are too, and their spread
    # gives an honest interval for the overall average. A chi2 band with an AR(1)
    # effective sample count is far too narrow here, because NEES of a random walk
    # stays correlated much longer than one step.
    lines += ["", "consistency, averaged over all runs and timesteps "
              "(95% t-interval from the spread of the per-run averages):"]
    for name, values in [("ANIS", nis), ("ANEES filter", nees_f), ("ANEES smoother", nees_s)]:
        per_run = values.mean(axis=1)
        half = t.ppf(0.5 + CONFIDENCE / 2, M - 1) * per_run.std(ddof=1) / np.sqrt(M)
        low, high = per_run.mean() - half, per_run.mean() + half
        verdict = "consistent" if low <= 1 <= high else "INCONSISTENT"
        lines.append(f"  {name:<14} {per_run.mean():6.4f}  interval [{low:.4f}, {high:.4f}]  "
                     f"range over runs [{per_run.min():.4f}, {per_run.max():.4f}]  -> {verdict}")

    report = "\n".join(lines)
    print(report)
    (mc_dir / "stats.txt").write_text(report + "\n")

    figures_dir = mc_dir / "figures"
    figures_dir.mkdir(exist_ok=True)
    figures = {}

    figure, axis = plt.subplots(figsize=(11, 4.5))
    plot_mc_average(axis, k, {"filter": (nees_f.mean(axis=0), FILTER),
                              "smoother": (nees_s.mean(axis=0), SMOOTHER)},
                    STATE_DIM, M, "NEES")
    figures["mc_nees.png"] = figure

    figure, axis = plt.subplots(figsize=(11, 4.5))
    plot_mc_average(axis, k, {"filter": (nis.mean(axis=0), FILTER)},
                    MEASUREMENT_DIM, M, "NIS")
    figures["mc_nis.png"] = figure

    # error spread across runs against what the filter claims, sqrt(P)
    figure, axis = plt.subplots(figsize=(11, 4.5))
    axis.plot(k, np.sqrt(np.mean(err_f ** 2, axis=0)), color=FILTER, linewidth=0.8,
              alpha=0.7, label="filter RMSE over runs")
    axis.plot(k, np.sqrt(P_f[0]), color=FILTER, linewidth=1.6, linestyle="--",
              label=r"filter $\sqrt{P}$")
    axis.plot(k, np.sqrt(np.mean(err_s ** 2, axis=0)), color=SMOOTHER, linewidth=0.8,
              alpha=0.7, label="smoother RMSE over runs")
    axis.plot(k, np.sqrt(P_s[0]), color=SMOOTHER, linewidth=1.6, linestyle="--",
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
