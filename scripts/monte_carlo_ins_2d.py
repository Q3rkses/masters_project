#!/usr/bin/env python3
"""Monte Carlo over seeds for the 2-D strapdown INS cases.

Every case (a folder under results/INS2D with a config.yaml) is run once per
seed, in two variants that differ only in the white noise of the simulated IMU.

  matched       the truth's white-noise variance per sample is the configured
                intensity divided by dt, which is what the filter's
                continuous-time Q = G D G^T dt assumes.
  conservative  the configured intensity is used as the variance per sample,
                as the single-run results do. The truth then has 1/dt = 100
                times less white noise than the filter believes.

Any other difference between the truth and the filter that a case sets on
purpose (the mistuned bias noise of gnss_outage, the heading error of the
heading stress cases) is kept in both variants.

A single run writes 150 to 220 MB of csv, so every run is reduced to the
numbers below as soon as it finishes and its csv files are deleted. What is
kept per case and variant goes to <case>/monte_carlo/<variant>/: stats.txt,
figures/ and summary.npz, from which --skip-run rebuilds the report.

usage: monte_carlo_ins_2d.py [--skip-run] [--pdf] [--jobs N] [case_dir ...]
"""

import re
import shutil
import subprocess
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter, LogLocator, NullFormatter
import numpy as np
from scipy.stats import chi2, t as student_t

PROJECT = Path(__file__).resolve().parent.parent
EXECUTABLE = PROJECT / "build" / "kalman_filtering_and_smoothing"
INS2D = PROJECT / "results" / "INS2D"
SEEDS = range(22, 43)  # 22 up to and including 42, so 21 runs
VARIANTS = ("matched", "conservative")

STATE_DIM = 8
HEADING = 4
CONFIDENCE = 0.95
NIS_BIN_SECONDS = 10.0  # NIS fixes of all runs are pooled in bins of this length

# filter and smoother pairs, by the prefix of their csv files
PAIRS = {"ekf_": ("EKF", "ERTS"), "ukf_": ("UKF", "URTSS"),
         "iekf_": ("IEKF", "ERTS (IEKF)"), "ieks_": ("IEKF", "IEKS")}
# experiments whose point is comparing estimators, all of which are kept;
# everywhere else only the EKF and ERTS are run
COMPARISONS = {"ukf", "iekf", "ieks"}
SENSORS = {"GNSS": ("gnss_fixes", 2), "magnetometer": ("magnetometer_fixes", 1),
           "DVL": ("dvl_fixes", 2)}

FILTER_COLOR = "#2a78d6"
SMOOTHER_COLOR = "#eb6834"
BAND = "#8a8984"
DENIED = "#f2c48d"
SENSOR_COLORS = {"GNSS": "#c0392b", "magnetometer": "#8e44ad", "DVL": "#16a085"}


# --- configs -----------------------------------------------------------------

def number(text, key):
    match = re.search(rf"^{key}:\s*([-+0-9.eE]+)", text, flags=re.M)
    return float(match.group(1)) if match else None


def denied_window(text):
    start, end = number(text, "gnss_denied_start"), number(text, "gnss_denied_end")
    if start is None or end is None or end <= start or start > 1e8:
        return None
    return start, end


def scale_line(text, key, factor):
    """Multiply the number or list of numbers on the `key:` line by factor."""
    def scaled(match):
        values = re.sub(r"[-+]?[0-9.]+(?:[eE][-+]?[0-9]+)?",
                        lambda m: repr(float(m.group(0)) * factor), match.group(2))
        return f"{match.group(1)}{values}{match.group(3)}"
    text, count = re.subn(rf"^({key}:\s*)(\[[^\]]*\]|\S+)(.*)$", scaled, text, flags=re.M)
    if count != 1:
        raise ValueError(f"expected one {key} line, found {count}")
    return text


def run_config(base, case_dir, variant, seed):
    """The case's config with the seed, the variant and the estimators set."""
    text = re.sub(r"^seed:.*$", f"seed: {seed}", base, flags=re.M)
    if variant == "matched":
        dt = number(base, "dt")
        text = scale_line(text, "gyro_white_noise_intensity", 1 / dt)
        text = scale_line(text, "accelerometer_white_noise_intensity", 1 / dt)
    if case_dir.parent.name not in COMPARISONS:
        text = re.sub(r"^(estimators|run_iekf|run_ieks):.*$\n?", "", text, flags=re.M)
        text += "\nestimators: [ekf]\n"
    return text


# --- one run -----------------------------------------------------------------

def read_columns(path, names):
    """The named columns of a csv file, as a dict of float arrays."""
    with open(path) as handle:
        header = handle.readline().strip().split(",")
    index = [header.index(name) for name in names]
    data = np.loadtxt(path, delimiter=",", skiprows=1, usecols=index, ndmin=2)
    return {name: data[:, i] for i, name in enumerate(names)}


def angle_diff(a, b):
    return np.arctan2(np.sin(a - b), np.cos(a - b))


def position_statistics(x_true, columns, kind):
    """Position error, its NEES and the position trace(P) of one estimator."""
    x = np.column_stack([columns[f"x_{kind}_{i}"] for i in (0, 1, HEADING)])
    P = np.stack([[columns[f"P_{kind}_{i}_{j}"] for j in (0, 1)] for i in (0, 1)])
    P = np.moveaxis(P, -1, 0)  # timesteps x 2 x 2
    error = x_true[:, :2] - x[:, :2]
    nees = np.full(len(error), np.nan)
    valid = np.linalg.eigvalsh(P).min(axis=-1) > 0
    nees[valid] = np.einsum("ti,ti->t", error[valid],
                            np.linalg.solve(P[valid], error[valid][..., None])[..., 0])
    return {
        "distance": np.linalg.norm(error, axis=1),
        "heading": np.degrees(angle_diff(x[:, 2], x_true[:, HEADING])),
        "nees": nees,
        "trace": np.trace(P, axis1=1, axis2=2),
    }


def nis_of(path, dim):
    names = ["timestep"] + [f"innovation_{i}" for i in range(dim)] + \
            [f"S_{i}_{j}" for i in range(dim) for j in range(dim)]
    columns = read_columns(path, names)
    if len(columns["timestep"]) == 0:
        return np.zeros(0), np.zeros(0)
    nu = np.column_stack([columns[f"innovation_{i}"] for i in range(dim)])
    S = np.column_stack([columns[f"S_{i}_{j}"] for i in range(dim) for j in range(dim)])
    S = S.reshape(-1, dim, dim)
    values = np.einsum("ti,ti->t", nu, np.linalg.solve(S, nu[..., None])[..., 0])
    return columns["timestep"].astype(int), values


def run_one(job):
    """Runs one seed of one case and variant, and keeps only what is needed."""
    case_dir, variant, seed = job
    base = (case_dir / "config.yaml").read_text()
    with tempfile.TemporaryDirectory(prefix="mc_ins2d_") as scratch:
        run_dir = Path(scratch)
        (run_dir / "config.yaml").write_text(run_config(base, case_dir, variant, seed))
        subprocess.run([EXECUTABLE, run_dir], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        data = run_dir / "data"
        truth = read_columns(data / "truth.csv", [f"x_true_{i}" for i in range(STATE_DIM)])
        x_true = np.column_stack([truth[f"x_true_{i}"] for i in range(STATE_DIM)])

        estimators, nis = {}, {}
        for prefix, (filter_name, smoother_name) in PAIRS.items():
            if not (data / f"{prefix}filter.csv").exists():
                continue
            for kind, name, file in (("updated", filter_name, "filter"),
                                     ("smoothed", smoother_name, "smoother")):
                names = [f"x_{kind}_{i}" for i in (0, 1, HEADING)] + \
                        [f"P_{kind}_{i}_{j}" for i in (0, 1) for j in (0, 1)]
                columns = read_columns(data / f"{prefix}{file}.csv", names)
                estimators[(prefix, name)] = position_statistics(x_true, columns, kind)
            nis[prefix] = {sensor: nis_of(data / f"{prefix}{file}.csv", dim)
                           for sensor, (file, dim) in SENSORS.items()}
    return {"estimators": estimators, "nis": nis}


# --- statistics --------------------------------------------------------------

def chi2_interval(dof, samples):
    tail = (1 - CONFIDENCE) / 2
    samples = np.asarray(samples, dtype=float)
    low, high = chi2.ppf(tail, dof * samples), chi2.ppf(1 - tail, dof * samples)
    return low / samples, high / samples


def t_interval(per_run):
    """Mean, standard error and 95% t-interval of independent per-run values."""
    per_run = per_run[~np.isnan(per_run)]
    mean = per_run.mean()
    se = per_run.std(ddof=1) / np.sqrt(len(per_run))
    half = student_t.ppf(0.5 + CONFIDENCE / 2, len(per_run) - 1) * se
    return mean, se, mean - half, mean + half


def verdict(low, high, expected):
    if high < expected:
        return "underconfident"
    if low > expected:
        return "overconfident"
    return "consistent"


def summarise(runs, dt, denied):
    """Everything the report and the figures need, as plain arrays."""
    names = list(runs[0]["estimators"])
    timesteps = len(runs[0]["estimators"][names[0]]["distance"])
    summary = {"dt": dt, "runs": len(runs), "timesteps": timesteps,
               "estimators": names, "denied": denied}
    for key in names:
        stack = {field: np.vstack([run["estimators"][key][field] for run in runs])
                 for field in ("distance", "heading", "nees", "trace")}
        distance, heading = stack["distance"], stack["heading"]
        entry = {
            # per timestep, over the runs
            "nees_mean": np.nanmean(stack["nees"], axis=0),
            "nees_count": np.sum(~np.isnan(stack["nees"]), axis=0),
            "rmse_t": np.sqrt(np.mean(distance ** 2, axis=0)),
            "sigma_t": np.sqrt(np.mean(stack["trace"], axis=0)),
            # per run, over time
            "rmse": np.sqrt(np.mean(distance ** 2, axis=1)),
            "mae": np.mean(distance, axis=1),
            "heading_rmse": np.sqrt(np.mean(heading ** 2, axis=1)),
            "heading_mae": np.mean(np.abs(heading), axis=1),
            "anees": np.nanmean(stack["nees"], axis=1),
            "dropped": int(np.sum(np.isnan(stack["nees"]))),
        }
        if denied:
            t = np.arange(timesteps) * dt
            inside = (t >= denied[0]) & (t <= denied[1])
            entry["outage_rmse"] = np.sqrt(np.mean(distance[:, inside] ** 2, axis=1))
            entry["outage_mae"] = np.mean(distance[:, inside], axis=1)
        summary[key] = entry

    summary["nis"] = {}
    for prefix in runs[0]["nis"]:
        for sensor in SENSORS:
            pairs = [run["nis"][prefix][sensor] for run in runs]
            per_run = np.array([values.mean() if len(values) else np.nan for _, values in pairs])
            steps = np.concatenate([steps for steps, _ in pairs])
            values = np.concatenate([values for _, values in pairs])
            summary["nis"][(prefix, sensor)] = {"per_run": per_run, "steps": steps,
                                                "values": values}
    return summary


def report(summary, case_dir, variant):
    M, dt, denied = summary["runs"], summary["dt"], summary["denied"]
    out = [f"{case_dir.relative_to(INS2D)}, {variant} white noise",
           f"{M} runs (seeds {SEEDS[0]} to {SEEDS[-1]}), {summary['timesteps']} timesteps, dt {dt} s"]
    if variant == "matched":
        out.append("truth white noise per sample = configured intensity / dt, as the filter's Q assumes")
    else:
        out.append("truth white noise per sample = configured intensity, 1/dt times less than the filter assumes")
    names = summary["estimators"]

    out += ["", "position error [m], mean +- std over runs of each run's time average:"]
    out.append(f"  {'':<14}{'RMSE':>18}{'MAE':>18}")
    for key in names:
        e = summary[key]
        out.append(f"  {key[1]:<14}{e['rmse'].mean():10.4f} +- {e['rmse'].std(ddof=1):.4f}"
                   f"{e['mae'].mean():10.4f} +- {e['mae'].std(ddof=1):.4f}")
    if denied:
        out += ["", f"position error inside the GNSS outage, {denied[0]:g} to {denied[1]:g} s [m]:"]
        out.append(f"  {'':<14}{'RMSE':>18}{'MAE':>18}")
        for key in names:
            e = summary[key]
            out.append(f"  {key[1]:<14}{e['outage_rmse'].mean():10.4f} +- {e['outage_rmse'].std(ddof=1):.4f}"
                       f"{e['outage_mae'].mean():10.4f} +- {e['outage_mae'].std(ddof=1):.4f}")
    out += ["", "heading error [deg], mean +- std over runs:"]
    out.append(f"  {'':<14}{'RMSE':>18}{'MAE':>18}")
    for key in names:
        e = summary[key]
        out.append(f"  {key[1]:<14}{e['heading_rmse'].mean():10.4f} +- {e['heading_rmse'].std(ddof=1):.4f}"
                   f"{e['heading_mae'].mean():10.4f} +- {e['heading_mae'].std(ddof=1):.4f}")

    low, high = chi2_interval(2, M)
    out += ["", f"position NEES (dof 2), run-averaged at each timestep, band [{low:.4f}, {high:.4f}]:"]
    for key in names:
        e = summary[key]
        full = e["nees_count"] == M
        inside = np.mean((e["nees_mean"][full] >= low) & (e["nees_mean"][full] <= high))
        below = np.mean(e["nees_mean"][full] < low)
        above = np.mean(e["nees_mean"][full] > high)
        note = f"  ({e['dropped']} run-steps dropped, P not PD)" if e["dropped"] else ""
        out.append(f"  {key[1]:<14}{inside:6.1%} inside, {below:6.1%} below, {above:6.1%} above{note}")

    out += ["", "ANEES position (expected 2), mean +- standard error and 95% t-interval over runs:"]
    for key in names:
        mean, se, lo, hi = t_interval(summary[key]["anees"])
        per_run = summary[key]["anees"]
        out.append(f"  {key[1]:<14}{mean:8.4f} +- {se:.4f}  [{lo:.4f}, {hi:.4f}]  "
                   f"runs [{np.nanmin(per_run):.3f}, {np.nanmax(per_run):.3f}]  -> {verdict(lo, hi, 2)}")

    out += ["", "ANIS (expected = measurement dimension), mean +- standard error and 95% t-interval over runs:"]
    for (prefix, sensor), entry in summary["nis"].items():
        label = f"{sensor} {PAIRS[prefix][0]}"
        if np.all(np.isnan(entry["per_run"])):
            out.append(f"  {label:<20} no fixes")
            continue
        dim = SENSORS[sensor][1]
        mean, se, lo, hi = t_interval(entry["per_run"])
        out.append(f"  {label:<20}{mean:8.4f} +- {se:.4f}  [{lo:.4f}, {hi:.4f}]  "
                   f"expected {dim}  -> {verdict(lo, hi, dim)}")
    return "\n".join(out) + "\n"


# --- figures -----------------------------------------------------------------

def style(axis, title, ylabel, denied):
    if denied:
        axis.axvspan(*denied, color=DENIED, alpha=0.35, linewidth=0, label="GNSS denied")
    if axis.get_yscale() == "log":
        axis.yaxis.set_major_locator(LogLocator(subs=(1.0, 2.0, 5.0)))
        axis.yaxis.set_major_formatter(FormatStrFormatter("%g"))
        axis.yaxis.set_minor_formatter(NullFormatter())
    axis.set_title(title, loc="left")
    axis.set_ylabel(ylabel)
    axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
    axis.set_axisbelow(True)
    axis.spines[["top", "right"]].set_visible(False)
    axis.legend(loc="best", frameon=False, fontsize=9)


def pairs_of(summary):
    """The (filter, smoother) keys of each pair, in run order."""
    names = summary["estimators"]
    return [(names[i], names[i + 1]) for i in range(0, len(names), 2)]


def nees_figure(summary):
    M, dt, denied = summary["runs"], summary["dt"], summary["denied"]
    pairs = pairs_of(summary)
    t = np.arange(summary["timesteps"]) * dt
    low, high = chi2_interval(2, M)
    figure, axes = plt.subplots(len(pairs), 1, figsize=(11, 3.6 * len(pairs)),
                                sharex=True, squeeze=False)
    for axis, (f, s) in zip(axes[:, 0], pairs):
        axis.axhspan(low, high, color=BAND, alpha=0.15, linewidth=0,
                     label=f"95% band [{low:.2f}, {high:.2f}]")
        axis.axhline(2, color=BAND, linewidth=1, linestyle="--")
        for key, color in ((f, FILTER_COLOR), (s, SMOOTHER_COLOR)):
            axis.plot(t, summary[key]["nees_mean"], color=color, linewidth=0.7, label=key[1])
        axis.set_yscale("log")
        # a prior with no position error starts the NEES near zero for a few
        # steps, which would otherwise stretch the axis over decades
        values = np.concatenate([summary[f]["nees_mean"], summary[s]["nees_mean"]])
        axis.set_ylim(bottom=min(low / 2, np.nanpercentile(values, 1)))
        style(axis, f"Position NEES averaged over {M} runs, {f[1]} and {s[1]}", "NEES", denied)
    axes[-1, 0].set_xlabel("time [s]")
    return figure


def nis_figure(summary):
    M, dt, denied = summary["runs"], summary["dt"], summary["denied"]
    duration = summary["timesteps"] * dt
    edges = np.arange(0, duration + NIS_BIN_SECONDS, NIS_BIN_SECONDS)
    sensors = [sensor for sensor in SENSORS
               if len(summary["nis"][("ekf_", sensor)]["values"])]
    figure, axes = plt.subplots(len(sensors), 1, figsize=(11, 3.2 * len(sensors)),
                                sharex=True, squeeze=False)
    for axis, sensor in zip(axes[:, 0], sensors):
        dim = SENSORS[sensor][1]
        entry = summary["nis"][("ekf_", sensor)]
        times = entry["steps"] * dt
        which = np.digitize(times, edges) - 1
        counts = np.bincount(which, minlength=len(edges) - 1)[:len(edges) - 1]
        sums = np.bincount(which, weights=entry["values"], minlength=len(edges) - 1)[:len(edges) - 1]
        centers = (edges[:-1] + edges[1:]) / 2
        used = counts > 0
        means = sums[used] / counts[used]
        lows, highs = chi2_interval(dim, counts[used])
        axis.fill_between(centers[used], lows, highs, color=BAND, alpha=0.2, linewidth=0,
                          step="mid", label="95% band")
        axis.axhline(dim, color=BAND, linewidth=1, linestyle="--")
        axis.plot(centers[used], means, color=SENSOR_COLORS[sensor], linewidth=1.2,
                  drawstyle="steps-mid", label=f"{sensor} NIS")
        style(axis, f"{sensor} NIS of the EKF, all {M} runs pooled in "
                    f"{NIS_BIN_SECONDS:g} s bins", "NIS", denied)
    axes[-1, 0].set_xlabel("time [s]")
    return figure


def rmse_figure(summary):
    M, dt, denied = summary["runs"], summary["dt"], summary["denied"]
    pairs = pairs_of(summary)
    t = np.arange(summary["timesteps"]) * dt
    figure, axes = plt.subplots(len(pairs), 1, figsize=(11, 3.6 * len(pairs)),
                                sharex=True, squeeze=False)
    for axis, (f, s) in zip(axes[:, 0], pairs):
        for key, color in ((f, FILTER_COLOR), (s, SMOOTHER_COLOR)):
            axis.plot(t, summary[key]["rmse_t"], color=color, linewidth=0.8, alpha=0.8,
                      label=f"{key[1]} RMSE")
            axis.plot(t, summary[key]["sigma_t"], color=color, linewidth=1.5, linestyle="--",
                      label=rf"{key[1]} $\sqrt{{\mathrm{{tr}}\,P}}$")
        axis.set_yscale("log")
        style(axis, f"Position RMSE over {M} runs against reported uncertainty, "
                    f"{f[1]} and {s[1]}", "position error [m]", denied)
    axes[-1, 0].set_xlabel("time [s]")
    return figure


# --- driver ------------------------------------------------------------------

def find_cases(arguments):
    roots = [Path(a).resolve() for a in arguments] or [INS2D]
    cases = []
    for root in roots:
        for config in sorted(root.rglob("config.yaml")):
            if "monte_carlo" not in config.parts:
                cases.append(config.parent)
    return cases


def write_outputs(summary, case_dir, variant, pdf):
    out_dir = case_dir / "monte_carlo" / variant
    figures_dir = out_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    text = report(summary, case_dir, variant)
    (out_dir / "stats.txt").write_text(text)
    figures = {"mc_nees.png": nees_figure(summary), "mc_nis.png": nis_figure(summary),
               "mc_position_rmse.png": rmse_figure(summary)}
    for name, figure in figures.items():
        figure.tight_layout()
        figure.savefig(figures_dir / name, dpi=150)
        if pdf:
            figure.savefig((figures_dir / name).with_suffix(".pdf"))
        plt.close(figure)
    return text


def save_summary(summary, path):
    np.save(path, summary, allow_pickle=True)


def load_summary(path):
    return np.load(path, allow_pickle=True).item()


def main():
    arguments = sys.argv[1:]
    skip_run = "--skip-run" in arguments
    pdf = "--pdf" in arguments
    jobs = 8
    if "--jobs" in arguments:
        jobs = int(arguments[arguments.index("--jobs") + 1])
        del arguments[arguments.index("--jobs"):arguments.index("--jobs") + 2]
    arguments = [a for a in arguments if not a.startswith("--")]

    cases = find_cases(arguments)
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        for case_dir in cases:
            text = (case_dir / "config.yaml").read_text()
            dt, denied = number(text, "dt"), denied_window(text)
            for variant in VARIANTS:
                summary_path = case_dir / "monte_carlo" / variant / "summary.npy"
                if skip_run:
                    summary = load_summary(summary_path)
                else:
                    runs = list(pool.map(run_one, [(case_dir, variant, s) for s in SEEDS]))
                    summary = summarise(runs, dt, denied)
                    summary_path.parent.mkdir(parents=True, exist_ok=True)
                    save_summary(summary, summary_path)
                print(write_outputs(summary, case_dir, variant, pdf), flush=True)


if __name__ == "__main__":
    main()
