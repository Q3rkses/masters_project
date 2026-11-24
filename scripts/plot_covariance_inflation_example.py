#!/usr/bin/env python3
"""The x position error of the IEKF and the IEKS through a GNSS outage, with
their 95% bands. The filter's band grows through the whole outage, while the
smoother's band grows towards the middle and shrinks again as the fixes after
the outage reach back in time.

Reads the stored run in results/INS2D/ieks/gnss_outage/data.

usage: plot_covariance_inflation_example.py [output.pdf]
"""

import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm

ROOT = Path(__file__).resolve().parent.parent
CASE = ROOT / "results" / "INS2D" / "ieks" / "gnss_outage"
DEFAULT_OUTPUT = ROOT.parent / "docs/possible_results/covariance_inflation_example.pdf"

FILTER = "#7f5539"
SMOOTHER = "#c2477f"
DENIED = "#f2c48d"
BAND = "#8a8984"
LEGEND_SIZE = 13  # about 9 pt once the 10 inch figure is scaled to a report's text width


def read_columns(path, names):
    with open(path) as handle:
        header = handle.readline().strip().split(",")
    data = np.loadtxt(path, delimiter=",", skiprows=1, usecols=[header.index(n) for n in names], ndmin=2)
    return {name: data[:, i] for i, name in enumerate(names)}


def main():
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_OUTPUT
    config = (CASE / "config.yaml").read_text()
    number = lambda key: float(re.search(rf"^{key}:\s*([-+0-9.eE]+)", config, re.M).group(1))
    dt = number("dt")
    denied = (number("gnss_denied_start"), number("gnss_denied_end"))

    data = CASE / "data"
    x_true = read_columns(data / "truth.csv", ["x_true_0"])["x_true_0"]
    t = np.arange(len(x_true)) * dt
    z = norm.ppf(0.975)

    figure, axes = plt.subplots(1, 2, figsize=(10, 5), sharey=True)
    for axis, file, kind, title, color in (
            (axes[0], "ieks_filter.csv", "updated", "Filter (IEKF)", FILTER),
            (axes[1], "ieks_smoother.csv", "smoothed", "Smoother (IEKS)", SMOOTHER)):
        columns = read_columns(data / file, [f"x_{kind}_0", f"P_{kind}_0_0"])
        error = columns[f"x_{kind}_0"] - x_true
        sigma = z * np.sqrt(columns[f"P_{kind}_0_0"])
        axis.axvspan(*denied, color=DENIED, alpha=0.35, linewidth=0, label="GNSS denied")
        axis.fill_between(t, -sigma, sigma, color=color, alpha=0.2, linewidth=0, label="95% band")
        axis.plot(t, error, color=color, linewidth=0.9, label="error")
        axis.axhline(0, color=BAND, linewidth=0.8, linestyle=":")
        axis.set_title(title, loc="left")
        axis.set_xlabel("time [s]")
        axis.grid(color=BAND, alpha=0.2, linewidth=0.5)
        axis.set_axisbelow(True)
        axis.spines[["top", "right"]].set_visible(False)
    axes[0].set_ylabel("x error [m]")
    # one legend for both panels, below them, since the panels share their entries
    handles, labels = axes[0].get_legend_handles_labels()
    order = [labels.index(name) for name in ("error", "95% band", "GNSS denied")]
    figure.legend([handles[i] for i in order], ["error", "95% band", "GNSS denied"],
                  loc="lower center", ncol=3, frameon=False, fontsize=LEGEND_SIZE,
                  bbox_to_anchor=(0.5, -0.02), handlelength=2.5)
    figure.tight_layout(rect=(0, 0.08, 1, 1))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output)
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
