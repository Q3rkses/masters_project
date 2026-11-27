"""Ground truth of the two 2D paths, for the method chapter.

Reads truth.csv of the 2D multi-sensor fusion runs and draws, for the
four-corner test and the figure-eight side by side, the true path (straights
and arcs in different colours) and the true turn rate over one lap. The
centripetal acceleration is the turn rate times the speed, a = V omega, so
it is left to the caption.

Usage: python3 scripts/plot_ground_truth_paths.py [output.pdf]
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[1]
RUNS = {
    "Four-corner test": ROOT / "results/INS2D/multi_sensor_fusion/rounded_rectangle",
    "Figure-eight": ROOT / "results/INS2D/multi_sensor_fusion/figure_eight",
}
DEFAULT_OUTPUT = ROOT.parent / "docs/possible_results/ins2d_ground_truth_paths.pdf"

STRAIGHT = "#3d3d3a"
ARC = "#2a78d6"
GRID = "#8a8984"
SPEED = 2.0  # path_speed in the configs [m/s]
DT = 0.01
TURNING = 1e-3  # turn rate above this counts as an arc [rad/s]


def load_truth(case_dir):
    table = np.genfromtxt(case_dir / "data/truth.csv", delimiter=",", names=True)
    position = np.column_stack([table["x_true_0"], table["x_true_1"]])
    heading = np.unwrap(table["x_true_4"])
    return position, heading


def lap_length(position, heading):
    """Steps until the vehicle is first back at the start with the start
    heading. The heading check matters on the figure-eight, which passes
    through its start point twice per lap."""
    distance = np.linalg.norm(position - position[0], axis=1)
    same_heading = np.cos(heading - heading[0]) > 0.99
    return int(np.flatnonzero((distance < 0.03) & same_heading & (np.arange(len(position)) > 100))[0])


def draw_path(axis, position, heading, turn_rate):
    lap = lap_length(position, heading)
    # drawn as a map, x (north) up and y (east) to the right, so a positive
    # turn rate is a turn to starboard and looks clockwise
    p, arc = position[:lap + 1, ::-1], np.abs(turn_rate[:lap + 1]) > TURNING
    # one line per run of straight or arc steps, so the colours switch cleanly
    edges = np.flatnonzero(np.diff(arc.astype(int))) + 1
    for lo, hi in zip(np.r_[0, edges], np.r_[edges, len(p) - 1]):
        axis.plot(*p[lo:hi + 1].T, color=ARC if arc[lo] else STRAIGHT,
                  linewidth=2.0, solid_capstyle="butt")
    # one arrow half way along each of the two longest segments, enough to
    # show the direction of travel
    runs = sorted(zip(np.r_[0, edges], np.r_[edges, len(p) - 1]), key=lambda r: r[0] - r[1])
    for lo, hi in runs[:2]:
        k = (lo + hi) // 2
        axis.annotate("", xy=p[k + 25], xytext=p[k - 25],
                      arrowprops=dict(arrowstyle="-|>", color=STRAIGHT, lw=1.0,
                                      mutation_scale=14))
    axis.scatter(*p[0], s=30, color=STRAIGHT, zorder=5)
    # equal aspect inside a box of the same size for both paths, so both fill
    # the full height and come out equally big
    axis.set_aspect("equal", adjustable="datalim")
    span = np.ptp(p, axis=0)
    axis.set_xlim(p[:, 0].min() - 0.06 * span[0], p[:, 0].max() + 0.06 * span[0])
    axis.set_ylim(p[:, 1].min() - 0.06 * span[1], p[:, 1].max() + 0.06 * span[1])
    axis.set_xlabel("y, east [m]")
    axis.set_ylabel("x, north [m]")
    return lap


def draw_turn_rate(axis, turn_rate, lap):
    t = np.arange(lap + 1) * DT
    axis.plot(t, turn_rate[:lap + 1], color=ARC, linewidth=1.6)
    axis.axhline(0, color=GRID, linewidth=0.6)
    axis.set_xlim(0, t[-1])
    peak = np.abs(turn_rate).max()
    axis.set_ylim(-1.3 * peak, 1.3 * peak)
    axis.set_yticks([-peak, 0, peak])
    axis.set_xlabel("time [s]")
    axis.set_ylabel("turn rate [rad/s]")


def main():
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_OUTPUT
    # drawn at the printed size, about 15 cm wide, so the text keeps its size
    plt.rcParams.update({"font.size": 9, "axes.titlesize": 10,
                         "axes.spines.top": False, "axes.spines.right": False})
    figure, axes = plt.subplots(2, 2, figsize=(5.9, 4.4),
                                gridspec_kw=dict(height_ratios=[2.8, 1], hspace=0.45, wspace=0.4))
    for column, (title, case_dir) in enumerate(RUNS.items()):
        position, heading = load_truth(case_dir)
        turn_rate = np.gradient(heading, DT)
        lap = draw_path(axes[0, column], position, heading, turn_rate)
        draw_turn_rate(axes[1, column], turn_rate, lap)
        axes[0, column].set_title(title)
        print(f"{title}: lap {lap * DT:.1f} s, {lap * DT * SPEED:.1f} m, "
              f"peak turn rate {np.abs(turn_rate).max():.3f} rad/s")
    for axis in axes.flat:
        axis.grid(color=GRID, alpha=0.2, linewidth=0.6)
    figure.legend(handles=[Line2D([], [], color=STRAIGHT, lw=2, label="straight"),
                           Line2D([], [], color=ARC, lw=2, label="arc")],
                  loc="lower center", frameon=False, fontsize=10, ncol=2,
                  bbox_to_anchor=(0.5, -0.09))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, bbox_inches="tight")
    figure.savefig(output.with_suffix(".png"), bbox_inches="tight", dpi=200)
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
