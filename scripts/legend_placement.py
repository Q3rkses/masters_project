"""Places a legend inside an axis where it does not cover any of the plotted curves.

The four corners are tried first with a one-column legend. If every corner
covers a curve, the legend is laid out in up to three columns along the top, and
the y axis is extended just enough for the curves to end below it. Shaded bands
and spans are background and may be covered.
"""

import numpy as np

CORNERS = ("upper right", "upper left", "lower right", "lower left")
MAX_COLUMNS = 3
GAP = 0.03  # share of the axis height left between the curves and the legend
SAMPLES_PER_SEGMENT_PX = 2.0  # curves are sampled about every 2 px in the overlap test


def _curve_points(axis, avoid_reference_lines=False):
    """Every plotted curve as points in display coordinates. Segments longer than
    a couple of pixels are filled in, so a long straight stretch still counts.
    Reference lines such as axhline only count when asked for."""
    points = []
    for line in axis.get_lines():
        if len(line.get_xdata()) < 3 and not avoid_reference_lines:
            continue
        path = line.get_transform().transform_path(line.get_path())
        vertices = path.vertices[np.all(np.isfinite(path.vertices), axis=1)]
        if len(vertices) < 2:
            continue
        points.append(vertices)
        steps = np.linalg.norm(np.diff(vertices, axis=0), axis=1)
        for i in np.flatnonzero(steps > SAMPLES_PER_SEGMENT_PX):
            t = np.linspace(0.0, 1.0, int(steps[i] / SAMPLES_PER_SEGMENT_PX) + 1)[:, None]
            points.append(vertices[i] + t * (vertices[i + 1] - vertices[i]))
    return np.vstack(points) if points else np.zeros((0, 2))


def _covered(points, legend):
    box = legend.get_window_extent().expanded(1.03, 1.08)
    inside = ((points[:, 0] >= box.x0) & (points[:, 0] <= box.x1)
              & (points[:, 1] >= box.y0) & (points[:, 1] <= box.y1))
    return int(inside.sum())


def _headroom_for(axis, legend, points):
    """Raises the top of the y axis so the curves under the legend end below it."""
    box = legend.get_window_extent()
    under = points[(points[:, 0] >= box.x0) & (points[:, 0] <= box.x1)]
    if not len(under):
        return
    frame = axis.get_window_extent()
    room = (box.y0 - frame.y0) / frame.height - GAP  # axis share below the legend
    highest = axis.transData.inverted().transform((0, under[:, 1].max()))[1]
    bottom, top = axis.get_ylim()
    if axis.get_yscale() == "log":
        low, high = np.log10(bottom), np.log10(max(highest, bottom))
        top = max(top, 10 ** (low + (high - low) / room))
    else:
        top = max(top, bottom + (highest - bottom) / room)
    axis.set_ylim(bottom, top)


def place_legend(axis, avoid_reference_lines=False, **legend_options):
    """Draws the axis legend in a spot that leaves every curve visible."""
    figure = axis.figure
    renderer = figure.canvas.get_renderer()
    figure.canvas.draw()
    points = _curve_points(axis, avoid_reference_lines)
    for corner in CORNERS:
        legend = axis.legend(loc=corner, **legend_options)
        legend.draw(renderer)  # lays the legend out, so its extent is known
        if _covered(points, legend) == 0:
            return legend
    entries = len(axis.get_legend_handles_labels()[0])
    # two even rows, which keeps each estimator's pair of entries in one column
    legend = axis.legend(loc="upper right", ncol=min(-(-entries // 2), MAX_COLUMNS),
                         **legend_options)
    legend.draw(renderer)
    _headroom_for(axis, legend, points)
    return legend
