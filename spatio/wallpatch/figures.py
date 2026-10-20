"""Cross-section figures. Measurement never reads these pixels back.

matplotlib is forced onto the Agg backend so a headless run can write PNGs.
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")

import textwrap

import matplotlib.pyplot as plt
import numpy as np

# A scatter of a whole wall can be tens of thousands of points. The PNG
# shows a stride of those. The residual numbers are computed earlier, on
# every cropped point.
_MAX_SCATTER = 8000


def write_patch_figure(
    path,
    title,
    status,
    hist_counts,
    hist_edges,
    marks,
    across,
    vertical,
    slice_mask,
):
    """Write one PNG: histogram of signed distance, and a thin side view."""
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 4.4), constrained_layout=True)
    _draw_histogram(axes[0], hist_counts, hist_edges, marks)
    note = _draw_slice(axes[1], across, vertical, slice_mask)
    fig.suptitle("%s — %s" % (title, status), fontsize=12)
    if note:
        fig.text(0.01, 0.01, note, fontsize=8, color="#444444")
    fig.savefig(path, dpi=120)
    plt.close(fig)


def write_note_figure(path, title, status, reason):
    """A figure for a patch that was not scored, so every patch still has a PNG."""
    fig, ax = plt.subplots(figsize=(6.4, 3.2), constrained_layout=True)
    ax.set_axis_off()
    ax.set_title("%s — %s" % (title, status))
    # A long reason is one line. Unwrapped, it collapses the layout and
    # matplotlib warns instead of drawing the note.
    note = "\n".join(textwrap.wrap(reason, width=42)) or reason
    ax.text(0.5, 0.5, note, ha="center", va="center", fontsize=11)
    fig.savefig(path, dpi=120)
    plt.close(fig)


def _draw_histogram(ax, counts, edges, marks):
    ax.set_title("signed distance")
    ax.set_xlabel("signed distance (m)")
    ax.set_ylabel("count")
    if counts is None or edges is None or len(counts) == 0:
        ax.text(0.5, 0.5, "no points", ha="center", va="center", transform=ax.transAxes)
        return
    centers = 0.5 * (edges[:-1] + edges[1:])
    width = float(edges[1] - edges[0]) if len(edges) > 1 else 0.002
    ax.bar(
        centers,
        counts,
        width=width,
        align="center",
        color="#3d5a80",
        linewidth=0,
    )
    labelled = False
    for mark in marks or []:
        ax.axvline(
            float(mark),
            color="#c44900",
            linewidth=1.3,
            label="surface" if not labelled else None,
        )
        labelled = True
    if labelled:
        ax.legend(frameon=False)
    ax.grid(axis="y", color="#dddddd", linewidth=0.6)


def _draw_slice(ax, across, vertical, slice_mask):
    ax.set_title("cross-section")
    ax.set_xlabel("across (m)")
    ax.set_ylabel("vertical (m)")
    if across is None or slice_mask is None or not np.any(slice_mask):
        ax.text(0.5, 0.5, "no slice", ha="center", va="center", transform=ax.transAxes)
        return None
    idx = np.flatnonzero(slice_mask)
    note = None
    if idx.size > _MAX_SCATTER:
        stride = int(np.ceil(idx.size / float(_MAX_SCATTER)))
        shown = idx[::stride]
        note = (
            "display plots %d of %d slice points; the measurement uses every cropped point"
            % (int(shown.size), int(idx.size))
        )
        idx = shown
    ax.scatter(
        across[idx],
        vertical[idx],
        s=4,
        c="#1b1b1b",
        alpha=0.45,
        linewidths=0,
    )
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(color="#eeeeee", linewidth=0.6)
    return note
