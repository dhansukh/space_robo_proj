"""
visualizer.py
=============

Standalone matplotlib-based 3-D visualiser for debugging cave generation.

Functions
---------
plot_graph(graph)
    3-D scatter + line plot of cave nodes and edges.

plot_splines(graph, splines)
    Overlay Catmull-Rom spline paths on the graph.

plot_sdf_cross_section(sdf_volume, origin, resolution, axis, level)
    Heatmap of a 2-D slice through the SDF volume.

plot_all(graph, splines, sdf_volume, origin, resolution)
    Composite figure combining graph, splines, and SDF cross-section.

All functions return the ``matplotlib.figure.Figure`` object so callers
can save or display them.
"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")  # non-interactive backend (safe in headless ROS env)
    import matplotlib.pyplot as plt
    from matplotlib.axes import Axes
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — needed for 3D projection
    _MPL_AVAILABLE = True
except ImportError:  # pragma: no cover
    _MPL_AVAILABLE = False

from cave_generator.cave_graph import CaveGraph

logger = logging.getLogger(__name__)


def _require_matplotlib() -> None:
    if not _MPL_AVAILABLE:
        raise ImportError(
            "matplotlib is required for the visualiser. "
            "Install it with: pip install matplotlib"
        )


# ---------------------------------------------------------------------------
# Public visualisation functions
# ---------------------------------------------------------------------------

def plot_graph(
    graph: CaveGraph,
    title: str = "Cave Graph Topology",
    figsize: Tuple[float, float] = (10, 8),
) -> "plt.Figure":
    """
    Render the cave graph as a 3-D scatter (nodes) + line (edges) plot.

    Node colours:
    - Cyan   : normal junction
    - Red    : dead-end node
    - Green  : entrance node (lowest z)

    Parameters
    ----------
    graph : CaveGraph
    title : str
    figsize : tuple[float, float]

    Returns
    -------
    matplotlib.figure.Figure
    """
    _require_matplotlib()
    G = graph.graph

    fig = plt.figure(figsize=figsize)
    ax: Axes3D = fig.add_subplot(111, projection="3d")

    entrance = graph.get_entrance_node()

    # Draw edges
    for u, v in G.edges:
        pu = G.nodes[u]["position"]
        pv = G.nodes[v]["position"]
        edge_data = G.edges[u, v]
        lw = 0.5 + edge_data["width"] * 0.3
        ls = "--" if edge_data["has_squeeze"] else "-"
        ax.plot(
            [pu[0], pv[0]],
            [pu[1], pv[1]],
            [pu[2], pv[2]],
            color="orange",
            linewidth=lw,
            linestyle=ls,
            alpha=0.7,
        )

    # Draw nodes
    for n in G.nodes:
        props = G.nodes[n]
        pos = props["position"]
        if n == entrance:
            colour = "lime"
            marker = "*"
            size = 200
        elif props["is_dead_end"]:
            colour = "red"
            marker = "D"
            size = 80
        else:
            colour = "deepskyblue"
            marker = "o"
            size = max(20, props["radius"] * 10)

        ax.scatter(*pos, c=colour, marker=marker, s=size, zorder=5)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(title)

    # Legend proxy
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker="*", color="w", markerfacecolor="lime", markersize=12, label="Entrance"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="deepskyblue", markersize=8, label="Junction"),
        Line2D([0], [0], marker="D", color="w", markerfacecolor="red", markersize=8, label="Dead end"),
        Line2D([0], [0], color="orange", linewidth=2, label="Passage"),
        Line2D([0], [0], color="orange", linewidth=2, linestyle="--", label="Squeeze passage"),
    ]
    ax.legend(handles=legend_elements, loc="upper left", fontsize=8)

    fig.tight_layout()
    return fig


def plot_splines(
    graph: CaveGraph,
    splines: List[np.ndarray],
    title: str = "Cave Spline Paths",
    figsize: Tuple[float, float] = (10, 8),
) -> "plt.Figure":
    """
    Plot Catmull-Rom spline paths overlaid on the cave graph.

    Parameters
    ----------
    graph : CaveGraph
    splines : list of ndarray shape (N, 3)
        One spline per graph edge (in same order as ``graph.graph.edges``).
    title : str
    figsize : tuple[float, float]

    Returns
    -------
    matplotlib.figure.Figure
    """
    _require_matplotlib()
    G = graph.graph

    fig = plt.figure(figsize=figsize)
    ax: Axes3D = fig.add_subplot(111, projection="3d")

    # Draw node positions as small spheres
    for n in G.nodes:
        pos = G.nodes[n]["position"]
        ax.scatter(*pos, c="deepskyblue", s=40, zorder=5)

    # Draw splines with gradient colouring per edge
    cmap = plt.get_cmap("plasma")
    n_edges = len(splines)
    for i, spline in enumerate(splines):
        colour = cmap(i / max(n_edges - 1, 1))
        ax.plot(
            spline[:, 0], spline[:, 1], spline[:, 2],
            color=colour, linewidth=1.2, alpha=0.85,
        )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(title)
    fig.tight_layout()
    return fig


def plot_sdf_cross_section(
    sdf_volume: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    axis: int = 2,
    level: float = 0.0,
    slice_index: Optional[int] = None,
    title: str = "SDF Cross-Section",
    figsize: Tuple[float, float] = (8, 7),
) -> "plt.Figure":
    """
    Render a 2-D heatmap of one slice through the SDF volume.

    The iso-surface (level=0) is overlaid as a contour line.

    Parameters
    ----------
    sdf_volume : ndarray shape (Nx, Ny, Nz)
        The signed distance field.
    origin : ndarray shape (3,)
        World-space position of the grid's [0,0,0] corner.
    resolution : float
        Voxel size in metres.
    axis : int
        Which axis to slice along (0=X, 1=Y, 2=Z).  Default 2 (Z-slice).
    level : float
        Iso-surface level to highlight.  Default 0.
    slice_index : int, optional
        Voxel index along *axis* to take.  Defaults to the midpoint.
    title : str
    figsize : tuple[float, float]

    Returns
    -------
    matplotlib.figure.Figure
    """
    _require_matplotlib()

    if slice_index is None:
        slice_index = sdf_volume.shape[axis] // 2

    # Extract 2-D slice
    if axis == 0:
        sdf_slice = sdf_volume[slice_index, :, :]
        ax_labels = ("Y (m)", "Z (m)")
        coords_0 = origin[1] + np.arange(sdf_slice.shape[0]) * resolution
        coords_1 = origin[2] + np.arange(sdf_slice.shape[1]) * resolution
        world_pos = origin[0] + slice_index * resolution
    elif axis == 1:
        sdf_slice = sdf_volume[:, slice_index, :]
        ax_labels = ("X (m)", "Z (m)")
        coords_0 = origin[0] + np.arange(sdf_slice.shape[0]) * resolution
        coords_1 = origin[2] + np.arange(sdf_slice.shape[1]) * resolution
        world_pos = origin[1] + slice_index * resolution
    else:
        sdf_slice = sdf_volume[:, :, slice_index]
        ax_labels = ("X (m)", "Y (m)")
        coords_0 = origin[0] + np.arange(sdf_slice.shape[0]) * resolution
        coords_1 = origin[1] + np.arange(sdf_slice.shape[1]) * resolution
        world_pos = origin[2] + slice_index * resolution

    fig, ax = plt.subplots(figsize=figsize)

    # Clamp display range for readability
    vmin = max(sdf_slice.min(), -10.0)
    vmax = min(sdf_slice.max(), 10.0)
    extent = [coords_1[0], coords_1[-1], coords_0[0], coords_0[-1]]

    im = ax.imshow(
        sdf_slice,
        extent=extent,
        origin="lower",
        aspect="equal",
        cmap="RdBu_r",
        vmin=vmin,
        vmax=vmax,
    )
    plt.colorbar(im, ax=ax, label="SDF value (m)")

    # Overlay iso-contour
    ax.contour(
        coords_1, coords_0, sdf_slice,
        levels=[level],
        colors="black",
        linewidths=1.5,
    )

    ax.set_xlabel(ax_labels[1])
    ax.set_ylabel(ax_labels[0])
    ax_names = ["X", "Y", "Z"]
    ax.set_title(f"{title}  (slice {ax_names[axis]}={world_pos:.1f} m)")
    fig.tight_layout()
    return fig


def plot_all(
    graph: CaveGraph,
    splines: List[np.ndarray],
    sdf_volume: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    save_path: Optional[str] = None,
    figsize_large: Tuple[float, float] = (18, 6),
) -> "plt.Figure":
    """
    Composite figure: graph (left), splines (centre), SDF cross-section (right).

    Parameters
    ----------
    graph : CaveGraph
    splines : list of ndarray
    sdf_volume : ndarray
    origin : ndarray shape (3,)
    resolution : float
    save_path : str, optional
        If given, save the figure to this path (PNG/PDF).
    figsize_large : tuple[float, float]
        Size of the composite figure.

    Returns
    -------
    matplotlib.figure.Figure
    """
    _require_matplotlib()

    fig = plt.figure(figsize=figsize_large)

    # Subplot 1: graph
    ax1: Axes3D = fig.add_subplot(131, projection="3d")
    _draw_graph_on_ax(graph, ax1)
    ax1.set_title("Graph Topology")

    # Subplot 2: splines
    ax2: Axes3D = fig.add_subplot(132, projection="3d")
    _draw_splines_on_ax(graph, splines, ax2)
    ax2.set_title("Spline Paths")

    # Subplot 3: SDF slice
    ax3: Axes = fig.add_subplot(133)
    _draw_sdf_on_ax(sdf_volume, origin, resolution, ax3)
    ax3.set_title("SDF Z-Slice (mid)")

    fig.suptitle("Cave Generator Debug View", fontsize=14)
    fig.tight_layout()

    if save_path is not None:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        logger.info("Saved composite visualisation to: %s", save_path)

    return fig


# ---------------------------------------------------------------------------
# Private helpers (draw onto existing axes)
# ---------------------------------------------------------------------------

def _draw_graph_on_ax(graph: CaveGraph, ax: "Axes3D") -> None:
    G = graph.graph
    entrance = graph.get_entrance_node()
    for u, v in G.edges:
        pu, pv = G.nodes[u]["position"], G.nodes[v]["position"]
        ax.plot([pu[0], pv[0]], [pu[1], pv[1]], [pu[2], pv[2]],
                color="orange", linewidth=0.8, alpha=0.7)
    for n in G.nodes:
        pos = G.nodes[n]["position"]
        c = "lime" if n == entrance else ("red" if G.nodes[n]["is_dead_end"] else "deepskyblue")
        ax.scatter(*pos, c=c, s=30, zorder=5)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")


def _draw_splines_on_ax(
    graph: CaveGraph, splines: List[np.ndarray], ax: "Axes3D"
) -> None:
    G = graph.graph
    for n in G.nodes:
        pos = G.nodes[n]["position"]
        ax.scatter(*pos, c="deepskyblue", s=20, zorder=5)
    cmap = plt.get_cmap("plasma")
    n_edges = max(len(splines) - 1, 1)
    for i, sp in enumerate(splines):
        ax.plot(sp[:, 0], sp[:, 1], sp[:, 2],
                color=cmap(i / n_edges), linewidth=1.0, alpha=0.8)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")


def _draw_sdf_on_ax(
    sdf_volume: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    ax: "Axes",
) -> None:
    sl_idx = sdf_volume.shape[2] // 2
    sdf_slice = sdf_volume[:, :, sl_idx]
    coords_x = origin[0] + np.arange(sdf_slice.shape[0]) * resolution
    coords_y = origin[1] + np.arange(sdf_slice.shape[1]) * resolution
    extent = [coords_y[0], coords_y[-1], coords_x[0], coords_x[-1]]
    vmin = max(sdf_slice.min(), -10.0)
    vmax = min(sdf_slice.max(), 10.0)
    ax.imshow(sdf_slice, extent=extent, origin="lower", aspect="equal",
              cmap="RdBu_r", vmin=vmin, vmax=vmax)
    ax.contour(coords_y, coords_x, sdf_slice, levels=[0.0], colors="black", linewidths=1.2)
    ax.set_xlabel("Y (m)"); ax.set_ylabel("X (m)")
