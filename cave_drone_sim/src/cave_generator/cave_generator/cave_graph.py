"""
cave_graph.py
=============

CaveGraph: generates a randomized 3-D cave topology using a minimum spanning tree
(Prim's algorithm with random weights) plus extra loop edges for SLAM loop-closure
testing.

Node properties
---------------
- position : tuple[float, float, float]   (x, y, z) in metres
- radius    : float                        chamber size in [2, 6] m
- is_dead_end : bool                       True for leaf nodes (~15 %)

Edge properties
---------------
- width      : float   passage radius in [1, 3] m (>= 0.6 m so a 0.5 m drone fits)
- has_squeeze : bool   True ~20 % of edges (radius shrinks to 0.8 m at midpoint)
"""

from __future__ import annotations

import json
import math
import random
from typing import Any, Dict, List, Optional, Tuple

import networkx as nx
import numpy as np


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MIN_TUNNEL_RADIUS: float = 2.5   # minimum radius (5 m diameter) so drone always fits
SQUEEZE_RADIUS: float = 2.5      # radius at a squeeze point (5 m diameter minimum)
DEAD_END_FRACTION: float = 0.15  # fraction of leaf nodes marked as dead ends
SQUEEZE_PROBABILITY: float = 0.20
MIN_CHAMBER_RADIUS: float = 4.0  # larger chambers for wide passages
MAX_CHAMBER_RADIUS: float = 8.0
MIN_PASSAGE_WIDTH: float = 3.0   # passage radius 3-5 m (6-10 m diameter)
MAX_PASSAGE_WIDTH: float = 5.0


class CaveGraph:
    """
    Procedural cave topology represented as a NetworkX undirected graph.

    The cave skeleton is built in three steps:

    1. Place *num_nodes* random 3-D nodes inside *bounds_3d*.
    2. Build a minimum spanning tree via a randomized Prim's algorithm
       (random edge weights guarantee variety while keeping full connectivity).
    3. Optionally add *num_extra_loops* random edges so the cave contains
       cycles — essential for SLAM loop-closure testing.

    Parameters
    ----------
    seed : int, optional
        Random seed for deterministic generation.  Default ``None`` (random).
    """

    def __init__(self, seed: Optional[int] = None) -> None:
        self._seed: Optional[int] = seed
        self._rng: random.Random = random.Random(seed)
        self._np_rng: np.random.Generator = np.random.default_rng(seed)
        self.graph: nx.Graph = nx.Graph()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def generate_topology(
        self,
        num_nodes: int = 20,
        bounds_3d: Tuple[float, float, float] = (80.0, 80.0, 25.0),
        min_edge_length: float = 8.0,
        max_edge_length: float = 30.0,
        num_extra_loops: int = 3,
    ) -> None:
        """
        Generate a cave graph topology.

        Parameters
        ----------
        num_nodes : int
            Number of cave nodes (chambers / junctions).
        bounds_3d : tuple[float, float, float]
            Bounding box (width_x, width_y, height_z) in metres.
        min_edge_length : float
            Minimum Euclidean distance between connected nodes (metres).
        max_edge_length : float
            Maximum Euclidean distance between connected nodes (metres).
        num_extra_loops : int
            Extra edges added on top of the MST to create cycles.
        """
        self.graph.clear()
        self._bounds = bounds_3d

        # 1. Place nodes
        positions = self._place_nodes(num_nodes, bounds_3d)

        for idx, pos in enumerate(positions):
            radius = self._rng.uniform(MIN_CHAMBER_RADIUS, MAX_CHAMBER_RADIUS)
            self.graph.add_node(
                idx,
                position=tuple(pos),
                radius=float(radius),
                is_dead_end=False,
            )

        # 2. Build MST with randomized Prim's
        self._build_prim_mst(min_edge_length, max_edge_length)

        # 3. Add loop edges
        self._add_loop_edges(num_extra_loops, min_edge_length, max_edge_length)

        # 4. Mark dead ends
        self._mark_dead_ends()

    def to_json(self, path: Optional[str] = None) -> str:
        """
        Serialise the graph to a JSON string (and optionally write to *path*).

        Returns
        -------
        str
            JSON representation of nodes and edges.
        """
        data: Dict[str, Any] = {
            "nodes": [
                {
                    "id": n,
                    "position": list(self.graph.nodes[n]["position"]),
                    "radius": self.graph.nodes[n]["radius"],
                    "is_dead_end": self.graph.nodes[n]["is_dead_end"],
                }
                for n in self.graph.nodes
            ],
            "edges": [
                {
                    "source": u,
                    "target": v,
                    "width": self.graph.edges[u, v]["width"],
                    "has_squeeze": self.graph.edges[u, v]["has_squeeze"],
                    "length": self.graph.edges[u, v]["length"],
                }
                for u, v in self.graph.edges
            ],
        }
        json_str = json.dumps(data, indent=2)
        if path is not None:
            with open(path, "w") as fh:
                fh.write(json_str)
        return json_str

    @classmethod
    def from_json(cls, path: str) -> "CaveGraph":
        """
        Reconstruct a CaveGraph from a previously exported JSON file.

        Parameters
        ----------
        path : str
            Path to the JSON file.
        """
        instance = cls()
        with open(path, "r") as fh:
            data = json.load(fh)
        for node in data["nodes"]:
            instance.graph.add_node(
                node["id"],
                position=tuple(node["position"]),
                radius=node["radius"],
                is_dead_end=node["is_dead_end"],
            )
        for edge in data["edges"]:
            instance.graph.add_edge(
                edge["source"],
                edge["target"],
                width=edge["width"],
                has_squeeze=edge["has_squeeze"],
                length=edge["length"],
            )
        return instance

    def get_entrance_node(self) -> int:
        """Return the node with the lowest z-coordinate (cave entrance)."""
        return min(
            self.graph.nodes,
            key=lambda n: self.graph.nodes[n]["position"][2],
        )

    def get_dead_end_nodes(self) -> List[int]:
        """Return a list of node IDs marked as dead ends."""
        return [
            n for n in self.graph.nodes if self.graph.nodes[n]["is_dead_end"]
        ]

    def summary(self) -> str:
        """Return a human-readable summary of the graph statistics."""
        dead_ends = len(self.get_dead_end_nodes())
        squeezes = sum(
            1 for u, v in self.graph.edges if self.graph.edges[u, v]["has_squeeze"]
        )
        return (
            f"CaveGraph: {self.graph.number_of_nodes()} nodes, "
            f"{self.graph.number_of_edges()} edges, "
            f"{dead_ends} dead ends, "
            f"{squeezes} squeeze passages"
        )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _place_nodes(
        self,
        num_nodes: int,
        bounds: Tuple[float, float, float],
    ) -> np.ndarray:
        """
        Place *num_nodes* points in the 3-D bounding box using Poisson-disk-like
        minimum separation to avoid degenerate geometry.
        """
        bx, by, bz = bounds
        # Centre the cave at origin
        half_x, half_y, half_z = bx / 2.0, by / 2.0, bz / 2.0

        min_sep = 6.0  # metres — chambers shouldn't overlap
        positions: List[np.ndarray] = []
        max_attempts = num_nodes * 200

        for _ in range(max_attempts):
            if len(positions) >= num_nodes:
                break
            candidate = np.array([
                self._rng.uniform(-half_x, half_x),
                self._rng.uniform(-half_y, half_y),
                0.0,  # Planar cave: all nodes at z=0
            ])
            if all(
                np.linalg.norm(candidate - p) >= min_sep for p in positions
            ):
                positions.append(candidate)

        # If we couldn't place all nodes with strict separation, relax and fill
        while len(positions) < num_nodes:
            candidate = np.array([
                self._rng.uniform(-half_x, half_x),
                self._rng.uniform(-half_y, half_y),
                0.0,  # Planar cave: all nodes at z=0
            ])
            positions.append(candidate)

        return np.array(positions)

    def _euclidean(self, u: int, v: int) -> float:
        """Euclidean distance between two node positions."""
        pu = np.array(self.graph.nodes[u]["position"])
        pv = np.array(self.graph.nodes[v]["position"])
        return float(np.linalg.norm(pu - pv))

    def _build_prim_mst(
        self,
        min_edge_length: float,
        max_edge_length: float,
    ) -> None:
        """
        Build a spanning tree using randomised Prim's algorithm.

        Edge weights are a combination of Euclidean distance and a random
        perturbation so the resulting tree looks organic, not grid-like.

        Edges whose geometric length falls outside [min_edge_length,
        max_edge_length] are still added if they are the only way to connect
        a node (guaranteeing full connectivity).
        """
        nodes = list(self.graph.nodes)
        if not nodes:
            return

        in_tree = {nodes[0]}
        candidates: List[Tuple[float, int, int]] = []

        # Seed the candidate list from the start node
        for v in nodes[1:]:
            dist = self._euclidean(nodes[0], v)
            jitter = self._rng.uniform(0.0, dist * 0.4)
            candidates.append((dist + jitter, nodes[0], v))

        # Simple O(n²) Prim's — sufficient for n <= 100
        while len(in_tree) < len(nodes):
            # Sort by weight
            candidates.sort(key=lambda x: x[0])

            # Pick the cheapest edge that connects a new node
            chosen = None
            for idx, (w, u, v) in enumerate(candidates):
                if v not in in_tree:
                    chosen = (w, u, v, idx)
                    break

            if chosen is None:
                # Fallback: connect remaining nodes by nearest in-tree node
                remaining = [n for n in nodes if n not in in_tree]
                for r in remaining:
                    nearest = min(
                        in_tree,
                        key=lambda t: self._euclidean(t, r),
                    )
                    self._add_edge(nearest, r)
                    in_tree.add(r)
                break

            w, u, v, idx = chosen
            candidates.pop(idx)
            self._add_edge(u, v)
            in_tree.add(v)

            # Expand candidates from newly added node
            for other in nodes:
                if other not in in_tree:
                    dist = self._euclidean(v, other)
                    jitter = self._rng.uniform(0.0, dist * 0.4)
                    candidates.append((dist + jitter, v, other))

    def _add_loop_edges(
        self,
        num_extra: int,
        min_edge_length: float,
        max_edge_length: float,
    ) -> None:
        """
        Add random extra edges to create cycles in the graph.

        Candidate pairs are chosen from nodes that are geometrically close
        (within max_edge_length) but not already connected.
        """
        nodes = list(self.graph.nodes)
        # Collect all non-existing pairs within range
        candidates: List[Tuple[int, int]] = []
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                u, v = nodes[i], nodes[j]
                if not self.graph.has_edge(u, v):
                    dist = self._euclidean(u, v)
                    if min_edge_length <= dist <= max_edge_length:
                        candidates.append((u, v))

        self._rng.shuffle(candidates)
        added = 0
        for u, v in candidates:
            if added >= num_extra:
                break
            self._add_edge(u, v)
            added += 1

    def _add_edge(self, u: int, v: int) -> None:
        """
        Add an edge between *u* and *v* with randomised passage properties.
        """
        dist = self._euclidean(u, v)
        width = self._rng.uniform(MIN_PASSAGE_WIDTH, MAX_PASSAGE_WIDTH)
        width = max(width, MIN_TUNNEL_RADIUS)
        has_squeeze = self._rng.random() < SQUEEZE_PROBABILITY
        self.graph.add_edge(
            u,
            v,
            width=float(width),
            has_squeeze=bool(has_squeeze),
            length=float(dist),
            squeeze_radius=SQUEEZE_RADIUS if has_squeeze else None,
        )

    def _mark_dead_ends(self) -> None:
        """
        Mark approximately DEAD_END_FRACTION of leaf nodes (degree == 1)
        as dead ends.
        """
        leaf_nodes = [n for n in self.graph.nodes if self.graph.degree(n) == 1]
        num_to_mark = max(1, int(len(leaf_nodes) * DEAD_END_FRACTION / (DEAD_END_FRACTION + (1 - DEAD_END_FRACTION))))
        # Actually use the proper fraction
        num_to_mark = max(0, round(len(leaf_nodes) * DEAD_END_FRACTION))
        chosen = self._rng.sample(leaf_nodes, min(num_to_mark, len(leaf_nodes)))
        for n in chosen:
            self.graph.nodes[n]["is_dead_end"] = True
