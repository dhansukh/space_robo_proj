"""
obstacle_placer.py
==================

ObstaclePlacer: generates and places cave obstacles (stalactites, rubble)
as separate trimesh objects.  Each obstacle can be exported as an STL file
for inclusion as a distinct Gazebo model.

Stalactites
-----------
- Found by sampling ceiling faces (face normal pointing predominantly downward,
  i.e. normal_z < -0.5) using Poisson-disk distribution.
- Modelled as parametric cones (base attached to ceiling, tip pointing down).
- Height and base radius are randomised within cave-appropriate ranges.

Rubble / rocks
--------------
- Found by sampling floor faces (normal_z > 0.5).
- Modelled as random convex hulls of a jittered icosphere for organic shapes.
- Scaled to small boulder-sized objects (0.2–1.5 m).
"""

from __future__ import annotations

import logging
import os
from typing import List, Optional, Tuple

import numpy as np
import trimesh
from trimesh.creation import cone, icosphere

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Poisson-disk sampling on mesh faces
# ---------------------------------------------------------------------------

def _poisson_disk_sample_faces(
    mesh: trimesh.Trimesh,
    face_mask: np.ndarray,
    min_dist: float,
    rng: np.random.Generator,
    max_samples: int = 500,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Sample face centroids on *mesh* subject to *face_mask*, using a
    dart-throwing Poisson-disk rejection strategy.

    Parameters
    ----------
    mesh : trimesh.Trimesh
    face_mask : ndarray bool, shape (n_faces,)
        Which faces are eligible.
    min_dist : float
        Minimum distance between two accepted sample points.
    rng : np.random.Generator
    max_samples : int
        Hard cap on number of returned samples.

    Returns
    -------
    positions : ndarray shape (K, 3)   world-space positions on mesh surface
    normals   : ndarray shape (K, 3)   face normals at sample positions
    """
    eligible_idx = np.where(face_mask)[0]
    if len(eligible_idx) == 0:
        return np.empty((0, 3)), np.empty((0, 3))

    # Compute face areas for weighted sampling
    areas = mesh.area_faces[eligible_idx]
    total_area = areas.sum()
    if total_area < 1e-9:
        return np.empty((0, 3)), np.empty((0, 3))
    probs = areas / total_area

    centroids = mesh.triangles_center[eligible_idx]
    normals = mesh.face_normals[eligible_idx]

    accepted_pos: List[np.ndarray] = []
    accepted_nrm: List[np.ndarray] = []
    # Keep accepted positions as a contiguous array for vectorised distance checks
    accepted_arr = np.empty((max_samples, 3), dtype=np.float64)
    n_accepted = 0

    # Cap attempts: 5× the desired sample count is enough for dart-throwing
    attempts = max(max_samples * 5, 2000)
    min_dist_sq = min_dist * min_dist

    for _ in range(attempts):
        if n_accepted >= max_samples:
            break
        i = rng.choice(len(eligible_idx), p=probs)
        # Random point within the chosen triangle
        v0, v1, v2 = mesh.triangles[eligible_idx[i]]
        r1, r2 = rng.random(), rng.random()
        if r1 + r2 > 1.0:
            r1, r2 = 1.0 - r1, 1.0 - r2
        candidate = v0 + r1 * (v1 - v0) + r2 * (v2 - v0)

        # Vectorised distance check against all accepted points
        if n_accepted > 0:
            diffs = accepted_arr[:n_accepted] - candidate
            dists_sq = np.einsum('ij,ij->i', diffs, diffs)
            if np.any(dists_sq < min_dist_sq):
                continue

        accepted_arr[n_accepted] = candidate
        accepted_pos.append(candidate)
        accepted_nrm.append(normals[i])
        n_accepted += 1

    if not accepted_pos:
        return np.empty((0, 3)), np.empty((0, 3))
    return np.array(accepted_pos), np.array(accepted_nrm)


# ---------------------------------------------------------------------------
# Parametric obstacle shapes
# ---------------------------------------------------------------------------

def _make_stalactite(
    height: float,
    base_radius: float,
    sections: int = 8,
) -> trimesh.Trimesh:
    """
    Create a cone mesh representing a stalactite.

    The base is at z=0 (ceiling attachment), the tip is at z=-height.

    Parameters
    ----------
    height : float
        Length of the stalactite in metres.
    base_radius : float
        Radius of the base in metres.
    sections : int
        Number of circumferential segments.
    """
    # trimesh.creation.cone: tip at origin, base at z=height
    m = cone(radius=base_radius, height=height, sections=sections)
    # Flip so base is at z=0 and tip points down (-z)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
    return m


def _make_rock(
    size: float,
    rng: np.random.Generator,
    subdivisions: int = 2,
) -> trimesh.Trimesh:
    """
    Create a random convex rock mesh by jittering icosphere vertices.

    Parameters
    ----------
    size : float
        Approximate bounding-sphere radius in metres.
    rng : np.random.Generator
    subdivisions : int
        Icosphere subdivision level (2 = 80 faces, 3 = 320 faces).
    """
    base = icosphere(subdivisions=subdivisions, radius=size)
    # Jitter vertex positions for organic shape
    jitter = rng.uniform(-size * 0.35, size * 0.35, size=base.vertices.shape)
    jittered_verts = base.vertices + jitter
    try:
        rock = trimesh.convex.convex_hull(trimesh.Trimesh(vertices=jittered_verts))
    except Exception:
        rock = trimesh.Trimesh(
            vertices=jittered_verts,
            faces=base.faces,
            process=True,
        )
    return rock


# ---------------------------------------------------------------------------
# Main class
# ---------------------------------------------------------------------------

class ObstaclePlacer:
    """
    Place stalactites and rubble on a cave mesh surface.

    Parameters
    ----------
    seed : int, optional
        Random seed for deterministic placement.
    """

    def __init__(self, seed: Optional[int] = None) -> None:
        self._rng = np.random.default_rng(seed)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def place_stalactites(
        self,
        mesh: trimesh.Trimesh,
        density: float = 0.3,
        height_range: Tuple[float, float] = (0.5, 3.0),
        radius_range: Tuple[float, float] = (0.1, 0.5),
    ) -> List[trimesh.Trimesh]:
        """
        Place stalactites on ceiling faces of the cave mesh.

        Parameters
        ----------
        mesh : trimesh.Trimesh
            Cave mesh with inward-facing normals.  Ceiling = normals pointing
            in the +z direction (up) from the face; since normals are inward
            for the cave interior, ceiling faces have normals with z > 0
            when viewed from inside.  We actually identify ceiling attachment
            points by finding faces where the world-space normal has z < -0.5
            (outward = downward from the rock mass perspective).
        density : float
            Approximate coverage density [0, 1]; controls min spacing.
        height_range : tuple[float, float]
            (min_height, max_height) of stalactites in metres.
        radius_range : tuple[float, float]
            (min_radius, max_radius) of stalactite bases in metres.

        Returns
        -------
        list of trimesh.Trimesh
            One mesh per placed stalactite, already transformed to world position.
        """
        # Ceiling faces: inward normals pointing in +Z direction
        # (from inside the cave looking up, normals face UP = ceiling)
        # Since we flipped normals to face inward, ceiling faces have normal_z > 0.5
        face_normals = mesh.face_normals
        ceiling_mask = face_normals[:, 2] > 0.5

        logger.info(
            "Stalactite placement: %d/%d ceiling faces eligible.",
            ceiling_mask.sum(), len(face_normals),
        )

        # Minimum spacing derived from density and typical stalactite size
        min_dist = max(0.5, radius_range[1] * 2 / max(density, 0.05))
        min_dist = min(min_dist, 5.0)

        positions, normals = _poisson_disk_sample_faces(
            mesh, ceiling_mask, min_dist, self._rng, max_samples=200,
        )

        stalactites: List[trimesh.Trimesh] = []
        for pos, nrm in zip(positions, normals):
            height = float(self._rng.uniform(*height_range))
            radius = float(self._rng.uniform(*radius_range))
            stal = _make_stalactite(height, radius)

            # Align stalactite tip direction to the negated surface normal
            # (grow away from ceiling into the cave)
            target_dir = -nrm  # direction from ceiling surface into cave
            target_dir /= np.linalg.norm(target_dir) + 1e-9
            # Default stalactite tip is at -Z; rotate to align with target_dir
            default_dir = np.array([0.0, 0.0, -1.0])
            rot = _rotation_between(default_dir, target_dir)
            stal.apply_transform(rot)

            # Translate so base is at face position
            stal.apply_translation(pos)
            stalactites.append(stal)

        logger.info("Placed %d stalactites.", len(stalactites))
        return stalactites

    def place_rubble(
        self,
        mesh: trimesh.Trimesh,
        density: float = 0.2,
        size_range: Tuple[float, float] = (0.15, 0.75),
    ) -> List[trimesh.Trimesh]:
        """
        Place rubble rocks on floor faces of the cave mesh.

        Parameters
        ----------
        mesh : trimesh.Trimesh
            Cave mesh with inward-facing normals.  Floor faces = normal_z < -0.5
            (normals pointing downward = floor of the cave, viewed from inside).
        density : float
            Approximate coverage density [0, 1].
        size_range : tuple[float, float]
            (min_size, max_size) approximate rock radius in metres.

        Returns
        -------
        list of trimesh.Trimesh
            One mesh per rock, transformed to world position.
        """
        face_normals = mesh.face_normals
        floor_mask = face_normals[:, 2] < -0.5

        logger.info(
            "Rubble placement: %d/%d floor faces eligible.",
            floor_mask.sum(), len(face_normals),
        )

        min_dist = max(0.4, size_range[1] * 2.5 / max(density, 0.05))
        min_dist = min(min_dist, 4.0)

        positions, normals = _poisson_disk_sample_faces(
            mesh, floor_mask, min_dist, self._rng, max_samples=300,
        )

        rocks: List[trimesh.Trimesh] = []
        for pos, nrm in zip(positions, normals):
            size = float(self._rng.uniform(*size_range))
            rock = _make_rock(size, self._rng)

            # Random rotation around the up axis for variety
            angle = float(self._rng.uniform(0, 2 * np.pi))
            rot_z = trimesh.transformations.rotation_matrix(angle, [0, 0, 1])
            rock.apply_transform(rot_z)

            # Sit the rock on the floor: translate so bottom touches the surface
            # The rock centre is at origin; move up by its approximate radius
            rock.apply_translation(pos + nrm * size * 0.3)
            rocks.append(rock)

        logger.info("Placed %d rubble rocks.", len(rocks))
        return rocks

    def export_obstacles(
        self,
        obstacles: List[trimesh.Trimesh],
        output_dir: str,
        prefix: str = "obstacle",
    ) -> List[str]:
        """
        Export a list of obstacle meshes as individual STL files.

        Parameters
        ----------
        obstacles : list of trimesh.Trimesh
        output_dir : str
            Directory to write STL files into.
        prefix : str
            Filename prefix.

        Returns
        -------
        list of str
            Absolute paths to the written STL files.
        """
        os.makedirs(output_dir, exist_ok=True)
        paths: List[str] = []
        for i, obs in enumerate(obstacles):
            path = os.path.join(output_dir, f"{prefix}_{i:04d}.stl")
            obs.export(path)
            paths.append(path)
        logger.info("Exported %d obstacles to %s.", len(paths), output_dir)
        return paths


# ---------------------------------------------------------------------------
# Geometry helper
# ---------------------------------------------------------------------------

def _rotation_between(
    vec_from: np.ndarray,
    vec_to: np.ndarray,
) -> np.ndarray:
    """
    Compute a 4×4 rotation matrix that rotates *vec_from* to *vec_to*.

    Uses Rodrigues' rotation formula.
    """
    a = vec_from / (np.linalg.norm(vec_from) + 1e-9)
    b = vec_to / (np.linalg.norm(vec_to) + 1e-9)
    cross = np.cross(a, b)
    cross_norm = np.linalg.norm(cross)

    if cross_norm < 1e-9:
        # Parallel or anti-parallel
        if np.dot(a, b) > 0:
            return np.eye(4)
        else:
            # 180° rotation around any perpendicular axis
            perp = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            axis = np.cross(a, perp)
            axis /= np.linalg.norm(axis)
            return trimesh.transformations.rotation_matrix(np.pi, axis)

    axis = cross / cross_norm
    angle = np.arctan2(cross_norm, np.dot(a, b))
    return trimesh.transformations.rotation_matrix(angle, axis)
