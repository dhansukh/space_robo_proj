"""
cave_mesh.py
============

CaveMeshGenerator: converts a CaveGraph into two trimesh.Trimesh objects:

* **visual mesh**    — full-resolution, normals facing inward (cave interior)
* **collision mesh** — decimated version for Gazebo physics

Pipeline per edge
-----------------
1. Fit a Catmull-Rom spline through start → 2 random midpoints → end.
2. Sample the spline densely and carve a variable-radius tunnel into the SDF
   volume using smooth-union of spheres.
3. At node positions carve spherical chambers with each node's *radius*.
4. Displace the SDF with 3-D OpenSimplex noise for a rocky appearance.
5. Run scikit-image marching_cubes at iso-surface threshold = 0.
6. Build trimesh, fix normals to face inward, remove degenerate faces.
7. Return high-poly visual mesh + decimated collision mesh.
"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

import numpy as np
import trimesh
from skimage.measure import marching_cubes

from cave_generator.cave_graph import CaveGraph, SQUEEZE_RADIUS

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Catmull-Rom spline helpers
# ---------------------------------------------------------------------------

def _catmull_rom_segment(
    p0: np.ndarray,
    p1: np.ndarray,
    p2: np.ndarray,
    p3: np.ndarray,
    n_samples: int = 40,
) -> np.ndarray:
    """
    Evaluate a single Catmull-Rom segment from *p1* to *p2*.

    Parameters
    ----------
    p0, p1, p2, p3 : ndarray shape (3,)
        Control points; the segment runs from p1 → p2.
    n_samples : int
        Number of evenly-spaced parameter values in [0, 1].

    Returns
    -------
    ndarray shape (n_samples, 3)
    """
    t = np.linspace(0.0, 1.0, n_samples)
    # Standard Catmull-Rom matrix formulation
    t2 = t ** 2
    t3 = t ** 3
    # Basis
    b0 = -0.5 * t3 + t2 - 0.5 * t
    b1 = 1.5 * t3 - 2.5 * t2 + 1.0
    b2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t
    b3 = 0.5 * t3 - 0.5 * t2
    # Shape: (n_samples, 3)
    points = (
        np.outer(b0, p0)
        + np.outer(b1, p1)
        + np.outer(b2, p2)
        + np.outer(b3, p3)
    )
    return points


def catmull_rom_spline(
    control_points: List[np.ndarray],
    n_samples_per_segment: int = 40,
) -> np.ndarray:
    """
    Evaluate a Catmull-Rom spline through all *control_points*.

    Ghost points are added at both ends by reflection.

    Parameters
    ----------
    control_points : list of ndarray shape (3,)
        At least 2 points.
    n_samples_per_segment : int
        Samples per segment (between consecutive control points).

    Returns
    -------
    ndarray shape (N, 3)
    """
    pts = [np.asarray(p, dtype=np.float64) for p in control_points]
    if len(pts) < 2:
        return np.array(pts)

    # Add ghost points by reflection
    pts_ext = [2 * pts[0] - pts[1]] + pts + [2 * pts[-1] - pts[-2]]

    all_samples: List[np.ndarray] = []
    for i in range(len(pts) - 1):
        seg = _catmull_rom_segment(
            pts_ext[i],
            pts_ext[i + 1],
            pts_ext[i + 2],
            pts_ext[i + 3],
            n_samples=n_samples_per_segment,
        )
        if i > 0:
            seg = seg[1:]  # avoid duplicate junction points
        all_samples.append(seg)

    return np.vstack(all_samples)


# ---------------------------------------------------------------------------
# SDF primitives
# ---------------------------------------------------------------------------

def sdf_sphere(
    grid_x: np.ndarray,
    grid_y: np.ndarray,
    grid_z: np.ndarray,
    cx: float,
    cy: float,
    cz: float,
    radius: float,
) -> np.ndarray:
    """
    SDF of a sphere: negative inside, positive outside.

    Parameters
    ----------
    grid_x, grid_y, grid_z : ndarray (broadcast-compatible)
        Voxel grid coordinate arrays.
    cx, cy, cz : float
        Sphere centre.
    radius : float
        Sphere radius.

    Returns
    -------
    ndarray  (same shape as inputs after broadcasting)
    """
    return np.sqrt(
        (grid_x - cx) ** 2 + (grid_y - cy) ** 2 + (grid_z - cz) ** 2
    ) - radius


def smooth_union(a: np.ndarray, b: np.ndarray, k: float = 0.5) -> np.ndarray:
    """
    Smooth-union of two SDF fields using the exponential formulation.

    Parameters
    ----------
    a, b : ndarray
        Input SDF fields (same shape).
    k : float
        Smoothing radius (higher = more blending).

    Returns
    -------
    ndarray
    """
    # Clamp to avoid overflow in exp
    a_clamped = np.clip(a, -100.0, 100.0)
    b_clamped = np.clip(b, -100.0, 100.0)
    return -np.log(
        np.exp(-k * a_clamped) + np.exp(-k * b_clamped) + 1e-30
    ) / k


# ---------------------------------------------------------------------------
# Main generator
# ---------------------------------------------------------------------------

class CaveMeshGenerator:
    """
    Generate a 3-D cave mesh from a CaveGraph.

    Parameters
    ----------
    graph : CaveGraph
        A fully generated cave topology.
    resolution : float
        Voxel grid cell size in metres.  Default 0.3 m.
    seed : int, optional
        Random seed for midpoint jitter and noise.
    noise_amplitude : float
        Peak displacement from Simplex noise, as a fraction of minimum radius.
        Default 0.25 (i.e. 25 % of the smallest tunnel radius).
    noise_frequency : float
        Spatial frequency of the Simplex noise field.  Default 0.08.
    smooth_k : float
        Smoothing factor for smooth-union SDF blending.  Default 0.5.
    collision_target_faces : int
        Target face count for the decimated collision mesh.  Default 8 000.
    spline_samples : int
        Samples per spline segment.  Default 40.
    """

    def __init__(
        self,
        graph: CaveGraph,
        resolution: float = 0.3,
        seed: Optional[int] = None,
        noise_amplitude: float = 0.25,
        noise_frequency: float = 0.08,
        smooth_k: float = 0.5,
        collision_target_faces: int = 8_000,
        spline_samples: int = 40,
    ) -> None:
        self.graph = graph
        self.resolution = resolution
        self._rng = np.random.default_rng(seed)
        self._py_rng = np.random.default_rng(seed)  # kept for compatibility
        self.noise_amplitude_frac = noise_amplitude
        self.noise_frequency = noise_frequency
        self.smooth_k = smooth_k
        self.collision_target_faces = collision_target_faces
        self.spline_samples = spline_samples

        # Lazy-initialised
        self._sdf: Optional[np.ndarray] = None
        self._grid_origin: Optional[np.ndarray] = None
        self._grid_shape: Optional[Tuple[int, int, int]] = None

        # Cached splines: populated during _carve_graph for export
        self._cached_splines: dict = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def generate(self) -> Tuple[trimesh.Trimesh, trimesh.Trimesh]:
        """
        Run the full pipeline and return ``(visual_mesh, collision_mesh)``.

        Returns
        -------
        visual_mesh : trimesh.Trimesh
            High-resolution cave interior mesh (normals inward).
        collision_mesh : trimesh.Trimesh
            Decimated mesh for Gazebo collision geometry.
        """
        logger.info("Building SDF volume …")
        self._init_sdf_volume()
        self._carve_graph()
        self._apply_noise()

        logger.info("Running marching cubes …")
        visual_mesh = self._extract_mesh()

        logger.info("Building collision mesh …")
        collision_mesh = self._decimate(visual_mesh)

        return visual_mesh, collision_mesh

    def get_splines(self) -> List[np.ndarray]:
        """
        Return the spline paths for all edges (useful for visualisation).

        Returns
        -------
        list of ndarray shape (N, 3)
        """
        return self._build_all_splines()

    def get_centerlines(self) -> dict:
        """
        Return cached tunnel centerline data from the most recent ``generate()``.

        Must be called **after** ``generate()``.  The returned dict maps
        edge keys ``"u-v"`` to lists of ``[x, y, z]`` waypoints that trace
        the Catmull-Rom spline centreline of each tunnel.

        Returns
        -------
        dict
            ``{"edges": {"0-8": [[x,y,z], ...], ...}}``
        """
        if not self._cached_splines:
            logger.warning(
                "get_centerlines() called before generate() — "
                "returning empty dict."
            )
        return {"edges": self._cached_splines}

    # ------------------------------------------------------------------
    # Private: SDF volume initialisation
    # ------------------------------------------------------------------

    def _init_sdf_volume(self) -> None:
        """Allocate the SDF voxel grid, initialised to a large positive value."""
        all_pos = np.array([
            self.graph.graph.nodes[n]["position"]
            for n in self.graph.graph.nodes
        ])
        # Bounding box with padding equal to the largest chamber radius + noise buffer
        max_radius = max(
            self.graph.graph.nodes[n]["radius"]
            for n in self.graph.graph.nodes
        )
        pad = max_radius + 4.0
        lo = all_pos.min(axis=0) - pad
        hi = all_pos.max(axis=0) + pad

        self._grid_origin = lo.astype(np.float64)
        shape = np.ceil((hi - lo) / self.resolution).astype(int)
        # Enforce odd dimensions for marching cubes
        self._grid_shape = tuple((s + 1 if s % 2 == 0 else s) for s in shape)
        logger.info(
            "SDF grid: shape=%s, origin=%s, res=%.3f m",
            self._grid_shape, self._grid_origin, self.resolution,
        )
        # Positive large value = "solid rock"
        self._sdf = np.full(self._grid_shape, fill_value=1e6, dtype=np.float64)

        # Build coordinate arrays once
        ix = np.arange(self._grid_shape[0])
        iy = np.arange(self._grid_shape[1])
        iz = np.arange(self._grid_shape[2])
        gi, gj, gk = np.meshgrid(ix, iy, iz, indexing="ij")
        self._gx = self._grid_origin[0] + gi * self.resolution
        self._gy = self._grid_origin[1] + gj * self.resolution
        self._gz = self._grid_origin[2] + gk * self.resolution

    # ------------------------------------------------------------------
    # Private: carving
    # ------------------------------------------------------------------

    def _carve_sphere_local(self, cx: float, cy: float, cz: float, radius: float) -> None:
        """
        Carve a single sphere into the SDF using localised bounding-box slicing.

        Instead of computing over the entire volume, only touches voxels
        within `2 × radius` of the sphere centre — dramatic speedup.
        """
        pad = radius * 2.0  # generous padding for smooth-union blending
        # Index bounds from world coordinates
        lo_idx = np.maximum(
            np.floor((np.array([cx, cy, cz]) - pad - self._grid_origin) / self.resolution).astype(int),
            0,
        )
        hi_idx = np.minimum(
            np.ceil((np.array([cx, cy, cz]) + pad - self._grid_origin) / self.resolution).astype(int) + 1,
            np.array(self._grid_shape),
        )
        sx = slice(lo_idx[0], hi_idx[0])
        sy = slice(lo_idx[1], hi_idx[1])
        sz = slice(lo_idx[2], hi_idx[2])

        gx_sub = self._gx[sx, sy, sz]
        gy_sub = self._gy[sx, sy, sz]
        gz_sub = self._gz[sx, sy, sz]

        sphere_sdf = sdf_sphere(gx_sub, gy_sub, gz_sub, cx, cy, cz, radius)
        self._sdf[sx, sy, sz] = np.minimum(self._sdf[sx, sy, sz], sphere_sdf)

    def _carve_graph(self) -> None:
        """Carve chambers at nodes and tunnels along edges into the SDF volume."""
        G = self.graph.graph

        # Carve chambers
        for n in G.nodes:
            pos = G.nodes[n]["position"]
            r = G.nodes[n]["radius"]
            self._carve_sphere_local(pos[0], pos[1], pos[2], r)

        # Carve tunnels along splines
        for u, v in G.edges:
            self._carve_edge(u, v)

    def _carve_edge(self, u: int, v: int) -> None:
        """Carve a single edge as a variable-radius spline tunnel."""
        G = self.graph.graph
        edge_data = G.edges[u, v]
        tunnel_radius = edge_data["width"]
        has_squeeze = edge_data["has_squeeze"]

        # Build control points
        start = np.array(G.nodes[u]["position"])
        end = np.array(G.nodes[v]["position"])
        mid1, mid2 = self._random_midpoints(start, end)
        ctrl = [start, mid1, mid2, end]
        spline = catmull_rom_spline(ctrl, n_samples_per_segment=self.spline_samples)

        # Cache the spline centerline for SLAM path planning export
        key = f"{u}-{v}"
        self._cached_splines[key] = spline.tolist()

        n_pts = len(spline)
        for i, pt in enumerate(spline):
            # Vary the radius along the spline: taper at ends, squeeze in middle
            t = i / max(n_pts - 1, 1)
            # Taper factor: full radius from 10% to 90% of spline, taper at ends
            taper = np.clip(
                np.sin(t * np.pi) * 1.2, 0.0, 1.0
            )
            r = tunnel_radius * taper
            if has_squeeze:
                # Squeeze at spline midpoint (t ≈ 0.5)
                squeeze_factor = 1.0 - 0.5 * np.exp(-((t - 0.5) / 0.1) ** 2)
                r = max(r * squeeze_factor, SQUEEZE_RADIUS)
            r = max(r, 2.5)  # never narrower than 5 m diameter

            self._carve_sphere_local(pt[0], pt[1], pt[2], r)

    def _random_midpoints(
        self, start: np.ndarray, end: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate two random midpoints between *start* and *end*.

        The midpoints are displaced laterally and vertically by up to
        30 % of the edge length for organic tunnel shapes.
        """
        seg = end - start
        length = np.linalg.norm(seg)
        lateral_scale = length * 0.30

        # Unit perpendicular vectors in the plane perpendicular to seg
        if abs(seg[0]) < 0.9 * length:
            perp1 = np.cross(seg, [1, 0, 0])
        else:
            perp1 = np.cross(seg, [0, 1, 0])
        perp1 /= np.linalg.norm(perp1)
        perp2 = np.cross(seg, perp1)
        perp2 /= np.linalg.norm(perp2)

        offsets = self._rng.uniform(-lateral_scale, lateral_scale, size=(2, 3))
        mid1 = start + (end - start) * 0.33 + offsets[0, 0] * perp1 + offsets[0, 1] * perp2
        mid2 = start + (end - start) * 0.66 + offsets[1, 0] * perp1 + offsets[1, 1] * perp2

        # Planar cave: keep midpoints at z=0 so tunnels don't wander vertically
        mid1[2] = 0.0
        mid2[2] = 0.0

        return mid1, mid2

    # ------------------------------------------------------------------
    # Private: noise displacement
    # ------------------------------------------------------------------

    def _apply_noise(self) -> None:
        """
        Add 3-D noise to the SDF field to create rocky surface texture.

        Fallback chain: opensimplex → noise → pure-numpy hash-based noise.
        """
        _backend = None  # 'opensimplex', 'noise', or 'numpy'

        try:
            from opensimplex import noise3array  # noqa: F401
            _backend = 'opensimplex'
        except Exception:
            pass

        if _backend is None:
            try:
                import noise as _noise_lib  # noqa: F401
                _backend = 'noise'
            except Exception:
                pass

        if _backend is None:
            _backend = 'numpy'

        # Minimum tunnel radius across all edges
        min_radius = min(
            self.graph.graph.edges[u, v]["width"]
            for u, v in self.graph.graph.edges
        )
        amplitude = self.noise_amplitude_frac * min_radius
        freq = self.noise_frequency

        logger.info(
            "Applying noise (backend=%s): amplitude=%.3f m, frequency=%.4f",
            _backend, amplitude, freq,
        )

        noise_field = None

        if _backend == 'opensimplex':
            try:
                from opensimplex import noise3array
                ax_x = (self._grid_origin[0] + np.arange(self._grid_shape[0]) * self.resolution) * freq
                ax_y = (self._grid_origin[1] + np.arange(self._grid_shape[1]) * self.resolution) * freq
                ax_z = (self._grid_origin[2] + np.arange(self._grid_shape[2]) * self.resolution) * freq
                noise_field = noise3array(ax_x, ax_y, ax_z).T * amplitude
            except Exception:
                logger.warning("opensimplex noise3array failed at runtime.")
                _backend = 'numpy'

        if _backend == 'noise' and noise_field is None:
            try:
                import noise as _noise_lib
                gx_flat = (self._gx * freq).ravel()
                gy_flat = (self._gy * freq).ravel()
                gz_flat = (self._gz * freq).ravel()
                noise_vals = np.array([
                    _noise_lib.snoise3(float(x), float(y), float(z))
                    for x, y, z in zip(gx_flat, gy_flat, gz_flat)
                ])
                noise_field = noise_vals.reshape(self._grid_shape) * amplitude
            except Exception:
                logger.warning("noise package failed at runtime.")
                _backend = 'numpy'

        if noise_field is None:
            # Pure-numpy fallback: multi-octave sin-based pseudo noise
            logger.info("Using pure-numpy pseudo-noise (no external noise library).")
            gx = self._gx * freq
            gy = self._gy * freq
            gz = self._gz * freq
            noise_field = (
                0.50 * np.sin(gx * 1.7 + gy * 2.3 + gz * 3.1 + 0.5)
                + 0.25 * np.sin(gx * 4.1 - gy * 3.7 + gz * 1.9 + 1.3)
                + 0.15 * np.sin(gx * 7.3 + gy * 5.1 - gz * 4.7 + 2.7)
                + 0.10 * np.sin(gx * 11.3 - gy * 9.7 + gz * 8.3 + 4.1)
            ) * amplitude

        # Only displace near the carved surface (don't push open-air voxels)
        surface_mask = np.abs(self._sdf) < (amplitude * 4.0)
        self._sdf[surface_mask] += noise_field[surface_mask]

    # ------------------------------------------------------------------
    # Private: mesh extraction
    # ------------------------------------------------------------------

    def _extract_mesh(self) -> trimesh.Trimesh:
        """
        Run marching cubes on the SDF field, build a trimesh, fix normals.

        The iso-surface threshold is 0: negative SDF = inside the cave (air),
        positive = rock.  After extraction we flip normals so they point inward
        (the cave interior is what the drone sees).
        """
        try:
            verts, faces, normals, _ = marching_cubes(
                self._sdf,
                level=0.0,
                spacing=(self.resolution, self.resolution, self.resolution),
            )
        except ValueError as exc:
            logger.error("marching_cubes failed: %s", exc)
            raise

        # Translate verts from grid-local to world coordinates
        verts += self._grid_origin.astype(np.float32)

        mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)

        # Remove degenerate faces via process()
        mesh.process(validate=True)

        # --- Remove debris micro-bodies ---
        # SDF noise can create tiny isolated pockets; keep only the largest body.
        if mesh.body_count > 1:
            bodies = mesh.split(only_watertight=False)
            bodies.sort(key=lambda b: len(b.faces), reverse=True)
            logger.info(
                "Mesh had %d bodies; keeping largest (%d faces), "
                "discarding %d debris bodies.",
                len(bodies), len(bodies[0].faces), len(bodies) - 1,
            )
            mesh = bodies[0]

        # --- Fix normals to face inward (toward the cave interior) ---
        # 1. fix_normals() makes winding consistent and outward-pointing (positive volume).
        # 2. invert() flips faces + normals so they point inward — toward the air
        #    the drone camera sees.  This prevents OGRE2 back-face culling from
        #    hiding cave walls and ensures LiDAR rays hit the interior surface.
        mesh.fix_normals(multibody=True)
        mesh.invert()

        logger.info(
            "Extracted mesh: %d verts, %d faces (volume=%.1f, expect negative)",
            len(mesh.vertices), len(mesh.faces), mesh.volume,
        )
        return mesh

    def _decimate(self, mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        """
        Return a decimated copy of *mesh* suitable for collision geometry.

        Uses trimesh's built-in simplification (quadric decimation if available,
        otherwise vertex clustering).

        Parameters
        ----------
        mesh : trimesh.Trimesh
            High-resolution source mesh.

        Returns
        -------
        trimesh.Trimesh
        """
        n_faces = len(mesh.faces)
        if n_faces <= self.collision_target_faces:
            logger.info(
                "Mesh already has %d faces (<= target %d); skipping decimation.",
                n_faces, self.collision_target_faces,
            )
            return mesh.copy()

        collision: Optional[trimesh.Trimesh] = None

        # Strategy 1: quadric decimation (best quality)
        try:
            collision = mesh.simplify_quadric_decimation(
                face_count=self.collision_target_faces
            )
            if collision is None or len(collision.faces) == 0:
                raise ValueError("simplify_quadric_decimation returned empty mesh")
            logger.info(
                "Quadric decimation: %d → %d faces.",
                n_faces, len(collision.faces),
            )
        except Exception as exc:
            logger.warning(
                "simplify_quadric_decimation failed (%s); trying vertex clustering.",
                exc,
            )
            collision = None

        # Strategy 2: vertex clustering (coarser but reliable)
        if collision is None:
            try:
                # Binary-search for a voxel size yielding close to target face count.
                bbox_diag = np.linalg.norm(
                    mesh.bounds[1] - mesh.bounds[0]
                )
                lo_vs = self.resolution * 1.5   # finest reasonable voxel
                hi_vs = bbox_diag / 4.0          # coarsest reasonable voxel
                best_mesh = None
                best_diff = float("inf")

                for _ in range(10):  # ~10 iterations of bisection
                    mid_vs = (lo_vs + hi_vs) / 2.0
                    voxelized = mesh.voxelized(mid_vs)
                    candidate = voxelized.marching_cubes
                    n_cand = len(candidate.faces)
                    diff = abs(n_cand - self.collision_target_faces)
                    if diff < best_diff:
                        best_diff = diff
                        best_mesh = candidate
                        best_vs = mid_vs
                    if n_cand > self.collision_target_faces:
                        lo_vs = mid_vs   # too many faces → coarser voxels
                    else:
                        hi_vs = mid_vs   # too few faces → finer voxels

                collision = best_mesh
                # Preserve inward-facing normals
                if collision.volume > 0:
                    collision.invert()
                logger.info(
                    "Vertex clustering (voxel=%.3f m): %d → %d faces.",
                    best_vs, n_faces, len(collision.faces),
                )
            except Exception as exc2:
                logger.warning(
                    "Vertex clustering also failed (%s); using original mesh.",
                    exc2,
                )
                collision = mesh.copy()

        logger.info(
            "Collision mesh: %d faces (target %d)",
            len(collision.faces), self.collision_target_faces,
        )
        return collision

    # ------------------------------------------------------------------
    # Private: spline export helper
    # ------------------------------------------------------------------

    def _build_all_splines(self) -> List[np.ndarray]:
        """Build and return spline paths for all graph edges."""
        G = self.graph.graph
        splines: List[np.ndarray] = []
        for u, v in G.edges:
            start = np.array(G.nodes[u]["position"])
            end = np.array(G.nodes[v]["position"])
            mid1, mid2 = self._random_midpoints(start, end)
            ctrl = [start, mid1, mid2, end]
            splines.append(catmull_rom_spline(ctrl, n_samples_per_segment=self.spline_samples))
        return splines
