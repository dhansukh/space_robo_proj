"""
path_planner.py
===============
3-D path planner combining RRT* (primary) with A* on a voxel grid (fallback).

Subscriptions
-------------
/slam/occupied_cells  (sensor_msgs/PointCloud2)  — obstacle voxels

Services
--------
/plan_path  (nav2_msgs/srv/ComputePathToPose)    — request path start→goal

Publications
------------
/planning/path  (nav_msgs/Path)                     — planned path
/planning/tree  (visualization_msgs/MarkerArray)     — RRT* tree (debug)
"""

from __future__ import annotations

import heapq
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import splprep, splev

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.srv import GetPlan
from builtin_interfaces.msg import Duration

from .collision_checker import CollisionCheckerCore


# ---------------------------------------------------------------------------
# RRT* node
# ---------------------------------------------------------------------------

class RRTNode:
    """Single node in the RRT* tree."""
    __slots__ = ('pos', 'parent', 'cost', 'children')

    def __init__(self, pos: np.ndarray, parent: Optional['RRTNode'] = None, cost: float = 0.0) -> None:
        self.pos: np.ndarray = pos
        self.parent: Optional['RRTNode'] = parent
        self.cost: float = cost
        self.children: List['RRTNode'] = []


# ---------------------------------------------------------------------------
# RRT* planner
# ---------------------------------------------------------------------------

class RRTStar:
    """
    3-D RRT* path planner.

    Parameters
    ----------
    checker : CollisionCheckerCore
        Shared collision checker.
    step_size : float
        Maximum extension step in metres.
    max_iterations : int
    goal_bias : float
        Fraction of samples directed toward the goal.
    rewiring_radius : float
        Neighbourhood radius for rewiring.
    safety_margin : float
        Collision check inflation (metres).
    goal_tolerance : float
        Distance at which goal is considered reached.
    """

    def __init__(
        self,
        checker: CollisionCheckerCore,
        step_size: float = 0.5,
        max_iterations: int = 5000,
        goal_bias: float = 0.1,
        rewiring_radius: float = 2.0,
        safety_margin: float = 0.4,
        goal_tolerance: float = 0.3,
        workspace_min: np.ndarray = None,
        workspace_max: np.ndarray = None,
    ) -> None:
        self._checker = checker
        self._step = step_size
        self._max_iter = max_iterations
        self._goal_bias = goal_bias
        self._rewire_radius = rewiring_radius
        self._margin = safety_margin
        self._goal_tol = goal_tolerance
        self._ws_min = workspace_min if workspace_min is not None else np.array([-50, -50, -1])
        self._ws_max = workspace_max if workspace_max is not None else np.array([50, 50, 20])

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        timeout: float = 10.0,
        tree_callback=None,
    ) -> Tuple[Optional[List[np.ndarray]], List[RRTNode]]:
        """
        Plan a path from *start* to *goal*.

        Returns
        -------
        path : list of np.ndarray or None
            Ordered waypoints, or None if planning failed.
        nodes : list of RRTNode
            All tree nodes (for visualisation).
        """
        start = np.asarray(start, dtype=np.float64)
        goal = np.asarray(goal, dtype=np.float64)

        root = RRTNode(start, parent=None, cost=0.0)
        nodes: List[RRTNode] = [root]
        best_goal_node: Optional[RRTNode] = None

        deadline = time.monotonic() + timeout

        for iteration in range(self._max_iter):
            if time.monotonic() > deadline:
                break

            # Sample
            if np.random.random() < self._goal_bias:
                q_rand = goal.copy()
            else:
                q_rand = self._random_sample()

            # Nearest
            q_near = self._nearest(nodes, q_rand)

            # Steer
            q_new_pos = self._steer(q_near.pos, q_rand)

            # Collision check
            if not self._checker.is_path_free(q_near.pos, q_new_pos,
                                               step=0.1,
                                               safety_margin=self._margin):
                continue

            # Find nearby nodes for rewiring
            near_nodes = self._near_nodes(nodes, q_new_pos)

            # Choose best parent
            parent, best_cost = self._choose_parent(near_nodes, q_near, q_new_pos)
            q_new = RRTNode(q_new_pos, parent=parent, cost=best_cost)
            parent.children.append(q_new)
            nodes.append(q_new)

            # Rewire
            self._rewire(near_nodes, q_new)

            # Check goal
            if np.linalg.norm(q_new.pos - goal) <= self._goal_tol:
                if best_goal_node is None or q_new.cost < best_goal_node.cost:
                    best_goal_node = q_new

        if best_goal_node is None:
            return None, nodes

        # Extract path
        path = self._extract_path(best_goal_node)
        return path, nodes

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _random_sample(self) -> np.ndarray:
        return np.random.uniform(self._ws_min, self._ws_max)

    @staticmethod
    def _nearest(nodes: List[RRTNode], q: np.ndarray) -> RRTNode:
        positions = np.array([n.pos for n in nodes])
        dists = np.linalg.norm(positions - q, axis=1)
        return nodes[int(np.argmin(dists))]

    def _steer(self, origin: np.ndarray, target: np.ndarray) -> np.ndarray:
        d = target - origin
        dist = np.linalg.norm(d)
        if dist <= self._step:
            return target.copy()
        return origin + (d / dist) * self._step

    def _near_nodes(self, nodes: List[RRTNode], q: np.ndarray) -> List[RRTNode]:
        return [n for n in nodes if np.linalg.norm(n.pos - q) <= self._rewire_radius]

    def _choose_parent(
        self,
        near_nodes: List[RRTNode],
        default_parent: RRTNode,
        q_new_pos: np.ndarray,
    ) -> Tuple[RRTNode, float]:
        best_parent = default_parent
        best_cost = default_parent.cost + np.linalg.norm(q_new_pos - default_parent.pos)

        for node in near_nodes:
            potential_cost = node.cost + np.linalg.norm(q_new_pos - node.pos)
            if potential_cost < best_cost:
                if self._checker.is_path_free(node.pos, q_new_pos,
                                               step=0.1,
                                               safety_margin=self._margin):
                    best_parent = node
                    best_cost = potential_cost

        return best_parent, best_cost

    def _rewire(self, near_nodes: List[RRTNode], q_new: RRTNode) -> None:
        for node in near_nodes:
            if node is q_new.parent:
                continue
            new_cost = q_new.cost + np.linalg.norm(node.pos - q_new.pos)
            if new_cost < node.cost:
                if self._checker.is_path_free(q_new.pos, node.pos,
                                               step=0.1,
                                               safety_margin=self._margin):
                    # Reparent
                    if node.parent is not None:
                        try:
                            node.parent.children.remove(node)
                        except ValueError:
                            pass
                    node.parent = q_new
                    node.cost = new_cost
                    q_new.children.append(node)
                    # Propagate cost update
                    self._propagate_cost(node)

    def _propagate_cost(self, node: RRTNode) -> None:
        """Recursively update costs for all descendants after rewiring."""
        stack = list(node.children)
        while stack:
            child = stack.pop()
            child.cost = child.parent.cost + np.linalg.norm(child.pos - child.parent.pos)
            stack.extend(child.children)

    @staticmethod
    def _extract_path(goal_node: RRTNode) -> List[np.ndarray]:
        path = []
        node = goal_node
        while node is not None:
            path.append(node.pos.copy())
            node = node.parent
        path.reverse()
        return path


# ---------------------------------------------------------------------------
# A* fallback planner
# ---------------------------------------------------------------------------

class AStarPlanner:
    """
    3-D A* on a voxel grid.  Used as fallback when RRT* times out.
    """

    def __init__(
        self,
        checker: CollisionCheckerCore,
        grid_resolution: float = 0.5,
        safety_margin: float = 0.4,
    ) -> None:
        self._checker = checker
        self._res = grid_resolution
        self._margin = safety_margin

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
    ) -> Optional[List[np.ndarray]]:
        """Return a path or None."""
        s_idx = self._to_idx(start)
        g_idx = self._to_idx(goal)

        # Check if goal is reachable
        if not self._checker.is_point_free(goal, self._margin):
            return None

        # Priority queue: (f, g, idx)
        open_set: List[Tuple[float, float, Tuple[int, int, int]]] = []
        heapq.heappush(open_set, (self._h(s_idx, g_idx), 0.0, s_idx))
        came_from: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {s_idx: None}
        g_score: Dict[Tuple[int, int, int], float] = {s_idx: 0.0}

        neighbours = [
            (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1),
            (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),
            (1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1),
            (0, 1, 1), (0, 1, -1), (0, -1, 1), (0, -1, -1),
        ]

        max_nodes = 50_000
        visited = 0

        while open_set and visited < max_nodes:
            _, g, current = heapq.heappop(open_set)
            visited += 1

            if current == g_idx:
                return self._reconstruct(came_from, current)

            if g > g_score.get(current, float('inf')):
                continue  # stale entry

            for dx, dy, dz in neighbours:
                nb = (current[0] + dx, current[1] + dy, current[2] + dz)
                nb_world = self._to_world(nb)
                if not self._checker.is_point_free(nb_world, self._margin):
                    continue
                step_cost = float(np.sqrt(dx*dx + dy*dy + dz*dz)) * self._res
                new_g = g_score[current] + step_cost
                if new_g < g_score.get(nb, float('inf')):
                    g_score[nb] = new_g
                    came_from[nb] = current
                    f = new_g + self._h(nb, g_idx)
                    heapq.heappush(open_set, (f, new_g, nb))

        return None  # planning failed

    def _to_idx(self, p: np.ndarray) -> Tuple[int, int, int]:
        p = np.asarray(p, dtype=np.float64)
        return (
            int(np.floor(p[0] / self._res)),
            int(np.floor(p[1] / self._res)),
            int(np.floor(p[2] / self._res)),
        )

    def _to_world(self, idx: Tuple[int, int, int]) -> np.ndarray:
        return np.array([
            (idx[0] + 0.5) * self._res,
            (idx[1] + 0.5) * self._res,
            (idx[2] + 0.5) * self._res,
        ], dtype=np.float64)

    def _h(
        self,
        a: Tuple[int, int, int],
        b: Tuple[int, int, int],
    ) -> float:
        """Octile/Chebyshev heuristic (admissible in 3-D)."""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        dz = abs(a[2] - b[2])
        vals = sorted([dx, dy, dz], reverse=True)
        return (vals[0] + (np.sqrt(2) - 1) * vals[1] + (np.sqrt(3) - np.sqrt(2)) * vals[2]) * self._res

    def _reconstruct(
        self,
        came_from: Dict,
        current: Tuple[int, int, int],
    ) -> List[np.ndarray]:
        path = []
        while current is not None:
            path.append(self._to_world(current))
            current = came_from[current]
        path.reverse()
        return path


# ---------------------------------------------------------------------------
# Path smoother
# ---------------------------------------------------------------------------

class PathSmoother:
    """
    Shortcut + B-spline smoothing for a raw waypoint list.
    """

    def __init__(
        self,
        checker: CollisionCheckerCore,
        safety_margin: float = 0.4,
        shortcut_passes: int = 3,
        spline_k: int = 3,
    ) -> None:
        self._checker = checker
        self._margin = safety_margin
        self._passes = shortcut_passes
        self._spline_k = spline_k

    def smooth(self, path: List[np.ndarray]) -> List[np.ndarray]:
        if len(path) < 3:
            return path
        path = self._shortcut(path)
        path = self._bspline(path)
        return path

    def _shortcut(self, path: List[np.ndarray]) -> List[np.ndarray]:
        for _ in range(self._passes):
            i = 0
            while i < len(path) - 2:
                j = len(path) - 1
                while j > i + 1:
                    if self._checker.is_path_free(path[i], path[j],
                                                   step=0.1,
                                                   safety_margin=self._margin):
                        path = path[:i+1] + path[j:]
                        break
                    j -= 1
                i += 1
        return path

    def _bspline(self, path: List[np.ndarray]) -> List[np.ndarray]:
        if len(path) < 4:
            return path
        pts = np.array(path).T  # shape (3, N)
        k = min(self._spline_k, len(path) - 1)
        try:
            tck, _ = splprep(pts, s=0, k=k)
            n_eval = max(50, len(path) * 5)
            u_new = np.linspace(0, 1, n_eval)
            smooth_pts = np.array(splev(u_new, tck)).T  # shape (n_eval, 3)
            return [smooth_pts[i] for i in range(len(smooth_pts))]
        except Exception:  # noqa: BLE001
            return path  # fallback: return unsmoothed


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class PathPlannerNode(Node):
    """
    ROS 2 path planner node.

    Wraps RRT* with an A* fallback and path smoothing.
    Serves path requests via the nav2_msgs/ComputePathToPose service.
    """

    def __init__(self) -> None:
        super().__init__('path_planner')

        # Parameters
        self.declare_parameter('step_size', 0.5)
        self.declare_parameter('max_iterations', 5000)
        self.declare_parameter('goal_bias', 0.1)
        self.declare_parameter('rewiring_radius', 2.0)
        self.declare_parameter('safety_margin', 0.4)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('timeout_sec', 10.0)
        self.declare_parameter('astar_grid_resolution', 0.5)
        self.declare_parameter('smooth_shortcut_passes', 3)
        self.declare_parameter('smooth_spline_k', 3)

        def _dp(name):
            return self.get_parameter(name).get_parameter_value().double_value

        def _ip(name):
            return self.get_parameter(name).get_parameter_value().integer_value

        step = _dp('step_size')
        max_iter = _ip('max_iterations')
        goal_bias = _dp('goal_bias')
        rewire_r = _dp('rewiring_radius')
        margin = _dp('safety_margin')
        goal_tol = _dp('goal_tolerance')
        self._timeout = _dp('timeout_sec')
        astar_res = _dp('astar_grid_resolution')
        sc_passes = _ip('smooth_shortcut_passes')
        spline_k = _ip('smooth_spline_k')

        # Shared collision checker
        self._checker = CollisionCheckerCore(safety_margin=margin)

        # Planners
        self._rrt_star = RRTStar(
            checker=self._checker,
            step_size=step,
            max_iterations=max_iter,
            goal_bias=goal_bias,
            rewiring_radius=rewire_r,
            safety_margin=margin,
            goal_tolerance=goal_tol,
        )
        self._astar = AStarPlanner(
            checker=self._checker,
            grid_resolution=astar_res,
            safety_margin=margin,
        )
        self._smoother = PathSmoother(
            checker=self._checker,
            safety_margin=margin,
            shortcut_passes=sc_passes,
            spline_k=spline_k,
        )

        # QoS
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions
        self._occ_sub = self.create_subscription(
            PointCloud2, '/slam/occupied_cells',
            self._occupied_cb, sensor_qos,
        )

        # Publishers
        self._path_pub = self.create_publisher(
            Path, '/planning/path', reliable_qos
        )
        self._tree_pub = self.create_publisher(
            MarkerArray, '/planning/tree', reliable_qos
        )

        # Service (nav_msgs/GetPlan — no nav2 required)
        self._plan_srv = self.create_service(
            GetPlan,
            '/plan_path',
            self._plan_service_cb,
        )

        self.get_logger().info('PathPlanner ready — RRT* + A* fallback.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _occupied_cb(self, msg: PointCloud2) -> None:
        try:
            pts = np.array(
                [[p[0], p[1], p[2]] for p in
                 pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)],
                dtype=np.float64,
            )
            if pts.shape[0] > 0:
                self._checker.update_obstacles(pts[:, :3])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'PathPlanner: occupied_cb error: {exc}')

    def _plan_service_cb(
        self,
        request: GetPlan.Request,
        response: GetPlan.Response,
    ) -> GetPlan.Response:
        """Handle an incoming path planning service request."""
        start_pose = request.start if hasattr(request, 'start') else None
        goal_pose = request.goal

        # Extract positions
        goal_pos = np.array([
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            goal_pose.pose.position.z,
        ], dtype=np.float64)

        if start_pose is not None and hasattr(start_pose, 'pose'):
            start_pos = np.array([
                start_pose.pose.position.x,
                start_pose.pose.position.y,
                start_pose.pose.position.z,
            ], dtype=np.float64)
        else:
            # Default: use goal altitude at origin
            start_pos = np.array([0.0, 0.0, goal_pos[2]], dtype=np.float64)

        path_waypoints = self._plan(start_pos, goal_pos)

        if path_waypoints is not None:
            ros_path = self._waypoints_to_path(path_waypoints)
            response.plan = ros_path
            self._path_pub.publish(ros_path)
            self.get_logger().info(
                f'PathPlanner: planned path with {len(path_waypoints)} waypoints.'
            )
        else:
            self.get_logger().warn('PathPlanner: planning failed — returning empty path.')
            response.plan = Path()

        return response

    # ------------------------------------------------------------------
    # Core planning
    # ------------------------------------------------------------------

    def _plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
    ) -> Optional[List[np.ndarray]]:
        """Try RRT*, fall back to A*."""
        self.get_logger().info(
            f'Planning {start} → {goal}  (RRT* timeout={self._timeout}s)'
        )

        path, nodes = self._rrt_star.plan(start, goal, timeout=self._timeout)

        if path is not None:
            self.get_logger().info(f'RRT* succeeded ({len(path)} nodes).')
            self._publish_tree(nodes)
            smoothed = self._smoother.smooth(path)
            return smoothed

        self.get_logger().warn('RRT* timed out — falling back to A*.')
        path = self._astar.plan(start, goal)

        if path is not None:
            self.get_logger().info(f'A* succeeded ({len(path)} nodes).')
            smoothed = self._smoother.smooth(path)
            return smoothed

        self.get_logger().error('PathPlanner: both RRT* and A* failed.')
        return None

    # ------------------------------------------------------------------
    # Message helpers
    # ------------------------------------------------------------------

    def _waypoints_to_path(self, waypoints: List[np.ndarray]) -> Path:
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for wp in waypoints:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(wp[0])
            ps.pose.position.y = float(wp[1])
            ps.pose.position.z = float(wp[2])
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        return path_msg

    def _publish_tree(self, nodes: List[RRTNode]) -> None:
        """Publish RRT* tree edges as LINE_LIST markers for RViz."""
        array = MarkerArray()

        delete = Marker()
        delete.action = Marker.DELETEALL
        array.markers.append(delete)

        if not nodes:
            self._tree_pub.publish(array)
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'rrt_tree'
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.2
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.5

        lifetime = Duration()
        lifetime.sec = 5
        marker.lifetime = lifetime

        for node in nodes:
            if node.parent is None:
                continue
            p1 = Point(x=float(node.parent.pos[0]),
                       y=float(node.parent.pos[1]),
                       z=float(node.parent.pos[2]))
            p2 = Point(x=float(node.pos[0]),
                       y=float(node.pos[1]),
                       z=float(node.pos[2]))
            marker.points.append(p1)
            marker.points.append(p2)

        array.markers.append(marker)
        self._tree_pub.publish(array)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
