"""MazeMind frontier explorer.

Inputs (all robot-observable):
  /map                         nav_msgs/OccupancyGrid  live SLAM Toolbox map
  TF map -> base_footprint     SLAM pose estimate
  /scan                        sensor_msgs/LaserScan   only used as a liveness check
  /mazemind/exit/detection     geometry_msgs/PoseStamped from exit_detector
  navigate_to_pose             nav2_msgs/action/NavigateToPose

Outputs:
  /mazemind/explorer/status    std_msgs/String (JSON, 1 Hz)
  /mazemind/outcome            std_msgs/String (JSON, transient-local, once)
  /mazemind/frontiers          visualization_msgs/MarkerArray
  /cmd_vel                     zero twist when stopping

This node has no access to the ground-truth maze; see test_information_boundary.py.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from functools import partial
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from builtin_interfaces.msg import Duration as DurationMsg
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from mazemind_exploration import memory as mem
from mazemind_exploration import outcomes
from mazemind_exploration.frontier import Frontier, detect_frontiers
from mazemind_exploration.grid import (GeodesicField, GridMap, compute_traversability,
                                       nearest_cell_in_mask)
from mazemind_exploration.memory import GoalMemory, StuckDetector
from mazemind_exploration.selection import (Candidate, EvaluationParams, Selector, Weights,
                                            distance, evaluate_candidates)
from mazemind_exploration.transforms import quaternion_from_yaw, yaw_from_quaternion
from mazemind_exploration.visual_coverage import (VisualCoverage, info_grid_with_unseen,
                                                  unseen_regions)

WAITING, EXPLORING, NAVIGATING, EXIT_APPROACH, FINISHED = (
    'WAITING_FOR_SYSTEM', 'EXPLORING', 'NAVIGATING', 'EXIT_APPROACH', 'FINISHED')


@dataclass
class ActiveGoal:
    seq: int
    kind: str                      # 'frontier' | 'exit'
    target: Tuple[float, float]
    sent: float                    # sim time
    timeout: float                 # seconds
    preference: float
    travel_cost: float
    handle: object = None
    recoveries: int = 0
    target_kind: str = 'map'       # frontier kind the goal serves ('map' | 'visual')


class ExplorerNode(Node):

    def __init__(self) -> None:
        super().__init__('explorer')
        p = self.declare_parameter
        self.strategy = p('strategy', 'weighted').value
        seed = int(p('random_seed', 0).value)
        self.time_limit = float(p('time_limit', 600.0).value)
        self.global_frame = p('global_frame', 'map').value
        self.robot_frame = p('robot_frame', 'base_footprint').value
        self.tick_period = float(p('tick_period', 0.5).value)
        self.replan_period = float(p('replan_period', 2.0).value)
        self.min_frontier_size = int(p('min_frontier_size', 8).value)
        self.gap_fill_cells = int(p('gap_fill_cells', 2).value)
        self.border_cells = int(p('border_cells', 3).value)
        self.costmap_margin = float(p('costmap_margin', 0.15).value)
        self.clearance_penalty = float(p('clearance_penalty', 2.0).value)
        # visual search: drive the camera over everything the LiDAR has mapped
        self.visual_search = bool(p('visual_search', True).value)
        self.camera_frame = p('camera_frame', 'camera_link').value
        self.min_unseen_cells = int(p('min_unseen_cells', 60).value)
        self.look_distance = float(p('look_distance', 0.6).value)
        self.coverage = VisualCoverage(resolution=float(p('coverage_resolution', 0.05).value),
                                       view_range=float(p('camera_view_range', 2.5).value),
                                       hfov=float(p('camera_hfov', 1.3).value),
                                       n_rays=int(p('camera_rays', 48).value))
        self.resolve_radius = float(p('resolve_radius', 0.4).value)
        self.min_commit_time = float(p('min_commit_time', 4.0).value)
        self.switch_margin_score = float(p('switch_margin_score', 0.25).value)
        self.switch_margin_distance = float(p('switch_margin_distance', 0.5).value)
        self.goal_timeout_base = float(p('goal_timeout_base', 30.0).value)
        self.goal_timeout_per_meter = float(p('goal_timeout_per_meter', 8.0).value)
        self.completion_checks = int(p('completion_checks', 4).value)
        self.max_consecutive_failures = int(p('max_consecutive_failures', 6).value)
        self.startup_timeout = float(p('startup_timeout', 120.0).value)       # wall clock
        self.sensor_timeout = float(p('sensor_timeout', 10.0).value)          # sim time
        self.nav_unavailable_timeout = float(p('nav_unavailable_timeout', 30.0).value)
        self.exit_approach_offset = float(p('exit_approach_offset', 0.40).value)
        self.exit_arrival_distance = float(p('exit_arrival_distance', 0.70).value)
        self.exit_lateral_tolerance = float(p('exit_lateral_tolerance', 0.30).value)
        self.max_exit_attempts = int(p('max_exit_attempts', 3).value)
        self.max_exit_goals = int(p('max_exit_goals', 8).value)
        self.eval_params = EvaluationParams(
            robot_radius=float(p('robot_radius', 0.15).value),
            safe_clearance=float(p('safe_clearance', 0.35).value),
            clearance_cap=float(p('clearance_cap', 0.45).value),
            info_radius=float(p('info_radius', 2.0).value),
            info_rays=int(p('info_rays', 72).value),
            min_goal_distance=float(p('min_goal_distance', 0.35).value))
        weights = Weights(
            info=float(p('w_info', 1.0).value), size=float(p('w_size', 0.3).value),
            clearance=float(p('w_clearance', 0.3).value), success=float(p('w_success', 0.5).value),
            distance=float(p('w_distance', 1.2).value), revisit=float(p('w_revisit', 0.8).value),
            failure=float(p('w_failure', 1.5).value))
        self.memory = GoalMemory(region_radius=float(p('region_radius', 0.5).value),
                                 max_failures=int(p('max_failures_per_region', 2).value),
                                 max_attempts=int(p('max_attempts_per_region', 3).value))
        self.stuck = StuckDetector(window=float(p('stuck_window', 20.0).value),
                                   min_displacement=float(p('stuck_distance', 0.15).value))
        self.selector = Selector(self.strategy, weights, self.eval_params.clearance_cap, seed)
        self.weights = weights

        # ---------------------------------------------------------------- ROS I/O
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, p('map_topic', '/map').value, self._on_map, map_qos)
        self.create_subscription(LaserScan, p('scan_topic', '/scan').value, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, outcomes.EXIT_DETECTION_TOPIC, self._on_exit, 10)
        self.create_subscription(OccupancyGrid, p('costmap_topic', '/global_costmap/costmap').value,
                                 self._on_costmap, 1)
        self.nav = ActionClient(self, NavigateToPose, p('nav_action', 'navigate_to_pose').value)
        self.spin_client = ActionClient(self, Spin, p('spin_action', 'spin').value)
        self.status_pub = self.create_publisher(String, outcomes.STATUS_TOPIC, 10)
        self.outcome_pub = self.create_publisher(String, outcomes.OUTCOME_TOPIC, latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/mazemind/frontiers', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # travelled path from the robot's own SLAM pose (visualisation)
        self.trail_pub = self.create_publisher(Path, '/mazemind/trail', 1)
        self.trail = Path()
        self.trail.header.frame_id = self.global_frame

        # ---------------------------------------------------------------- state
        self.state = WAITING
        self.grid: Optional[GridMap] = None
        self.map_count = 0
        self.first_map_time: Optional[float] = None
        self.last_scan_time: Optional[float] = None
        self.pose: Optional[Tuple[float, float, float]] = None
        self.goal: Optional[ActiveGoal] = None
        self.goal_seq = 0
        self._pending_cancel: list = []
        self.active_nav_seq: Optional[int] = None
        self.goal_log: List[dict] = []
        self.last_eval_time = -math.inf
        self.last_eval_map = -1
        self.last_frontiers: List[Frontier] = []
        self.last_candidates: List[Candidate] = []
        self.last_rejected: list = []
        self.last_unseen: Optional[np.ndarray] = None
        self.last_unreachable: list = []
        self.costmap_bounds: Optional[Tuple[float, float, float, float]] = None
        self.costmap_lag = False
        self.empty_checks = 0
        self.last_empty_check = -math.inf
        self.consecutive_failures = 0
        self.nav_down_since: Optional[float] = None
        self.exit_estimate: Optional[Tuple[float, float, float]] = None
        self.exit_detect_time: Optional[float] = None
        self.exit_attempts = 0
        self.exit_failures = 0
        self.exit_faced = False
        self.t_ready: Optional[float] = None
        self.wall_start = time.time()
        self.distance_est = 0.0
        self.counters = {'goals_sent': 0, 'reached': 0, 'failed': 0, 'timed_out': 0, 'stuck': 0,
                         'preempted': 0, 'resolved': 0, 'rejected': 0, 'recoveries': 0}
        self.outcome: Optional[dict] = None
        self._stop_ticks = 0

        self.create_timer(self.tick_period, self._tick)
        self.create_timer(0.2, self._update_pose)
        if self.visual_search:
            self.create_timer(0.2, self._observe_camera)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(f'explorer ready: strategy={self.strategy} seed={seed} '
                               f'time_limit={self.time_limit:.0f}s weights={weights}')

    # ===================================================================== helpers
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.grid = GridMap.from_msg(msg)
        self.map_count += 1
        if self.first_map_time is None:
            self.first_map_time = self.now()

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        info = msg.info
        x0, y0 = info.origin.position.x, info.origin.position.y
        self.costmap_bounds = (x0, x0 + info.width * info.resolution,
                               y0, y0 + info.height * info.resolution)

    def _on_scan(self, _msg: LaserScan) -> None:
        self.last_scan_time = self.now()

    def _update_pose(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        new = (t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if self.pose is not None and self.state not in (WAITING, FINISHED):
            self.distance_est += math.hypot(new[0] - self.pose[0], new[1] - self.pose[1])
        self.pose = new
        if self.goal is not None:
            self.stuck.update(self.now(), new[0], new[1])
        if self.state not in (WAITING, FINISHED):
            last = self.trail.poses[-1].pose.position if self.trail.poses else None
            if last is None or math.hypot(new[0] - last.x, new[1] - last.y) > 0.05:
                ps = PoseStamped()
                ps.header.frame_id = self.global_frame
                ps.pose.position.x, ps.pose.position.y = new[0], new[1]
                ps.pose.orientation.w = 1.0
                self.trail.poses.append(ps)
                self.trail.header.stamp = self.get_clock().now().to_msg()
                self.trail_pub.publish(self.trail)

    def _observe_camera(self) -> None:
        if self.grid is None or self.state in (WAITING, FINISHED):
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.camera_frame, Time())
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        self.coverage.observe(self.grid, t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    # ===================================================================== main loop
    def _tick(self) -> None:
        try:
            self._tick_inner()
        finally:
            self._flush_cancels()

    def _flush_cancels(self) -> None:
        """Cancel goals that were closed this tick and not replaced by a new nav goal."""
        for kind, handle in self._pending_cancel:
            handle.cancel_goal_async()
            if kind != 'look':
                self.active_nav_seq = None
        self._pending_cancel = []

    def _tick_inner(self) -> None:
        if self.state == FINISHED:
            self._stop_robot_tick()
            return
        now = self.now()
        if self.state == WAITING:
            self._check_ready(now)
            return

        elapsed = now - self.t_ready
        if elapsed > self.time_limit:
            self._finish(outcomes.TIMEOUT, f'time limit {self.time_limit:.0f}s reached')
            return
        if self.last_scan_time is None or now - self.last_scan_time > self.sensor_timeout:
            self._finish(outcomes.SIMULATION_ERROR, 'LiDAR data stale (sensor failure)')
            return
        if not self.nav.server_is_ready():
            self.nav_down_since = self.nav_down_since or now
            if now - self.nav_down_since > self.nav_unavailable_timeout:
                self._finish(outcomes.SIMULATION_ERROR, 'Nav2 action server unavailable')
            return
        self.nav_down_since = None

        if self.exit_estimate is not None and self.state != EXIT_APPROACH:
            self._begin_exit_approach(now)
            return
        if self.state == EXIT_APPROACH:
            self._monitor_exit(now)
            return
        if self.state == NAVIGATING:
            self._monitor_goal(now)
        if self.state == NAVIGATING and self._replan_due(now):
            self._replan(now)
        if self.state == EXPLORING:
            self._select_and_send(now)

    def _check_ready(self, now: float) -> None:
        missing = []
        if self.grid is None:
            missing.append('map')
        if self.pose is None:
            missing.append('map->base_footprint TF')
        if not self.nav.server_is_ready():
            missing.append('navigate_to_pose server')
        if self.visual_search and not self.spin_client.server_is_ready():
            missing.append('spin server')
        if not missing:
            self.state = EXPLORING
            self.t_ready = now
            self.get_logger().info(f'system ready at t={now:.1f}s: exploration started')
        elif time.time() - self.wall_start > self.startup_timeout:
            self._finish(outcomes.SIMULATION_ERROR, 'startup timeout waiting for ' + ', '.join(missing))

    # ===================================================================== evaluation
    def _evaluate(self, now: float) -> Optional[List[Candidate]]:
        """Return ranked candidates, or None if the robot is not localised in free space."""
        grid = self.grid
        trav = compute_traversability(grid, self.eval_params.robot_radius,
                                      self.gap_fill_cells, self.border_cells)
        cell = grid.world_to_cell(self.pose[0], self.pose[1])
        if not grid.in_bounds(*cell) or not trav.mask[cell]:
            snapped = nearest_cell_in_mask(trav.mask, cell, int(0.4 / grid.resolution))
            if snapped is None:
                self.get_logger().warn('robot cell not traversable in SLAM map; skipping evaluation')
                return None
            cell = snapped
        field = GeodesicField(trav, grid.resolution, cell, self.clearance_penalty,
                              self.eval_params.robot_radius, self.eval_params.safe_clearance)
        frontiers = detect_frontiers(grid, self.min_frontier_size, trav.mask)
        info_grid = grid
        self.last_unseen = None
        if self.visual_search:
            # camera-unseen regions of known-free, traversable floor ("visual frontiers")
            unseen = trav.mask & grid.free & ~self.coverage.mask(grid)
            for i, (cells, tcell) in enumerate(unseen_regions(unseen, np.zeros_like(unseen),
                                                              self.min_unseen_cells)):
                frontiers.append(Frontier(id=100000 + i, cells=cells,
                                          centroid=grid.cell_to_world(*cells.mean(axis=0)),
                                          target_cell=tcell, target=grid.cell_to_world(*tcell),
                                          kind='visual'))
            info_grid = info_grid_with_unseen(grid, unseen)   # unseen floor counts as information
            self.last_unseen = unseen
        rejected: list = []
        cands = evaluate_candidates(frontiers, info_grid, trav, field, self.memory,
                                    self.eval_params, rejected)
        # Nav2's global costmap follows the SLAM map with a lag; a target it does not
        # cover yet makes the planner abort instantly. Defer such targets (transient).
        self.costmap_lag = False
        if self.costmap_bounds is not None:
            x0, x1, y0, y1 = self.costmap_bounds
            m = self.costmap_margin
            inside = [c for c in cands
                      if x0 + m <= c.target[0] <= x1 - m and y0 + m <= c.target[1] <= y1 - m]
            for c in cands:
                if c not in inside:
                    rejected.append((c.frontier, 'outside Nav2 costmap (not updated yet)'))
            self.costmap_lag = len(inside) < len(cands)
            cands = inside
        ranked = self.selector.rank(cands)
        self.last_rejected = [(f.target, f.size, f'{f.kind}: {why}') for f, why in rejected]
        self.last_unreachable = [f for f, why in rejected
                                 if why == 'unreachable' and f.kind == 'map'
                                 and not self.memory.is_blacklisted(*f.target)]
        self.last_frontiers = frontiers
        self.last_candidates = ranked
        self.last_eval_time = now
        self.last_eval_map = self.map_count
        self._publish_markers()
        return ranked

    def _replan_due(self, now: float) -> bool:
        return (now - self.last_eval_time >= self.replan_period
                and self.map_count != self.last_eval_map)

    def _frontier_still_open(self, target: Tuple[float, float], kind: str) -> bool:
        """True if a frontier of the same kind still has a cell near ``target``."""
        r2 = self.resolve_radius ** 2
        for f in self.last_frontiers:
            if f.kind != kind:
                continue
            xs = self.grid.origin_x + (f.cells[:, 1] + 0.5) * self.grid.resolution
            ys = self.grid.origin_y + (f.cells[:, 0] + 0.5) * self.grid.resolution
            if (((xs - target[0]) ** 2 + (ys - target[1]) ** 2) <= r2).any():
                return True
        return False

    def _select_and_send(self, now: float) -> None:
        ranked = self._evaluate(now)
        if ranked is None:
            return
        if ranked:
            self.empty_checks = 0
            self._send_frontier_goal(ranked[0], now)
            return
        # No candidate: confirm over several map updates before declaring completion.
        if self.costmap_lag:
            return      # targets exist but Nav2 cannot see them yet: not "complete"
        if self.last_unreachable:
            # My reachability graph only crosses (nearly) known-free cells; Nav2 also plans
            # through unknown space. Before declaring completion, let Nav2 try. Failures are
            # recorded and blacklisted as usual, so this terminates. (A v1 campaign run
            # stopped at 51 % coverage with one "unreachable" frontier left.)
            f = min(self.last_unreachable,
                    key=lambda fr: math.hypot(fr.target[0] - self.pose[0], fr.target[1] - self.pose[1]))
            x, y = f.target
            d = math.hypot(x - self.pose[0], y - self.pose[1])
            self.get_logger().info(f'[{self.strategy}] goal #{self.goal_seq + 1}: no reachable '
                                   f'candidate; asking Nav2 to try graph-unreachable frontier '
                                   f'({x:.2f}, {y:.2f})')
            self.counters['fallback_goals'] = self.counters.get('fallback_goals', 0) + 1
            self._send('frontier', x, y, math.atan2(y - self.pose[1], x - self.pose[0]),
                       2.0 * d, -math.inf, now)
            self.goal.target_kind = 'map'
            self.state = NAVIGATING
            return
        if now - self.last_empty_check >= 1.0:
            self.empty_checks += 1
            self.last_empty_check = now
        if self.empty_checks >= self.completion_checks:
            remaining = len(self.last_frontiers)
            details = '; '.join(f'({x:.2f}, {y:.2f}) size={n}: {why}'
                                for (x, y), n, why in self.last_rejected)
            reason = ('no frontiers remain' if remaining == 0 else
                      f'{remaining} frontier(s) remain but none is a valid target [{details}]')
            self._finish(outcomes.EXPLORATION_COMPLETE_EXIT_NOT_FOUND, reason)

    def _replan(self, now: float) -> None:
        goal = self.goal
        ranked = self._evaluate(now)
        if ranked is None or goal is None:
            return
        if not self._frontier_still_open(goal.target, goal.target_kind):
            self.get_logger().info(f'target ({goal.target[0]:.2f}, {goal.target[1]:.2f}) resolved '
                                   'by new observations; reselecting')
            self._close_goal(mem.RESOLVED, cancel=True)
            self.state = EXPLORING
            if ranked:
                self._send_frontier_goal(ranked[0], now)
            return
        if (self.strategy == 'random' or goal.kind == 'look' or not ranked
                or now - goal.sent < self.min_commit_time):
            return
        best = ranked[0]
        if distance(best.target, goal.target) <= self.memory.region_radius:
            return
        current = min(ranked, key=lambda c: distance(c.target, goal.target))
        if distance(current.target, goal.target) > self.resolve_radius:
            current_pref = goal.preference
        else:
            current_pref = self.selector.preference(current)
        margin = (self.switch_margin_distance if self.strategy == 'nearest'
                  else self.switch_margin_score)
        if self.selector.preference(best) > current_pref + margin:
            self.get_logger().info(
                f'preempting goal for better frontier ({best.target[0]:.2f}, {best.target[1]:.2f}): '
                f'{self.selector.preference(best):.2f} > {current_pref:.2f} + {margin}')
            self._close_goal(mem.PREEMPTED, cancel=True)
            self._send_frontier_goal(best, now)

    # ===================================================================== goals
    def _make_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        _, _, qz, qw = quaternion_from_yaw(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        return goal

    def _send(self, kind: str, x: float, y: float, yaw: float, travel_cost: float,
              preference: float, now: float, spin_delta: Optional[float] = None) -> None:
        self.goal_seq += 1
        seq = self.goal_seq
        timeout = (self.goal_timeout_base + self.goal_timeout_per_meter * travel_cost
                   if spin_delta is None else 15.0)
        self.goal = ActiveGoal(seq=seq, kind=kind, target=(x, y), sent=now, timeout=timeout,
                               preference=preference, travel_cost=travel_cost)
        self.stuck.reset()
        self.counters['goals_sent'] += 1
        is_nav = spin_delta is None
        # replaced NavigateToPose goals are preempted by Nav2; anything else is cancelled
        keep = [(k, h) for k, h in self._pending_cancel if not (is_nav and k != 'look')]
        self._pending_cancel = []
        for _, handle in keep:
            handle.cancel_goal_async()
        self.active_nav_seq = seq if is_nav else None
        if spin_delta is None:
            future = self.nav.send_goal_async(self._make_goal(x, y, yaw),
                                              feedback_callback=partial(self._on_feedback, seq))
        else:
            goal = Spin.Goal()
            goal.target_yaw = float(spin_delta)
            goal.time_allowance = DurationMsg(sec=12)
            future = self.spin_client.send_goal_async(goal)
        future.add_done_callback(partial(self._on_goal_response, seq))

    def _send_frontier_goal(self, cand: Candidate, now: float) -> None:
        x, y = cand.target
        yaw = math.atan2(y - self.pose[1], x - self.pose[0])
        kind = cand.frontier.kind
        feats = ' '.join(f'{k}={v:.2f}' for k, v in cand.features.items())
        if kind == 'visual' and cand.travel_cost < self.look_distance:
            # Unseen region right next to the robot: turn the camera toward it.
            cx, cy = cand.frontier.centroid
            delta = math.remainder(math.atan2(cy - self.pose[1], cx - self.pose[0]) - self.pose[2],
                                   2 * math.pi)
            if abs(delta) < 0.5:
                delta = math.copysign(0.5, delta if delta != 0 else 1.0)
            self.get_logger().info(f'[{self.strategy}] goal #{self.goal_seq + 1}: look at unseen '
                                   f'region ({cx:.2f}, {cy:.2f}), spin {math.degrees(delta):.0f} deg')
            self._send('look', x, y, yaw, 0.0, self.selector.preference(cand), now, spin_delta=delta)
            self.goal.target_kind = kind
            self.state = NAVIGATING
            return
        self.get_logger().info(
            f'[{self.strategy}] goal #{self.goal_seq + 1} ({kind}) -> ({x:.2f}, {y:.2f}) '
            f'cost={cand.travel_cost:.2f}m info={cand.info_gain} size={cand.frontier.size} '
            f'score={cand.score:.2f} | {feats} | {len(self.last_candidates)} candidates')
        self._send('frontier', x, y, yaw, cand.travel_cost, self.selector.preference(cand), now)
        self.goal.target_kind = kind
        self.state = NAVIGATING

    def _on_goal_response(self, seq: int, future) -> None:
        handle = future.result()
        if self.goal is None or seq != self.goal.seq:
            # Stale goal accepted after we moved on. If a newer NavigateToPose is active,
            # Nav2 has already preempted this one; cancelling it could abort the new goal.
            if handle is not None and handle.accepted and self.active_nav_seq is None:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'goal #{seq} rejected by Nav2')
            self.counters['rejected'] += 1
            self._close_goal(mem.FAILED)
            self._after_goal_end()
            return
        self.goal.handle = handle
        handle.get_result_async().add_done_callback(partial(self._on_result, seq))

    def _on_feedback(self, seq: int, msg) -> None:
        if self.goal is not None and seq == self.goal.seq:
            self.goal.recoveries = max(self.goal.recoveries, int(msg.feedback.number_of_recoveries))

    def _on_result(self, seq: int, future) -> None:
        if self.goal is None or seq != self.goal.seq or self.state == FINISHED:
            return
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._close_goal(mem.REACHED)
        elif status == GoalStatus.STATUS_CANCELED:
            self._close_goal(mem.PREEMPTED)
        else:
            self.get_logger().warn(f'goal #{seq} ended with status {status} (Nav2 abort)')
            self._close_goal(mem.FAILED)
        self._after_goal_end()

    def _after_goal_end(self) -> None:
        if self.state == NAVIGATING:
            self.state = EXPLORING
        if self.consecutive_failures >= self.max_consecutive_failures:
            self._finish(outcomes.NAVIGATION_FAILURE,
                         f'{self.consecutive_failures} consecutive navigation failures')

    def _close_goal(self, outcome: str, cancel: bool = False) -> None:
        goal = self.goal
        if goal is None:
            return
        self.goal = None
        if cancel and goal.handle is not None:
            # Deferred: if a new NavigateToPose follows in this tick, Nav2 preempts the old
            # goal itself. Cancelling it explicitly right before sending a new one made Nav2
            # abort the *new* goal (seen in pilot logs as "failed after 0.0s").
            self._pending_cancel.append((goal.kind, goal.handle))
        now = self.now()
        if goal.kind in ('frontier', 'look'):
            self.memory.record(goal.target[0], goal.target[1], outcome, now)
        self.counters[outcome] = self.counters.get(outcome, 0) + 1
        self.counters['recoveries'] += goal.recoveries
        if outcome in mem.FAILURE_OUTCOMES:
            self.consecutive_failures += 1
            if goal.kind == 'exit':
                self.exit_failures += 1
        elif outcome == mem.REACHED:
            self.consecutive_failures = 0
        self.goal_log.append({'seq': goal.seq, 'kind': goal.kind, 'target_kind': goal.target_kind,
                              'x': round(goal.target[0], 3),
                              'y': round(goal.target[1], 3), 'outcome': outcome,
                              'duration': round(now - goal.sent, 2),
                              'planned_cost': round(goal.travel_cost, 2),
                              'recoveries': goal.recoveries})
        self.get_logger().info(f'goal #{goal.seq} ({goal.kind}) closed: {outcome} '
                               f'after {now - goal.sent:.1f}s, recoveries={goal.recoveries}')

    def _monitor_goal(self, now: float) -> None:
        goal = self.goal
        if goal is None:
            self.state = EXPLORING
            return
        if now - goal.sent > goal.timeout:
            self.get_logger().warn(f'goal #{goal.seq} timed out after {goal.timeout:.0f}s')
            self._close_goal(mem.TIMED_OUT, cancel=True)
            self._after_goal_end()
        elif self.stuck.is_stuck(now):
            self.get_logger().warn(f'goal #{goal.seq}: robot stuck (moved < '
                                   f'{self.stuck.min_displacement}m in {self.stuck.window}s)')
            self._close_goal(mem.STUCK, cancel=True)
            self._after_goal_end()

    # ===================================================================== exit handling
    def _on_exit(self, msg: PoseStamped) -> None:
        if self.state == FINISHED:
            return
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(f'ignoring exit detection in frame {msg.header.frame_id}')
            return
        q = msg.pose.orientation
        est = (msg.pose.position.x, msg.pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if self.exit_estimate is None:
            self.exit_detect_time = self.now()
            self.get_logger().info(f'EXIT MARKER DETECTED at ({est[0]:.2f}, {est[1]:.2f}); '
                                   'stopping exploration')
        self.exit_estimate = est

    def _exit_goal(self) -> Tuple[float, float, float]:
        dx, dy, n = self.exit_estimate
        x = dx + self.exit_approach_offset * math.cos(n)
        y = dy + self.exit_approach_offset * math.sin(n)
        if self.grid is not None:
            trav = compute_traversability(self.grid, self.eval_params.robot_radius,
                                          self.gap_fill_cells, self.border_cells)
            cell = self.grid.world_to_cell(x, y)
            if not (self.grid.in_bounds(*cell) and trav.mask[cell]):
                snapped = nearest_cell_in_mask(trav.mask, cell, int(0.4 / self.grid.resolution))
                if snapped is not None:
                    x, y = self.grid.cell_to_world(*snapped)
        return x, y, n + math.pi

    def _begin_exit_approach(self, now: float) -> None:
        if self.goal is not None:
            self._close_goal(mem.PREEMPTED, cancel=True)
        self.state = EXIT_APPROACH
        self.exit_attempts = 0
        self._send_exit_goal(now)

    def _send_exit_goal(self, now: float) -> None:
        x, y, yaw = self._exit_goal()
        self.exit_attempts += 1
        cost = math.hypot(x - self.pose[0], y - self.pose[1])
        self.get_logger().info(f'exit approach attempt {self.exit_attempts}: goal ({x:.2f}, {y:.2f})')
        self._send('exit', x, y, yaw, cost, 0.0, now)

    def _monitor_exit(self, now: float) -> None:
        dx, dy, n = self.exit_estimate
        dist = math.hypot(self.pose[0] - dx, self.pose[1] - dy)
        # Arrival = standing in front of the marker: close along its normal AND roughly
        # centred on it. A plain distance test was satisfied while approaching along the
        # wall from a neighbouring cell (caught by ground-truth verification in a pilot).
        ox, oy = self.pose[0] - dx, self.pose[1] - dy
        along_normal = ox * math.cos(n) + oy * math.sin(n)
        lateral = -ox * math.sin(n) + oy * math.cos(n)
        if (0.0 < along_normal <= self.exit_arrival_distance
                and abs(lateral) <= self.exit_lateral_tolerance):
            # Turn to face the exit before reporting (Nav2 accepts any final heading):
            # a last visual confirmation, and the camera ends on the exit sign.
            if (self.goal is not None and self.goal.kind == 'face'
                    and now - self.goal.sent <= self.goal.timeout):
                return
            facing = math.remainder(math.atan2(dy - self.pose[1], dx - self.pose[0]) - self.pose[2],
                                    2 * math.pi)
            if not self.exit_faced and abs(facing) > 0.35:
                self.exit_faced = True
                if self.goal is not None:
                    self._close_goal(mem.PREEMPTED, cancel=True)
                self._send('face', self.pose[0], self.pose[1], self.pose[2], 0.0, 0.0, now,
                           spin_delta=facing)
                return
            self._finish(outcomes.EXIT_REACHED,
                         f'robot in front of the detected exit marker ({along_normal:.2f} m out, '
                         f'{lateral:+.2f} m lateral)')
            return
        goal = self.goal
        if goal is not None and (now - goal.sent > goal.timeout or self.stuck.is_stuck(now)):
            self._close_goal(mem.TIMED_OUT, cancel=True)
            goal = None
        elif goal is not None and math.hypot(goal.target[0] - self._exit_goal()[0],
                                             goal.target[1] - self._exit_goal()[1]) > 0.3:
            self._close_goal(mem.PREEMPTED, cancel=True)   # estimate refined: re-target
            goal = None
        if goal is None:
            # Only Nav2 failures count against the exit approach. Reaching a goal and then
            # re-targeting because the marker estimate was refined is progress, not failure
            # (a v1 campaign run was declared failed 0.52 m from the door this way); the
            # total number of approach goals is still capped.
            if (self.exit_failures >= self.max_exit_attempts
                    or self.exit_attempts >= self.max_exit_goals):
                self._finish(outcomes.NAVIGATION_FAILURE,
                             f'exit detected but not reached ({self.exit_failures} failed of '
                             f'{self.exit_attempts} approach goals, last distance {dist:.2f} m)')
            else:
                self._send_exit_goal(now)

    # ===================================================================== termination
    def _finish(self, outcome: str, reason: str) -> None:
        if self.state == FINISHED:
            return
        if self.goal is not None:
            self._close_goal(mem.PREEMPTED, cancel=True)
        prev = self.state
        self.state = FINISHED
        now = self.now()
        self.outcome = {
            'outcome': outcome,
            'reason': reason,
            'state_before_finish': prev,
            'strategy': self.strategy,
            'sim_time_ready': self.t_ready,
            'sim_time_finish': now,
            'exploration_time': None if self.t_ready is None else now - self.t_ready,
            'first_map_sim_time': self.first_map_time,
            'exit_detect_sim_time': self.exit_detect_time,
            'exit_estimate_map': (None if self.exit_estimate is None else
                                  {'x': self.exit_estimate[0], 'y': self.exit_estimate[1],
                                   'normal_yaw': self.exit_estimate[2]}),
            'distance_estimate_slam': self.distance_est,
            'counters': self.counters,
            'blacklisted_regions': len(self.memory.blacklisted_points()),
            'goals': self.goal_log,
        }
        self.outcome_pub.publish(String(data=json.dumps(self.outcome)))
        self._stop_ticks = 10
        log = self.get_logger().info if outcome == outcomes.EXIT_REACHED else self.get_logger().warn
        log(f'FINISHED: {outcome} ({reason}); goals={self.counters}')

    def _stop_robot_tick(self) -> None:
        if self._stop_ticks > 0:
            self.cmd_pub.publish(Twist())
            self._stop_ticks -= 1

    # ===================================================================== reporting
    def _publish_status(self) -> None:
        now = self.now()
        status = {
            'state': self.state,
            'strategy': self.strategy,
            'sim_time': now,
            'elapsed': None if self.t_ready is None else now - self.t_ready,
            'frontiers': len(self.last_frontiers),
            'candidates': len(self.last_candidates),
            'current_goal': None if self.goal is None else list(self.goal.target),
            'exit_detected': self.exit_estimate is not None,
            'camera_seen_cells': len(self.coverage),
            'distance_estimate_slam': round(self.distance_est, 3),
            'counters': self.counters,
            'outcome': None if self.outcome is None else self.outcome['outcome'],
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def _publish_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        cells = Marker()
        cells.header.frame_id = self.global_frame
        cells.header.stamp = stamp
        cells.ns, cells.id, cells.type = 'frontier_cells', 0, Marker.POINTS
        cells.scale.x = cells.scale.y = self.grid.resolution
        cells.color = ColorRGBA(r=0.35, g=0.45, b=1.0, a=0.9)   # blue; cyan = travelled path
        cells.pose.orientation.w = 1.0
        for f in self.last_frontiers:
            if f.kind != 'map':
                continue
            for r, c in f.cells[::2]:
                x, y = self.grid.cell_to_world(r, c)
                cells.points.append(Point(x=x, y=y, z=0.02))
        arr.markers.append(cells)
        if self.last_unseen is not None and self.last_unseen.any():
            unseen = Marker()
            unseen.header = cells.header
            unseen.ns, unseen.id, unseen.type = 'unseen_cells', 0, Marker.POINTS
            unseen.scale.x = unseen.scale.y = self.grid.resolution * 0.5
            unseen.color = ColorRGBA(r=0.9, g=0.3, b=0.9, a=0.22)   # faint: context, not clutter
            unseen.pose.orientation.w = 1.0
            rows, cols = np.nonzero(self.last_unseen)
            for r, c in zip(rows[::6], cols[::6]):
                x, y = self.grid.cell_to_world(r, c)
                unseen.points.append(Point(x=x, y=y, z=0.01))
            arr.markers.append(unseen)
        for rank, cand in enumerate(self.last_candidates[:15]):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp
            m.ns, m.id, m.type = 'candidates', rank, Marker.SPHERE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = cand.target[0], cand.target[1], 0.1
            m.pose.orientation.w = 1.0
            s = 0.18 if rank == 0 else 0.11
            m.scale.x = m.scale.y = m.scale.z = s
            if rank == 0:
                m.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0)
            elif cand.frontier.kind == 'visual':
                m.color = ColorRGBA(r=0.9, g=0.2, b=0.9, a=0.8)
            else:
                m.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.8)
            arr.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns, t.id, t.type = 'scores', rank, Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y, t.pose.position.z = cand.target[0], cand.target[1], 0.35
            t.pose.orientation.w = 1.0
            t.scale.z = 0.12
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            t.text = f'{cand.score:.2f}' if self.strategy == 'weighted' else f'{cand.travel_cost:.1f}m'
            arr.markers.append(t)
        for i, (x, y) in enumerate(self.memory.blacklisted_points()):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp
            m.ns, m.id, m.type = 'blacklist', i, Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.02
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2 * self.memory.region_radius
            m.scale.z = 0.02
            m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.35)
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
