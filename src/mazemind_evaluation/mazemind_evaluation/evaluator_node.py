"""MazeMind evaluator: the only node allowed to read ground truth.

Subscribes:
  /ground_truth/odom        true robot pose (Gazebo OdometryPublisher)
  /ground_truth/contacts    chassis contact sensor
  /map                      SLAM map (for coverage)
  /odom                     wheel odometry (distance comparison)
  /mazemind/outcome         explorer's terminal outcome
  /mazemind/explorer/status explorer progress
  /mazemind/exit/detection  robot's exit-marker estimate (accuracy check)
  TF map->base_footprint    SLAM pose (localisation error)

Publishes /mazemind/metrics (JSON) and /mazemind/metrics/markers (RViz text
panel), and writes ``<results_dir>/runs/<run_id>.json`` when the run ends.
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from mazemind_evaluation.metrics import (ContactCounter, CoverageEvaluator, DistanceAccumulator,
                                         NearCollisionTracker, Pose2D, map_to_world, summarize)
from mazemind_simulation.maze import Maze

EXIT_REACHED = 'EXIT_REACHED'


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class EvaluatorNode(Node):

    def __init__(self) -> None:
        super().__init__('evaluator')
        p = self.declare_parameter
        gt_file = p('ground_truth_file', '').value
        self.run_id = p('run_id', 'manual').value
        self.strategy = p('strategy', 'unknown').value
        self.results_dir = os.path.expanduser(p('results_dir', '~/.mazemind/results').value)
        self.time_limit = float(p('time_limit', 630.0).value)
        self.body_radius = float(p('body_radius', 0.14).value)
        near_threshold = float(p('near_collision_threshold', 0.05).value)
        self.exit_verify_distance = float(p('exit_verify_distance', 1.0).value)
        if not gt_file or not os.path.exists(gt_file):
            raise RuntimeError(f'ground_truth_file not found: {gt_file!r}')
        self.gt_file = gt_file
        self.maze = Maze.load_json(gt_file)
        gt = self.maze.to_dict()
        self.start = Pose2D(gt['start']['x'], gt['start']['y'], gt['start']['yaw'])
        self.door = gt['exit']['door']
        self.gt_summary = {k: gt[k] for k in ('width', 'height', 'solution_length_m', 'dead_ends')}

        self.dist_gt = DistanceAccumulator()
        self.dist_odom = DistanceAccumulator()
        self.contacts = ContactCounter()
        self.near = NearCollisionTracker(self.maze.wall_boxes(), self.body_radius, near_threshold)
        self.coverage_eval: Optional[CoverageEvaluator] = None
        self.coverage = 0.0
        self.coverage_trace: List[List[float]] = []
        self.loc_errors: List[float] = []
        self.gt_pose: Optional[Pose2D] = None
        self.map_msg: Optional[OccupancyGrid] = None
        self.first_map_time: Optional[float] = None
        self.status: dict = {}
        self.outcome: Optional[dict] = None
        self.exit_estimate: Optional[dict] = None
        self.exit_first_seen: Optional[float] = None
        self.finished = False
        self.wall_start = time.time()
        self.sim_start: Optional[float] = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_gt, 50)
        self.create_subscription(Odometry, '/odom', self._on_odom, 50)
        self.create_subscription(Contacts, '/ground_truth/contacts', self._on_contacts, 50)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(String, '/mazemind/outcome', self._on_outcome, latched)
        self.create_subscription(String, '/mazemind/explorer/status', self._on_status, 10)
        self.create_subscription(PoseStamped, '/mazemind/exit/detection', self._on_exit, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.metrics_pub = self.create_publisher(String, '/mazemind/metrics', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mazemind/metrics/markers', 10)
        self.create_timer(1.0, self._periodic)
        self.get_logger().info(f'evaluator: run_id={self.run_id} strategy={self.strategy} '
                               f'ground truth={os.path.basename(gt_file)}')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ callbacks
    def _on_gt(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        self.gt_pose = Pose2D(pos.x, pos.y, yaw_of(msg.pose.pose.orientation))
        if self.finished:
            return
        self.dist_gt.update(pos.x, pos.y)
        self.near.update(pos.x, pos.y)

    def _on_odom(self, msg: Odometry) -> None:
        if not self.finished:
            self.dist_odom.update(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_contacts(self, msg: Contacts) -> None:
        if self.finished:
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9 or self.now()
        self.contacts.update(stamp, [(c.collision1.name, c.collision2.name) for c in msg.contacts])

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        if self.first_map_time is None:
            self.first_map_time = self.now()
        if self.coverage_eval is None or abs(self.coverage_eval.resolution - msg.info.resolution) > 1e-6:
            self.coverage_eval = CoverageEvaluator(self.maze, self.start, msg.info.resolution)

    def _on_status(self, msg: String) -> None:
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    def _on_outcome(self, msg: String) -> None:
        if self.outcome is None:
            self.outcome = json.loads(msg.data)
            self.get_logger().info(f"outcome received: {self.outcome['outcome']}")
            self._finish()

    def _on_exit(self, msg: PoseStamped) -> None:
        wx, wy = map_to_world(self.start, msg.pose.position.x, msg.pose.position.y)
        if self.exit_first_seen is None:
            self.exit_first_seen = self.now()
        self.exit_estimate = {'map_x': msg.pose.position.x, 'map_y': msg.pose.position.y,
                              'world_x': wx, 'world_y': wy,
                              'error_m': math.hypot(wx - self.door['x'], wy - self.door['y'])}

    # ------------------------------------------------------------------ periodic
    def _update_coverage(self) -> None:
        if self.map_msg is None or self.coverage_eval is None:
            return
        info = self.map_msg.info
        data = np.asarray(self.map_msg.data, dtype=np.int16).reshape(info.height, info.width)
        self.coverage = self.coverage_eval.coverage(data, info.resolution, info.origin.position.x,
                                                    info.origin.position.y)
        self.coverage_trace.append([round(self.now(), 2), round(self.coverage, 4)])

    def _update_localisation_error(self) -> None:
        if self.gt_pose is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return
        wx, wy = map_to_world(self.start, tf.transform.translation.x, tf.transform.translation.y)
        self.loc_errors.append(math.hypot(wx - self.gt_pose.x, wy - self.gt_pose.y))

    def _periodic(self) -> None:
        if self.finished:
            return
        if self.sim_start is None and self.now() > 0:
            self.sim_start = self.now()
        self._update_coverage()
        self._update_localisation_error()
        self._publish_live()
        if self.sim_start is not None and self.now() - self.sim_start > self.time_limit:
            self.get_logger().warn('evaluator watchdog: no outcome before time limit')
            self._finish(watchdog='TIMEOUT')
        elif time.time() - self.wall_start > 3 * self.time_limit + 300:
            self.get_logger().error('evaluator watchdog: wall-clock limit (simulation stalled?)')
            self._finish(watchdog='SIMULATION_ERROR')

    def _live_metrics(self) -> dict:
        counters = self.status.get('counters', {})
        failed = sum(counters.get(k, 0) for k in ('failed', 'timed_out', 'stuck'))
        return {
            'run_id': self.run_id, 'strategy': self.strategy,
            'state': self.status.get('state', 'starting'),
            'elapsed': self.status.get('elapsed'),
            'coverage': round(self.coverage, 4),
            'distance_m': round(self.dist_gt.total, 3),
            'goals_sent': counters.get('goals_sent', 0), 'goals_failed': failed,
            'recoveries': counters.get('recoveries', 0),
            'contact_events': self.contacts.events, 'near_collisions': self.near.events,
            'exit_detected': self.exit_estimate is not None,
        }

    def _publish_live(self) -> None:
        m = self._live_metrics()
        self.metrics_pub.publish(String(data=json.dumps(m)))
        elapsed = m['elapsed'] if m['elapsed'] is not None else 0.0
        lines = [f"strategy: {m['strategy']}   state: {m['state']}",
                 f"time {elapsed:5.0f} s   coverage {100 * m['coverage']:5.1f} %",
                 f"distance {m['distance_m']:5.1f} m   goals {m['goals_sent']} (failed {m['goals_failed']})",
                 f"recoveries {m['recoveries']}   contacts {m['contact_events']}   "
                 f"near-collisions {m['near_collisions']}",
                 'exit: DETECTED' if m['exit_detected'] else 'exit: not yet seen']
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = Time().to_msg()
        marker.ns, marker.id, marker.type = 'metrics', 0, Marker.TEXT_VIEW_FACING
        marker.pose.position.z = 0.9
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.12
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        marker.text = '\n'.join(lines)
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    # ------------------------------------------------------------------ result
    def _finish(self, watchdog: Optional[str] = None) -> None:
        if self.finished:
            return
        self._update_coverage()
        self._update_localisation_error()
        self.finished = True
        outcome = self.outcome or {}
        final = outcome.get('outcome') or watchdog or 'SIMULATION_ERROR'
        counters = outcome.get('counters') or self.status.get('counters', {})
        failed = sum(counters.get(k, 0) for k in ('failed', 'timed_out', 'stuck'))
        gt = self.gt_pose
        exit_cell = self.maze.exit_cell
        gt_cell = self.maze.world_to_cell(gt.x, gt.y) if gt else None
        dist_to_door = math.hypot(gt.x - self.door['x'], gt.y - self.door['y']) if gt else None
        verified = bool(final == EXIT_REACHED and gt_cell == tuple(exit_cell)
                        and dist_to_door is not None and dist_to_door <= self.exit_verify_distance)
        exploration_time = outcome.get('exploration_time')
        cfg = self.maze.config
        result = {
            'run_id': self.run_id,
            'strategy': self.strategy,
            'final_status': final,
            'reason': outcome.get('reason', 'evaluator watchdog' if watchdog else ''),
            'exit_success': verified,
            'exit_reported': final == EXIT_REACHED,
            'maze': {'seed': cfg.seed, 'rows': cfg.rows, 'cols': cfg.cols,
                     'cell_size': cfg.cell_size, 'loop_fraction': cfg.loop_fraction,
                     'ground_truth_file': os.path.basename(self.gt_file), **self.gt_summary},
            'time_to_first_map_s': self.first_map_time,
            'exploration_start_sim_s': outcome.get('sim_time_ready'),
            'exploration_time_s': exploration_time,
            'time_to_exit_s': exploration_time if final == EXIT_REACHED else None,
            'exit_detect_time_s': (None if outcome.get('exit_detect_sim_time') is None or
                                   outcome.get('sim_time_ready') is None else
                                   outcome['exit_detect_sim_time'] - outcome['sim_time_ready']),
            'distance_gt_m': self.dist_gt.total,
            'distance_odom_m': self.dist_odom.total,
            'distance_slam_estimate_m': outcome.get('distance_estimate_slam'),
            # distance travelled / length of the cell-centre shortest route from start to exit.
            # Can be < 1: the robot cuts corners and stops ~0.5 m before the door.
            'distance_ratio': (self.dist_gt.total / self.gt_summary['solution_length_m']
                               if final == EXIT_REACHED and self.gt_summary['solution_length_m'] > 0
                               else None),
            'coverage': self.coverage,
            'goals_sent': counters.get('goals_sent', 0),
            'goals_reached': counters.get('reached', 0),
            'goals_failed': failed,
            'goals_failed_detail': {k: counters.get(k, 0)
                                    for k in ('failed', 'timed_out', 'stuck', 'rejected')},
            'goals_preempted': counters.get('preempted', 0),
            'goals_resolved_early': counters.get('resolved', 0),
            'recoveries': counters.get('recoveries', 0),
            'blacklisted_regions': outcome.get('blacklisted_regions'),
            'collision_events': self.contacts.events,
            'contact_time_s': self.contacts.contact_time,
            'contact_partners': sorted(self.contacts.partners),
            'near_collision_events': self.near.events,
            'near_collision_threshold_m': self.near.threshold,
            'min_clearance_m': None if math.isinf(self.near.min_clearance) else self.near.min_clearance,
            'localization_error_m': summarize(self.loc_errors),
            'exit_detection': self.exit_estimate,
            'final_gt_pose': None if gt is None else {'x': gt.x, 'y': gt.y, 'yaw': gt.yaw,
                                                      'cell': list(gt_cell) if gt_cell else None},
            'final_distance_to_door_m': dist_to_door,
            'coverage_trace': self.coverage_trace,
            'goals': outcome.get('goals', []),
            'wall_time_s': time.time() - self.wall_start,
        }
        os.makedirs(os.path.join(self.results_dir, 'runs'), exist_ok=True)
        path = os.path.join(self.results_dir, 'runs', f'{self.run_id}.json')
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(result, fh, indent=2)
        self.get_logger().info(
            f'RESULT {final} exit_verified={verified} distance={self.dist_gt.total:.2f}m '
            f'coverage={self.coverage:.3f} goals={result["goals_sent"]} failed={failed} '
            f'contacts={self.contacts.events} near={self.near.events} -> {path}')
        self.create_timer(2.0, self._exit)

    def _exit(self) -> None:
        raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
