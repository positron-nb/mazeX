"""Record a side-by-side demo video from live simulation data.

Left:   Gazebo top-down camera (simulator ground-truth view of maze + robot).
Right:  the robot's own SLAM map with pose, frontier clusters, candidate
        targets, the Nav2 plan and the exit detection (what RViz shows).
Bottom: robot camera with the exit-detector annotation, and live metrics
        from /mazemind/metrics.

Every pixel comes from a ROS topic of the running simulation; nothing is
synthesised. Frames are sampled on simulated time (``sample_rate`` Hz) and
written at ``playback_fps``, so the video is time-lapsed by
playback_fps / sample_rate.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

PANEL = 640
STRIP = 220
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _title(img, text, y=26, scale=0.7):
    cv2.putText(img, text, (12, y), FONT, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, (12, y), FONT, scale, (255, 255, 255), 1, cv2.LINE_AA)


class MediaRecorder(Node):

    def __init__(self) -> None:
        super().__init__('media_recorder')
        p = self.declare_parameter
        self.output = os.path.expanduser(p('output', '~/.mazemind/media/demo.mp4').value)
        self.playback_fps = float(p('playback_fps', 16.0).value)
        self.timelapse = float(p('timelapse', 4.0).value)        # video seconds = sim seconds / timelapse
        self.sample_rate = self.playback_fps / self.timelapse
        self.hold_seconds = float(p('hold_seconds', 3.0).value)
        self.title = p('title', 'MazeMind').value
        # Display only: rotate the robot-map panel by the spawn heading (multiples of 90 deg)
        # so it lines up with the Gazebo top view. The robot's map frame starts at its spawn pose.
        self.display_quarter_turns = int(round(float(p('display_yaw', 0.0).value) / (math.pi / 2))) % 4
        self.bridge = CvBridge()
        self.overhead: Optional[np.ndarray] = None
        self.robot_cam: Optional[np.ndarray] = None
        self.map_msg: Optional[OccupancyGrid] = None
        self.frontiers: Optional[MarkerArray] = None
        self.plan: Optional[Path] = None
        self.scan: Optional[LaserScan] = None
        self.exit: Optional[PoseStamped] = None
        self.metrics: dict = {}
        self.outcome: Optional[dict] = None
        self.outcome_time: Optional[float] = None
        self.trail = []
        self.writer = None
        self.frames = 0
        self.closed = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, '/ground_truth/overhead_camera', self._on_overhead,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/mazemind/exit/debug_image', self._on_cam, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', lambda m: setattr(self, 'map_msg', m), latched)
        self.create_subscription(MarkerArray, '/mazemind/frontiers',
                                 lambda m: setattr(self, 'frontiers', m), 10)
        self.create_subscription(Path, '/plan', lambda m: setattr(self, 'plan', m), 10)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mazemind/exit/detection',
                                 lambda m: setattr(self, 'exit', m), 10)
        self.create_subscription(String, '/mazemind/metrics', self._on_metrics, 10)
        self.create_subscription(String, '/mazemind/outcome', self._on_outcome, latched)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self.sample_rate, self._tick)
        self.get_logger().info(f'recording to {self.output}')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_overhead(self, msg: Image) -> None:
        self.overhead = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _on_cam(self, msg: Image) -> None:
        self.robot_cam = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _on_metrics(self, msg: String) -> None:
        try:
            self.metrics = json.loads(msg.data)
        except ValueError:
            pass

    def _on_outcome(self, msg: String) -> None:
        if self.outcome is None:
            self.outcome = json.loads(msg.data)
            self.outcome_time = self.now()

    # ------------------------------------------------------------------ rendering
    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def _scan_points_map(self):
        scan = self.scan
        if scan is None:
            return []
        try:
            tf = self.tf_buffer.lookup_transform('map', scan.header.frame_id, Time())
        except TransformException:
            return []
        q, t = tf.transform.rotation, tf.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + scan.angle_increment * np.arange(r.size) + yaw
        ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        return list(zip(t.x + r[ok] * np.cos(a[ok]), t.y + r[ok] * np.sin(a[ok])))

    def _render_map(self) -> np.ndarray:
        canvas = np.full((PANEL, PANEL, 3), 40, np.uint8)
        msg = self.map_msg
        if msg is None:
            _title(canvas, 'SLAM map: waiting for /map')
            return canvas
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        img = np.full(data.shape + (3,), 128, np.uint8)
        img[(data >= 0) & (data < 25)] = (245, 245, 245)
        img[data >= 65] = (20, 20, 20)
        img = np.flipud(img)                          # row 0 = bottom of the map
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        scale = (PANEL - 40) / max(info.width, info.height)
        img = cv2.resize(img, (int(info.width * scale), int(info.height * scale)),
                         interpolation=cv2.INTER_NEAREST)
        x_off = (PANEL - img.shape[1]) // 2
        y_off = (PANEL - img.shape[0]) // 2 + 10
        canvas[y_off:y_off + img.shape[0], x_off:x_off + img.shape[1]] = img
        h_img = img.shape[0]

        def px(x, y):
            return (int(x_off + (x - ox) / res * scale),
                    int(y_off + h_img - (y - oy) / res * scale))

        if self.frontiers is not None:
            for m in self.frontiers.markers:
                if m.ns == 'frontier_cells':
                    for p in m.points:
                        cv2.circle(canvas, px(p.x, p.y), 2, (255, 200, 0), -1)
            for m in self.frontiers.markers:
                if m.ns == 'candidates':
                    color = (60, 230, 60) if m.id == 0 else (0, 150, 255)
                    cv2.circle(canvas, px(m.pose.position.x, m.pose.position.y),
                               9 if m.id == 0 else 5, color, -1)
        if self.plan is not None and len(self.plan.poses) > 1:
            pts = np.array([px(ps.pose.position.x, ps.pose.position.y) for ps in self.plan.poses])
            cv2.polylines(canvas, [pts], False, (80, 220, 80), 2, cv2.LINE_AA)
        if len(self.trail) > 1:
            cv2.polylines(canvas, [np.array([px(x, y) for x, y in self.trail])], False,
                          (230, 220, 30), 2, cv2.LINE_AA)            # travelled path (cyan)
        for x, y in self._scan_points_map():
            cv2.circle(canvas, px(x, y), 1, (0, 140, 255), -1)     # live LiDAR (orange)
        pose = self._robot_pose()
        if pose is not None:
            x, y, yaw = pose
            c = px(x, y)
            cv2.circle(canvas, c, int(0.14 / res * scale), (220, 90, 30), -1)
            tip = px(x + 0.25 * math.cos(yaw), y + 0.25 * math.sin(yaw))
            cv2.arrowedLine(canvas, c, tip, (255, 255, 255), 2, tipLength=0.4)
        if self.exit is not None:
            c = px(self.exit.pose.position.x, self.exit.pose.position.y)
            cv2.drawMarker(canvas, c, (0, 255, 0), cv2.MARKER_STAR, 26, 3)
        if self.display_quarter_turns:
            canvas = np.ascontiguousarray(np.rot90(canvas, self.display_quarter_turns))
        _title(canvas, 'Robot view: SLAM map, LiDAR, frontiers, plan, path')
        return canvas

    def _render_overhead(self) -> np.ndarray:
        if self.overhead is None:
            canvas = np.full((PANEL, PANEL, 3), 40, np.uint8)
            _title(canvas, 'Gazebo: waiting for overhead camera')
            return canvas
        img = cv2.resize(self.overhead, (PANEL, PANEL))
        _title(img, 'Gazebo simulation (top-down camera)')
        return img

    def _render_strip(self) -> np.ndarray:
        strip = np.full((STRIP, 2 * PANEL, 3), 25, np.uint8)
        if self.robot_cam is not None:
            cam = cv2.resize(self.robot_cam, (int(STRIP * 4 / 3) - 20, STRIP - 20))
            strip[10:10 + cam.shape[0], 10:10 + cam.shape[1]] = cam
        m = self.metrics
        elapsed = m.get('elapsed') or 0.0
        lines = [
            f"{self.title}   strategy: {m.get('strategy', '?')}   state: {m.get('state', '?')}",
            f"sim time {elapsed:6.1f} s    map coverage {100 * m.get('coverage', 0.0):5.1f} %"
            f"    distance {m.get('distance_m', 0.0):5.1f} m",
            f"nav goals {m.get('goals_sent', 0)}  failed {m.get('goals_failed', 0)}  "
            f"recoveries {m.get('recoveries', 0)}    contacts {m.get('contact_events', 0)}  "
            f"near-collisions {m.get('near_collisions', 0)}",
            'exit marker: DETECTED by camera + LiDAR' if m.get('exit_detected') else
            'exit marker: not yet detected',
        ]
        x0 = int(STRIP * 4 / 3) + 10
        for i, line in enumerate(lines):
            cv2.putText(strip, line, (x0, 40 + 38 * i), FONT, 0.62, (235, 235, 235), 1, cv2.LINE_AA)
        if self.outcome is not None:
            color = (60, 220, 60) if self.outcome['outcome'] == 'EXIT_REACHED' else (60, 160, 255)
            cv2.putText(strip, f"OUTCOME: {self.outcome['outcome']}", (x0, 40 + 38 * 4 + 8), FONT,
                        0.85, color, 2, cv2.LINE_AA)
        cv2.putText(strip, 'metrics: evaluator (ground truth); robot uses only its own sensors',
                    (x0, STRIP - 8), FONT, 0.45, (150, 150, 150), 1, cv2.LINE_AA)
        return strip

    # ------------------------------------------------------------------ loop
    def _tick(self) -> None:
        if self.closed:
            return
        pose = self._robot_pose()
        if pose is not None and (not self.trail or math.hypot(pose[0] - self.trail[-1][0],
                                                              pose[1] - self.trail[-1][1]) > 0.03):
            self.trail.append((pose[0], pose[1]))
        if self.overhead is None and self.map_msg is None:
            return
        frame = np.vstack([np.hstack([self._render_overhead(), self._render_map()]),
                           self._render_strip()])
        if self.writer is None:
            os.makedirs(os.path.dirname(self.output) or '.', exist_ok=True)
            self.raw_path = self.output + '.raw.mp4'
            self.writer = cv2.VideoWriter(self.raw_path, cv2.VideoWriter_fourcc(*'mp4v'),
                                          self.playback_fps, (frame.shape[1], frame.shape[0]))
        self.writer.write(frame)
        self.frames += 1
        if self.outcome_time is not None and self.now() - self.outcome_time >= self.hold_seconds:
            for _ in range(int(self.playback_fps * 2)):
                self.writer.write(frame)          # hold the final frame for 2 s
            cv2.imwrite(os.path.splitext(self.output)[0] + '_final.png', frame)
            self._close()

    def _close(self) -> None:
        if self.closed:
            return
        self.closed = True
        if self.writer is not None:
            self.writer.release()
            ffmpeg = shutil.which('ffmpeg')
            if ffmpeg:
                # re-encode to H.264 so browsers / LinkedIn / GitHub play it
                subprocess.run([ffmpeg, '-y', '-loglevel', 'error', '-i', self.raw_path,
                                '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23',
                                self.output], check=False)
                if os.path.exists(self.output):
                    os.remove(self.raw_path)
            else:
                os.replace(self.raw_path, self.output)
        self.get_logger().info(f'wrote {self.frames} frames to {self.output}')

    def destroy_node(self):
        self._close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MediaRecorder()
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
