"""Record a smooth 1920x1080 demo video from live simulation data.

Left:   Gazebo camera view of the maze (oblique 3D or top-down).
Right:  the robot's own SLAM map, RViz-style (LiDAR, frontiers, plan,
        travelled path, exit detection), rotated to the Gazebo orientation;
        below it the robot camera with the exit-detector box and live metrics.

Every pixel comes from a topic of the running simulation; nothing is
synthesised. Frames are sampled on *simulated* time at ``fps`` and streamed
to ffmpeg (H.264), so the main video plays at exactly 1x simulated time. If
the PC falls behind, the last frame is repeated to keep that timing (run with
a lower real_time_factor for perfectly smooth output). When the run ends, the
recorder also writes ``*_fast.mp4`` (``fast_factor`` x) and
``*_ending.mp4`` (the last ``ending_seconds`` at 1x).
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

W, H = 1920, 1080
LEFT = 1080                       # sim view is LEFT x LEFT
RIGHT_W = W - LEFT                # 840
HEADER = 64
MAP_SIZE = 760                    # square map canvas
STRIP_Y = HEADER + MAP_SIZE + 8   # 832
FONT = cv2.FONT_HERSHEY_SIMPLEX

# RViz-like palette (BGR)
UNKNOWN = (78, 72, 64)
FREE = (238, 238, 236)
OCCUPIED = (32, 32, 36)
TRAIL = (230, 200, 0)
PLAN = (40, 200, 40)
LIDAR = (0, 140, 255)
FRONTIER = (255, 115, 90)
BG = (34, 30, 28)


def _text(img, text, org, scale=0.6, color=(235, 235, 235), thick=1):
    cv2.putText(img, text, org, FONT, scale, (0, 0, 0), thick + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, color, thick, cv2.LINE_AA)


class MediaRecorder(Node):

    def __init__(self) -> None:
        super().__init__('media_recorder')
        p = self.declare_parameter
        self.output = os.path.expanduser(p('output', '~/.mazemind/media/demo.mp4').value)
        self.fps = float(p('fps', 30.0).value)
        self.fast_factor = float(p('fast_factor', 8.0).value)
        self.ending_seconds = float(p('ending_seconds', 35.0).value)
        self.hold_seconds = float(p('hold_seconds', 3.0).value)
        self.view = p('view', 'oblique').value           # oblique | top
        self.title = p('title', 'MazeMind').value
        # Display only: rotate the map panel by the spawn heading (multiples of 90 deg) so it
        # lines up with the Gazebo view. The robot's map frame starts at its spawn pose.
        self.quarter_turns = int(round(float(p('display_yaw', 0.0).value) / (math.pi / 2))) % 4
        self.bridge = CvBridge()
        self.sim_img: Optional[np.ndarray] = None
        self.robot_cam: Optional[np.ndarray] = None
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_cache = None
        self.frontiers: Optional[MarkerArray] = None
        self.plan: Optional[Path] = None
        self.trail: Optional[Path] = None
        self.scan: Optional[LaserScan] = None
        self.exit: Optional[PoseStamped] = None
        self.metrics: dict = {}
        self.outcome: Optional[dict] = None
        self.outcome_time: Optional[float] = None
        self.ffmpeg: Optional[subprocess.Popen] = None
        self.t0: Optional[float] = None
        self.frames = 0
        self.closed = False
        self.last_frame: Optional[np.ndarray] = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        sim_topic = ('/ground_truth/scene_camera' if self.view == 'oblique'
                     else '/ground_truth/overhead_camera')
        self.create_subscription(Image, sim_topic, self._on_sim, qos_profile_sensor_data)
        self.create_subscription(Image, '/mazemind/exit/debug_image', self._on_cam,
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(MarkerArray, '/mazemind/frontiers',
                                 lambda m: setattr(self, 'frontiers', m), 10)
        self.create_subscription(Path, '/plan', lambda m: setattr(self, 'plan', m), 10)
        self.create_subscription(Path, '/mazemind/trail', lambda m: setattr(self, 'trail', m), 1)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mazemind/exit/detection',
                                 lambda m: setattr(self, 'exit', m), 10)
        self.create_subscription(String, '/mazemind/metrics', self._on_metrics, 10)
        self.create_subscription(String, '/mazemind/outcome', self._on_outcome, latched)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self.fps, self._tick)
        self.get_logger().info(f'recording {W}x{H} @ {self.fps:.0f} fps (sim time) to {self.output}')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def _on_sim(self, msg: Image) -> None:
        self.sim_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _on_cam(self, msg: Image) -> None:
        self.robot_cam = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_cache = None

    def _on_metrics(self, msg: String) -> None:
        try:
            self.metrics = json.loads(msg.data)
        except ValueError:
            pass

    def _on_outcome(self, msg: String) -> None:
        if self.outcome is None:
            self.outcome = json.loads(msg.data)
            self.outcome_time = self.now()

    # ------------------------------------------------------------------ map panel
    def _map_base(self):
        """Rasterise the SLAM map once per update: returns (image, world->pixel function)."""
        if self.map_cache is not None:
            return self.map_cache
        info = self.map_msg.info
        data = np.asarray(self.map_msg.data, dtype=np.int16).reshape(info.height, info.width)
        img = np.empty(data.shape + (3,), np.uint8)
        img[:] = UNKNOWN
        img[(data >= 0) & (data < 25)] = FREE
        img[data >= 65] = OCCUPIED
        img = np.flipud(img)
        scale = (MAP_SIZE - 30) / max(info.width, info.height)
        img = cv2.resize(img, (max(1, int(info.width * scale)), max(1, int(info.height * scale))),
                         interpolation=cv2.INTER_NEAREST)
        canvas = np.empty((MAP_SIZE, MAP_SIZE, 3), np.uint8)
        canvas[:] = BG
        x_off = (MAP_SIZE - img.shape[1]) // 2
        y_off = (MAP_SIZE - img.shape[0]) // 2
        canvas[y_off:y_off + img.shape[0], x_off:x_off + img.shape[1]] = img
        res, ox, oy, h_img = info.resolution, info.origin.position.x, info.origin.position.y, img.shape[0]

        def px(x, y):
            return (int(round(x_off + (x - ox) / res * scale)),
                    int(round(y_off + h_img - (y - oy) / res * scale)))

        self.map_cache = (canvas, px, scale / res)
        return self.map_cache

    def _pose(self, frame: str):
        try:
            tf = self.tf_buffer.lookup_transform('map', frame, Time())
        except TransformException:
            return None
        q, t = tf.transform.rotation, tf.transform.translation
        return t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _render_map(self) -> np.ndarray:
        if self.map_msg is None:
            canvas = np.empty((MAP_SIZE, MAP_SIZE, 3), np.uint8)
            canvas[:] = BG
            return canvas
        base, px, ppm = self._map_base()
        canvas = base.copy()
        if self.frontiers is not None:
            for m in self.frontiers.markers:
                if m.ns == 'frontier_cells':
                    for p in m.points[::2]:
                        cv2.circle(canvas, px(p.x, p.y), 2, FRONTIER, -1)
        if self.trail is not None and len(self.trail.poses) > 1:
            pts = np.array([px(ps.pose.position.x, ps.pose.position.y) for ps in self.trail.poses])
            cv2.polylines(canvas, [pts], False, TRAIL, 3, cv2.LINE_AA)
        if self.plan is not None and len(self.plan.poses) > 1:
            pts = np.array([px(ps.pose.position.x, ps.pose.position.y) for ps in self.plan.poses])
            cv2.polylines(canvas, [pts], False, PLAN, 3, cv2.LINE_AA)
        scan, lidar = self.scan, self._pose(self.scan.header.frame_id) if self.scan else None
        if scan is not None and lidar is not None:
            r = np.asarray(scan.ranges, dtype=float)
            a = scan.angle_min + scan.angle_increment * np.arange(r.size) + lidar[2]
            ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
            for x, y in zip(lidar[0] + r[ok] * np.cos(a[ok]), lidar[1] + r[ok] * np.sin(a[ok])):
                cv2.circle(canvas, px(x, y), 2, LIDAR, -1)
        if self.frontiers is not None:
            for m in self.frontiers.markers:
                if m.ns == 'candidates':
                    chosen = m.id == 0
                    cv2.circle(canvas, px(m.pose.position.x, m.pose.position.y), 8 if chosen else 5,
                               (60, 230, 60) if chosen else (0, 165, 255), -1, cv2.LINE_AA)
        robot = self._pose('base_footprint')
        if robot is not None:
            c = px(robot[0], robot[1])
            rad = max(5, int(0.14 * ppm))
            cv2.circle(canvas, c, rad, (255, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(canvas, c, rad, (20, 20, 20), 2, cv2.LINE_AA)
            tip = px(robot[0] + 0.32 * math.cos(robot[2]), robot[1] + 0.32 * math.sin(robot[2]))
            cv2.arrowedLine(canvas, c, tip, (20, 20, 20), 2, cv2.LINE_AA, tipLength=0.45)
        if self.exit is not None:
            cv2.drawMarker(canvas, px(self.exit.pose.position.x, self.exit.pose.position.y),
                           (40, 255, 40), cv2.MARKER_STAR, 34, 3, cv2.LINE_AA)
        if self.quarter_turns:
            canvas = np.ascontiguousarray(np.rot90(canvas, self.quarter_turns))
        return canvas

    # ------------------------------------------------------------------ frame
    def _compose(self) -> np.ndarray:
        frame = np.empty((H, W, 3), np.uint8)
        frame[:] = BG
        if self.sim_img is not None:
            frame[:, :LEFT] = cv2.resize(self.sim_img, (LEFT, LEFT), interpolation=cv2.INTER_AREA)
        _text(frame, 'Gazebo simulation', (24, 44), 0.9, (255, 255, 255), 2)
        _text(frame, f'{self.title}', (24, H - 24), 0.7)

        x0 = LEFT
        _text(frame, "Robot's own map (SLAM) + plan", (x0 + 24, 44), 0.85, (255, 255, 255), 2)
        frame[HEADER:HEADER + MAP_SIZE, x0 + (RIGHT_W - MAP_SIZE) // 2:
              x0 + (RIGHT_W - MAP_SIZE) // 2 + MAP_SIZE] = self._render_map()

        cam_w, cam_h = 320, 240
        cy = STRIP_Y
        if self.robot_cam is not None:
            frame[cy:cy + cam_h, x0 + 16:x0 + 16 + cam_w] = cv2.resize(self.robot_cam, (cam_w, cam_h))
        m = self.metrics
        tx = x0 + 16 + cam_w + 24
        elapsed = m.get('elapsed') or 0.0
        rows = [
            (f"strategy: {m.get('strategy', '-')}", 0.62),
            (f"time  {elapsed:6.1f} s (sim)", 0.62),
            (f"distance  {m.get('distance_m', 0.0):6.1f} m", 0.62),
            (f"map coverage  {100 * m.get('coverage', 0.0):5.1f} %", 0.62),
            (f"goals {m.get('goals_sent', 0)}   failed {m.get('goals_failed', 0)}", 0.62),
            (f"wall contacts {m.get('contact_events', 0)}", 0.62),
        ]
        for i, (row, sc) in enumerate(rows):
            _text(frame, row, (tx, cy + 28 + 30 * i), sc)
        if self.outcome is not None:
            ok = self.outcome['outcome'] == 'EXIT_REACHED'
            _text(frame, 'EXIT REACHED' if ok else self.outcome['outcome'], (tx, cy + 228), 0.9,
                  (80, 255, 80) if ok else (80, 180, 255), 2)
        elif m.get('exit_detected'):
            _text(frame, 'exit sign detected', (tx, cy + 228), 0.75, (80, 255, 80), 2)
        return frame

    # ------------------------------------------------------------------ output
    def _open(self) -> None:
        os.makedirs(os.path.dirname(self.output) or '.', exist_ok=True)
        ffmpeg = shutil.which('ffmpeg')
        if ffmpeg is None:
            raise RuntimeError('ffmpeg not found (sudo apt install ffmpeg)')
        self.ffmpeg = subprocess.Popen(
            [ffmpeg, '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
             '-s', f'{W}x{H}', '-r', f'{self.fps}', '-i', '-', '-c:v', 'libx264',
             '-preset', 'veryfast', '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
             self.output], stdin=subprocess.PIPE)

    def _write(self, frame: np.ndarray, count: int = 1) -> None:
        data = frame.tobytes()
        for _ in range(count):
            self.ffmpeg.stdin.write(data)
        self.frames += count

    def _tick(self) -> None:
        if self.closed:
            return
        if self.sim_img is None and self.map_msg is None:
            return
        now = self.now()
        if self.ffmpeg is None:
            self._open()
            self.t0 = now
        frame = self._compose()
        # keep video time locked to sim time: repeat frames if callbacks were missed
        target = int((now - self.t0) * self.fps) + 1
        self._write(frame, max(1, target - self.frames))
        self.last_frame = frame
        if self.outcome_time is not None and now - self.outcome_time >= self.hold_seconds:
            self._write(frame, int(self.fps * 2))          # hold the final frame 2 s
            cv2.imwrite(os.path.splitext(self.output)[0] + '_final.png', frame)
            self._close()

    def _close(self) -> None:
        if self.closed:
            return
        self.closed = True
        if self.ffmpeg is None:
            return
        self.ffmpeg.stdin.close()
        self.ffmpeg.wait()
        self.get_logger().info(f'wrote {self.frames} frames ({self.frames / self.fps:.0f} s) '
                               f'to {self.output}')
        base = os.path.splitext(self.output)[0]
        ffmpeg = shutil.which('ffmpeg')
        # fast version: frame-dropping speed-up from a 30 fps source stays smooth
        subprocess.run([ffmpeg, '-y', '-loglevel', 'error', '-i', self.output, '-an',
                        '-vf', f'setpts=PTS/{self.fast_factor}', '-r', f'{self.fps}',
                        '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '21', '-pix_fmt', 'yuv420p',
                        '-movflags', '+faststart', base + '_fast.mp4'], check=False)
        subprocess.run([ffmpeg, '-y', '-loglevel', 'error', '-sseof', f'-{self.ending_seconds}',
                        '-i', self.output, '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20',
                        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', base + '_ending.mp4'],
                       check=False)
        self.get_logger().info(f'also wrote {base}_fast.mp4 ({self.fast_factor:g}x) and {base}_ending.mp4')

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
