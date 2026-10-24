"""Exit-marker detector: camera colour segmentation + LiDAR ranging.

Subscribes:  /camera/image_raw, /camera/camera_info, /scan, TF
Publishes:   /mazemind/exit/detection   geometry_msgs/PoseStamped (map frame; position =
                                        marker centre, orientation = marker normal pointing
                                        into the maze). Only after validation.
             /mazemind/exit/debug_image sensor_msgs/Image (annotated camera view)
             /mazemind/exit/markers     visualization_msgs/MarkerArray

Exit recognition is autonomous: nothing here reads simulator state; it
uses only the robot's simulated camera, LiDAR and SLAM pose (TF).
"""

from __future__ import annotations

import copy
from collections import deque
from typing import Deque, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from mazemind_exploration import outcomes
from mazemind_exploration.exit_perception import (ColorThreshold, DetectionValidator, Observation,
                                                  find_marker_blob, fit_marker_pose,
                                                  points_in_blob_columns, width_consistent)
from mazemind_exploration.transforms import apply, quaternion_from_yaw, transform_to_matrix


class ExitDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__('exit_detector')
        p = self.declare_parameter
        self.global_frame = p('global_frame', 'map').value
        self.marker_width = float(p('marker_width', 0.6).value)
        self.min_area_frac = float(p('min_area_frac', 0.002).value)
        self.min_points = int(p('min_points', 2).value)
        self.max_range = float(p('max_range', 4.0).value)
        self.process_period = 1.0 / float(p('process_rate', 10.0).value)
        self.max_scan_age = float(p('max_scan_age', 0.3).value)
        self.max_tf_age = float(p('max_tf_age', 0.1).value)
        self.tf_fallbacks = 0
        self.blob_frames = 0
        self.thresh = ColorThreshold(int(p('h_min', 45).value), int(p('h_max', 85).value),
                                     int(p('s_min', 120).value), int(p('v_min', 60).value))
        self.validator = DetectionValidator(int(p('confirm_count', 3).value),
                                            float(p('confirm_window', 4.0).value),
                                            float(p('cluster_radius', 0.3).value))

        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info: Optional[CameraInfo] = None
        self.scans: Deque[LaserScan] = deque(maxlen=6)   # ~0.6 s of scans at 10 Hz
        self.last_processed = -1e9
        self.last_published = -1e9
        self.observations = 0
        self.rejections = {'no_scan': 0, 'no_tf': 0, 'few_points': 0, 'width': 0}

        self.create_subscription(CameraInfo, p('camera_info_topic', '/camera/camera_info').value,
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(LaserScan, p('scan_topic', '/scan').value, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, p('image_topic', '/camera/image_raw').value, self._on_image,
                                 qos_profile_sensor_data)
        self.det_pub = self.create_publisher(PoseStamped, outcomes.EXIT_DETECTION_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, '/mazemind/exit/debug_image', 2)
        self.marker_pub = self.create_publisher(MarkerArray, '/mazemind/exit/markers', 2)
        self.get_logger().info('exit detector ready (camera colour + LiDAR range fusion)')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self.scans.append(msg)

    def _on_image(self, msg: Image) -> None:
        now = self.now()
        # 0.8 x period: sim-time jitter must not make a 10 Hz camera fall to 5 Hz processing
        if now - self.last_processed < 0.8 * self.process_period or self.camera_info is None:
            return
        self.last_processed = now
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        blob, _ = find_marker_blob(bgr, self.thresh, self.min_area_frac)
        label = 'searching'
        if blob is not None:
            label = self._process_blob(blob, msg)
            self.blob_frames += 1
            self.get_logger().info(
                f'diagnostics: {self.blob_frames} frames with a marker-coloured blob, '
                f'{self.observations} accepted observations, rejections={self.rejections}, '
                f'last="{label}" (blob width {blob.width}px, area {blob.area}px)',
                throttle_duration_sec=5.0)
        self._publish_debug(bgr, blob, label, msg.header)

    def _process_blob(self, blob, img_msg: Image) -> str:
        img_t = Time.from_msg(img_msg.header.stamp).nanoseconds * 1e-9
        scan = min(self.scans, default=None,
                   key=lambda s: abs(Time.from_msg(s.header.stamp).nanoseconds * 1e-9 - img_t))
        if scan is None:
            self.rejections['no_scan'] += 1
            return 'blob: no scan'
        scan_t = Time.from_msg(scan.header.stamp).nanoseconds * 1e-9
        if abs(img_t - scan_t) > self.max_scan_age:
            self.rejections['no_scan'] += 1
            return 'blob: stale scan'
        cam_frame = img_msg.header.frame_id or self.camera_info.header.frame_id
        try:
            # Time-aware fusion: LiDAR points go to the map frame at the scan time,
            # then into the camera frame at the image time, so robot motion between
            # the two captures does not shift image columns against LiDAR bearings.
            t_map_lidar = transform_to_matrix(
                self._lookup(scan.header.frame_id, scan.header.stamp))
            t_map_cam = transform_to_matrix(self._lookup(cam_frame, img_msg.header.stamp))
        except TransformException as exc:
            self.rejections['no_tf'] += 1
            self.get_logger().debug(f'TF unavailable: {exc}')
            return 'blob: no TF'
        t_cam_lidar = np.linalg.inv(t_map_cam) @ t_map_lidar

        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + scan.angle_increment * np.arange(ranges.size)
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        pts_lidar = np.stack([ranges * np.cos(angles), ranges * np.sin(angles),
                              np.zeros_like(ranges)], axis=1)[valid]
        k = self.camera_info.k
        fx, cx = float(k[0]), float(k[2])
        sel = points_in_blob_columns(apply(t_cam_lidar, pts_lidar), fx, cx, blob,
                                     max_range=self.max_range)
        if int(sel.sum()) < self.min_points:
            self.rejections['few_points'] += 1
            return f'blob: {int(sel.sum())} lidar pts (need {self.min_points})'
        rng = float(np.median(np.linalg.norm(pts_lidar[sel], axis=1)))
        if not width_consistent(blob.width, fx, self.marker_width, rng):
            self.rejections['width'] += 1
            return f'blob: width mismatch @ {rng:.1f}m'

        pts_map = apply(t_map_lidar, pts_lidar[sel])[:, :2]
        viewer = t_map_cam[:2, 3]
        (mx, my), normal_yaw, _ = fit_marker_pose(pts_map, viewer)
        self.observations += 1
        confirmed = self.validator.add(Observation(self.now(), mx, my, normal_yaw))
        if confirmed is None:
            return f'candidate {rng:.1f}m'
        if self.now() - self.last_published >= 0.5:
            self._publish_detection(confirmed)
        return f'EXIT CONFIRMED {rng:.1f}m'

    def _lookup(self, frame: str, stamp):
        """map <- frame at ``stamp``; if TF cannot serve that instant yet (sensor stamps can
        be slightly newer than the latest odometry TF), use the latest transform when it is
        at most ``max_tf_age`` seconds away. In tests the exact-stamp lookup alone dropped
        ~70 % of frames ("no TF") although the marker was clearly visible."""
        try:
            return self.tf_buffer.lookup_transform(self.global_frame, frame, Time.from_msg(stamp),
                                                   timeout=Duration(seconds=0.03))
        except TransformException:
            latest = self.tf_buffer.lookup_transform(self.global_frame, frame, Time())
            gap = abs(Time.from_msg(latest.header.stamp).nanoseconds
                      - Time.from_msg(stamp).nanoseconds) * 1e-9
            if gap > self.max_tf_age:
                raise
            self.tf_fallbacks += 1
            return latest

    def _publish_detection(self, obs: Observation) -> None:
        first = self.last_published < 0
        self.last_published = self.now()
        msg = PoseStamped()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = obs.x, obs.y
        _, _, qz, qw = quaternion_from_yaw(obs.normal_yaw)
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.det_pub.publish(msg)
        if first:
            self.get_logger().info(f'exit marker validated at ({obs.x:.2f}, {obs.y:.2f}) after '
                                   f'{self.observations} observations; rejections={self.rejections}, '
                                   f'tf_fallbacks={self.tf_fallbacks}')
        arr = MarkerArray()
        for i, (mtype, scale, color) in enumerate((
                (Marker.CUBE, (0.08, 0.6, 0.35), ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.8)),
                (Marker.ARROW, (0.45, 0.06, 0.06), ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)))):
            m = Marker()
            m.header = msg.header
            m.ns, m.id, m.type = 'exit', i, mtype
            m.pose = copy.deepcopy(msg.pose)
            m.pose.position.z = 0.2
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color = color
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _publish_debug(self, bgr, blob, label: str, header) -> None:
        if self.debug_pub.get_subscription_count() == 0:
            return
        img = bgr.copy()
        if blob is not None:
            color = (0, 255, 0) if label.startswith('EXIT') else (0, 200, 255)
            cv2.rectangle(img, (blob.u_min, blob.v_min), (blob.u_max, blob.v_max), color, 2)
        cv2.putText(img, label, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    cv2.LINE_AA)
        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = header
        self.debug_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExitDetectorNode()
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
