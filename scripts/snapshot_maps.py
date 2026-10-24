#!/usr/bin/env python3
"""Save the live SLAM map and global costmap as PNGs (debugging / documentation).

    python3 scripts/snapshot_maps.py OUT_DIR [--tag NAME]

Overlays the robot pose (TF map->base_footprint), frontier markers and the
current exit detection if available. Prints the grid metadata so size or
origin mismatches between SLAM and Nav2 are visible.
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.time import Time  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402
from visualization_msgs.msg import MarkerArray  # noqa: E402


class Snap(Node):
    def __init__(self):
        super().__init__('mazemind_snapshot', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.maps = {}
        self.frontiers = None
        self.exit = None
        self.create_subscription(OccupancyGrid, '/map', lambda m: self.maps.__setitem__('map', m), latched)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: self.maps.__setitem__('costmap', m), 10)
        self.create_subscription(MarkerArray, '/mazemind/frontiers',
                                 lambda m: setattr(self, 'frontiers', m), 10)
        self.create_subscription(PoseStamped, '/mazemind/exit/detection',
                                 lambda m: setattr(self, 'exit', m), 10)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)


def extent(info):
    x0, y0 = info.origin.position.x, info.origin.position.y
    return [x0, x0 + info.width * info.resolution, y0, y0 + info.height * info.resolution]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--tag', default='snapshot')
    ap.add_argument('--wait', type=float, default=6.0)
    args = ap.parse_args()
    rclpy.init()
    node = Snap()
    import time
    t0 = time.time()
    while time.time() - t0 < args.wait:
        rclpy.spin_once(node, timeout_sec=0.2)
    pose = None
    try:
        tf = node.tf.lookup_transform('map', 'base_footprint', Time())
        pose = (tf.transform.translation.x, tf.transform.translation.y)
    except Exception as exc:  # noqa: BLE001
        print('no robot pose:', exc)
    os.makedirs(args.out, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    for ax, key in zip(axes, ('map', 'costmap')):
        msg = node.maps.get(key)
        if msg is None:
            ax.set_title(f'{key}: not received')
            continue
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        img = np.where(data < 0, 0.5, 1.0 - data / 100.0)
        ax.imshow(img, origin='lower', cmap='gray', vmin=0, vmax=1, extent=extent(info))
        ax.set_title(f'{key} {info.width}x{info.height} @ {info.resolution:.2f} '
                     f'origin=({info.origin.position.x:.2f},{info.origin.position.y:.2f})')
        print(f'{key}: size {info.width}x{info.height} res {info.resolution:.3f} '
              f'extent x[{extent(info)[0]:.2f},{extent(info)[1]:.2f}] '
              f'y[{extent(info)[2]:.2f},{extent(info)[3]:.2f}]')
        if pose:
            ax.plot(*pose, 'bo', ms=8, label='robot (SLAM)')
        if node.frontiers:
            for m in node.frontiers.markers:
                if m.ns == 'frontier_cells':
                    ax.plot([p.x for p in m.points], [p.y for p in m.points], 'c.', ms=2)
                elif m.ns == 'candidates':
                    ax.plot(m.pose.position.x, m.pose.position.y, 'o',
                            color='lime' if m.id == 0 else 'orange', ms=7)
        if node.exit:
            ax.plot(node.exit.pose.position.x, node.exit.pose.position.y, 'g*', ms=16,
                    label='exit detection')
        ax.set_aspect('equal')
    axes[0].legend(loc='upper right', fontsize=8)
    path = os.path.join(args.out, f'{args.tag}.png')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print('saved', path)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
