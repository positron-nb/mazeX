"""Helpers shared by MazeMind launch files."""

from __future__ import annotations

import os
from typing import Dict

from mazemind_simulation.generate_cli import build_and_write, default_data_dir
from mazemind_simulation.maze import MazeConfig

# Gazebo <-> ROS bridge. Direction: '[' = Gazebo -> ROS, ']' = ROS -> Gazebo.
BRIDGE_TOPICS = [
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
    '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
    '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
    '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
    '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
    '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
]

# Ground-truth topics: consumed only by mazemind_evaluation.
GROUND_TRUTH_BRIDGE_TOPICS = [
    '/ground_truth/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
    '/ground_truth/contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
]

# Bridged only when recording a demo (the views cost rendering time).
OVERHEAD_CAMERA_TOPIC = '/ground_truth/overhead_camera@sensor_msgs/msg/Image[ignition.msgs.Image'
SCENE_CAMERA_TOPIC = '/ground_truth/scene_camera@sensor_msgs/msg/Image[ignition.msgs.Image'


def running_in_wsl() -> bool:
    try:
        with open('/proc/version', 'r', encoding='utf-8') as fh:
            return 'microsoft' in fh.read().lower()
    except OSError:
        return False


def render_environment(mode: str) -> Dict[str, str]:
    """Environment for the Gazebo process so Ogre2 sensors can render.

    WSL2's D3D12 Mesa driver under-reports its OpenGL version, and Ogre2
    then aborts in ``GL3PlusTextureGpu::copyTo`` before any LiDAR/camera
    data is produced. Overriding the reported version keeps GPU rendering
    and was verified to run at real-time factor 1.0; ``software`` (llvmpipe)
    is a slower but driver-independent fallback.
    """
    if mode == 'auto':
        mode = 'gpu_override' if running_in_wsl() else 'native'
    if mode == 'gpu_override':
        return {'MESA_GL_VERSION_OVERRIDE': '4.5', 'MESA_GLSL_VERSION_OVERRIDE': '450'}
    if mode == 'software':
        return {'LIBGL_ALWAYS_SOFTWARE': '1'}
    if mode == 'native':
        return {}
    raise ValueError(f'unknown render_mode {mode!r}')


def rviz_environment(mode: str) -> Dict[str, str]:
    """RViz under WSLg can show a blank white window on the D3D12 GPU path;
    software OpenGL (llvmpipe) renders it reliably and is fast enough for RViz."""
    if mode == 'auto':
        mode = 'software' if running_in_wsl() else 'native'
    return render_environment(mode)


def rviz_view(maze_json: str, window_px: int = 860) -> Dict[str, str]:
    """TopDownOrtho view parameters that frame the maze in the robot's map frame.

    The SLAM map frame starts at the spawn pose, so the maze centre and the
    rotation that puts world north at the top are computed from the spawn pose.
    Uses the ground-truth file for display only; nothing is sent to the robot.
    """
    import json
    import math
    with open(maze_json, encoding='utf-8') as fh:
        gt = json.load(fh)
    sx, sy, yaw = gt['start']['x'], gt['start']['y'], gt['start']['yaw']
    dx, dy = gt['width'] / 2 - sx, gt['height'] / 2 - sy
    c, s = math.cos(yaw), math.sin(yaw)
    return {'@X@': f'{c * dx + s * dy:.3f}', '@Y@': f'{-s * dx + c * dy:.3f}',
            # TopDownOrtho angle = -spawn yaw puts world north at the top (verified
            # against Gazebo's top-down camera; +yaw showed the map rotated 180 deg)
            '@ANGLE@': f'{-yaw:.4f}',
            '@SCALE@': f'{window_px / (max(gt["width"], gt["height"]) + 1.2):.2f}'}


def maze_from_launch_args(get) -> Dict[str, str]:
    """Generate (or reuse) a maze from launch-configuration values.

    ``get(name)`` returns the string value of a launch configuration.
    Returns world/ground-truth paths and the spawn pose as strings.
    """
    overrides = {
        'seed': int(get('seed')),
        'start': (int(get('start_row')), int(get('start_col'))),
    }
    if get('cell_size'):
        overrides['cell_size'] = float(get('cell_size'))
    if get('rows'):
        overrides['rows'] = int(get('rows'))
    if get('cols'):
        overrides['cols'] = int(get('cols'))
    if get('loop_fraction'):
        overrides['loop_fraction'] = float(get('loop_fraction'))
    cfg = MazeConfig.from_difficulty(get('difficulty'), **overrides)
    data_dir = os.path.expanduser(get('data_dir') or default_data_dir())
    rtf = float(get('real_time_factor') or 1.0)
    return build_and_write(cfg, os.path.join(data_dir, 'worlds'), real_time_factor=rtf)


MAZE_LAUNCH_ARGS = {
    'difficulty': ('easy', 'Maze preset: tiny | easy | medium | hard | showcase'),
    'seed': ('0', 'Maze random seed'),
    'rows': ('', 'Override preset rows'),
    'cols': ('', 'Override preset columns'),
    'loop_fraction': ('', 'Override preset loop fraction'),
    'cell_size': ('', 'Override the preset cell size (metres; presets use 1.0, showcase 1.5)'),
    'start_row': ('0', 'Start cell row'),
    'start_col': ('0', 'Start cell column'),
    'data_dir': ('', 'Where generated worlds are written (default ~/.mazemind)'),
    'real_time_factor': ('1.0', 'Simulation speed target (lower it to record smoothly on slow PCs)'),
}
