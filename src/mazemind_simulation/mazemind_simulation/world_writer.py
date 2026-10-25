"""Write a Gazebo (Ignition Fortress, SDF 1.8) world for a generated maze.

Physics and sensing geometry: wall collisions are merged boxes (few physics
entities). Appearance: brick-coloured segments between pillars, a two-tone
chequered floor (one tile per cell), the green exit panel, sun with shadows and sky, a clean GUI layout, and
two cameras used only for demo recordings (top-down and oblique), published
under /ground_truth/.
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple
from xml.sax.saxutils import escape

from mazemind_simulation.maze import Maze

WORLD_NAME = 'mazemind_world'

# Plain materials only: image textures (PBR albedo maps) made the walls invisible to
# the robot's camera under Fortress/ogre2 on WSL, which would break exit detection.
WALL_RGBA = '0.62 0.36 0.28 1'
PILLAR_RGBA = '0.55 0.55 0.57 1'
FLOOR_RGBA = '0.72 0.69 0.62 1'
FLOOR_ALT_RGBA = '0.64 0.61 0.55 1'
OUTER_RGBA = '0.42 0.40 0.36 1'
# The exit sign is the only saturated green surface in the world; the camera
# detector keys on this hue. Emissive so it reads the same in any lighting.
DOOR_RGBA = '0.05 0.85 0.10 1'


def _material(rgba: str, emissive: str = '0 0 0 1') -> str:
    return (f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular><emissive>{emissive}</emissive></material>')


def _box_visual(name: str, pose: Tuple[float, ...], size: Tuple[float, float, float],
                material: str, cast_shadows: bool = True) -> str:
    x, y, z, yaw = pose
    sx, sy, sz = size
    return f"""
      <visual name="{name}">
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>
        <cast_shadows>{str(cast_shadows).lower()}</cast_shadows>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        {material}
      </visual>"""


def _box_collision(name: str, pose: Tuple[float, ...], size: Tuple[float, float, float]) -> str:
    x, y, z, yaw = pose
    sx, sy, sz = size
    return f"""
      <collision name="{name}">
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
      </collision>"""


def _wall_segments(maze: Maze):
    """Individual wall segments (between grid nodes) and the grid nodes they touch."""
    s = maze.config.cell_size
    segments, nodes = [], set()
    for i, line in enumerate(maze.h_walls):
        for c, present in enumerate(line):
            if present:
                segments.append(((c + 0.5) * s, i * s, 0.0))
                nodes.update({(c, i), (c + 1, i)})
    for r, line in enumerate(maze.v_walls):
        for j, present in enumerate(line):
            if present:
                segments.append((j * s, (r + 0.5) * s, math.pi / 2))
                nodes.update({(j, r), (j, r + 1)})
    return segments, sorted(nodes)


def camera_poses(maze: Maze) -> Dict[str, Tuple[float, ...]]:
    """Recording / GUI viewpoints framing the whole maze (x, y, z, roll, pitch, yaw)."""
    cx, cy = maze.width / 2, maze.height / 2
    half = max(maze.width, maze.height) / 2 + 0.4
    top_h = 2.0 * half + 1.0
    elevation = math.radians(58)
    dist = 2.2 * half              # frames the whole maze with a 62 deg field of view
    return {
        'top': (cx, cy, top_h, 0.0, math.pi / 2, math.pi / 2),
        'oblique': (cx, cy - dist * math.cos(elevation), dist * math.sin(elevation),
                    0.0, elevation, math.pi / 2),
    }


def _camera_model(name: str, topic: str, pose: Tuple[float, ...], hfov: float, size: int,
                  rate: float, far: float) -> str:
    x, y, z, roll, pitch, yaw = pose
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>
      <link name="link">
        <sensor name="{name}" type="camera">
          <topic>{topic}</topic>
          <update_rate>{rate}</update_rate>
          <always_on>true</always_on>
          <camera>
            <horizontal_fov>{hfov:.4f}</horizontal_fov>
            <image><width>{size}</width><height>{size}</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>{far:.1f}</far></clip>
          </camera>
        </sensor>
      </link>
    </model>"""


def recording_cameras_sdf(maze: Maze, image_size: int = 1024, rate: float = 30.0) -> str:
    """Top-down and oblique cameras for demo videos. Gazebo renders a camera only
    while something subscribes, i.e. only when recording is enabled."""
    poses = camera_poses(maze)
    half = max(maze.width, maze.height) / 2 + 0.4
    top = poses['top']
    top_hfov = 2.0 * math.atan(half / (top[2] - 0.5))
    far = 4.0 * max(maze.width, maze.height) + 20.0
    return (_camera_model('overhead_camera', '/ground_truth/overhead_camera', top, top_hfov,
                          image_size, rate, far)
            + _camera_model('scene_camera', '/ground_truth/scene_camera', poses['oblique'],
                            math.radians(62), image_size, rate, far))


def gui_sdf(maze: Maze) -> str:
    """Clean Gazebo GUI: full-window 3D view framing the maze, play/pause and stats only."""
    x, y, z, _, pitch, yaw = camera_poses(maze)['oblique']
    return f"""
    <gui fullscreen="false">
      <plugin filename="GzScene3D" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.55 0.55 0.55</ambient_light>
        <background_color>0.72 0.82 0.93</background_color>
        <camera_pose>{x:.3f} {y:.3f} {z:.3f} 0 {pitch:.4f} {yaw:.4f}</camera_pose>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <ignition-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>
        <play_pause>true</play_pause>
        <step>false</step>
        <start_paused>false</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <ignition-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
      </plugin>
    </gui>"""


def world_sdf(maze: Maze, physics_step: float = 0.002, real_time_factor: float = 1.0) -> str:
    cfg = maze.config
    s, t, h = cfg.cell_size, cfg.wall_thickness, cfg.wall_height

    # --- physics: merged wall boxes (collision only)
    collisions: List[str] = [
        _box_collision(f'wall_{i}_collision', (b.cx, b.cy, h / 2, 0.0), (b.sx, b.sy, h))
        for i, b in enumerate(maze.wall_boxes())]

    # --- appearance: brick segments between pillars (no coplanar overlaps -> no flicker)
    segments, nodes = _wall_segments(maze)
    wall_mat = _material(WALL_RGBA)
    pillar_mat = _material(PILLAR_RGBA)
    visuals: List[str] = []
    for k, (x, y, yaw) in enumerate(segments):
        visuals.append(_box_visual(f'wall_seg_{k}', (x, y, h / 2, yaw), (s - t, t, h), wall_mat))
    pt = t * 1.12
    for k, (gx, gy) in enumerate(nodes):
        visuals.append(_box_visual(f'pillar_{k}', (gx * s, gy * s, (h + 0.04) / 2, 0.0),
                                   (pt, pt, h + 0.04), pillar_mat))

    # --- floor: chequered tiles, one per cell (gives a sense of scale), darker ground outside
    tile_mats = (_material(FLOOR_RGBA), _material(FLOOR_ALT_RGBA))
    floor: List[str] = []
    for r in range(cfg.rows):
        for c in range(cfg.cols):
            floor.append(_box_visual(f'tile_{r}_{c}', ((c + 0.5) * s, (r + 0.5) * s, -0.005, 0.0),
                                     (s, s, 0.01), tile_mats[(r + c) % 2], cast_shadows=False))
    outer = max(maze.width, maze.height) + 12.0

    d = maze.door_pose()
    door_collision = _box_collision('door_collision', (d['x'], d['y'], d['z'], d['yaw'] + math.pi / 2),
                                    (d['width'], d['thickness'], d['height']))
    door_visual = _box_visual('door_visual', (d['x'], d['y'], d['z'], d['yaw'] + math.pi / 2),
                              (d['width'], d['thickness'], d['height']),
                              _material(DOOR_RGBA, emissive=DOOR_RGBA))
    title = escape(f'MazeMind {cfg.rows}x{cfg.cols} seed={cfg.seed}')

    return f"""<?xml version="1.0"?>
<!-- Generated by mazemind_simulation: {title}. Do not edit by hand. -->
<sdf version="1.8">
  <world name="{WORLD_NAME}">
    <physics name="default_physics" type="ode">
      <max_step_size>{physics_step}</max_step_size>
      <real_time_factor>{real_time_factor}</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>
{gui_sdf(maze)}
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.72 0.82 0.93 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.35 0.45 -0.82</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{outer:.1f} {outer:.1f}</size></plane></geometry>
        </collision>
        <visual name="outer_ground">
          <pose>{maze.width / 2:.3f} {maze.height / 2:.3f} -0.02 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{outer:.1f} {outer:.1f}</size></plane></geometry>
          {_material(OUTER_RGBA)}
        </visual>{''.join(floor)}
      </link>
    </model>

    <model name="maze">
      <static>true</static>
      <link name="walls">{''.join(collisions)}{''.join(visuals)}
      </link>
    </model>

    <model name="exit_door">
      <static>true</static>
      <link name="panel">{door_collision}{door_visual}
      </link>
    </model>
{recording_cameras_sdf(maze)}
  </world>
</sdf>
"""


def write_world(maze: Maze, path: str, **kwargs) -> None:
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(world_sdf(maze, **kwargs))
