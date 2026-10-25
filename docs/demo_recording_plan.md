# Watching and recording MazeMind

## Watch it live (Gazebo + RViz)

Run in an **Ubuntu (WSL) terminal**, not PowerShell:

```bash
cd "/mnt/c/My Projects/Maze" && source scripts/env.sh
ros2 launch mazemind_bringup mazemind.launch.py difficulty:=medium seed:=3 rviz_render_mode:=gpu_override
```

* **Gazebo** opens as a clean full-window 3D view framing the whole maze, with play/pause and
  real-time stats in the corners.
* **RViz** opens with the view centred on the maze and rotated to match Gazebo. It shows the SLAM map,
  LiDAR (orange), frontier cells (blue), candidate targets (orange; chosen = green), the Nav2 plan
  (green), the travelled path (cyan) and the exit detection.
* `rviz_render_mode:=gpu_override` uses the GPU: ~20–30 % CPU instead of ~100–150 % with software
  rendering (measured). If RViz shows only a **white window** on your PC, drop that argument; the
  default on WSL is software rendering, which always displays but is heavier.
* Drag the two windows side by side. RViz opens at the top-left; WSLg lets Windows place windows.

## Record a demo video (recommended: no screen capture needed)

```bash
ros2 launch mazemind_bringup mazemind.launch.py difficulty:=showcase seed:=3 headless:=true \
    rviz:=false record:=true real_time_factor:=0.5 \
    record_path:=$MAZEMIND_REPO/media/my_demo.mp4
```

The recorder builds every frame from live simulation topics, 1920×1080:

| Region | Source |
|---|---|
| Left: Gazebo 3D view of the maze | `/ground_truth/scene_camera` (oblique simulator camera; `record_view:=top` for top-down) |
| Right: robot's own map | `/map`, `/scan`, TF, `/mazemind/frontiers`, `/plan`, `/mazemind/trail`, `/mazemind/exit/detection` |
| Bottom right: robot camera + metrics | `/mazemind/exit/debug_image`, `/mazemind/metrics` |

* Frames are sampled at **30 fps of simulated time** and streamed to H.264, so `my_demo.mp4` plays at
  exactly **1×**. If the PC falls behind, frames are repeated to keep the timing, so record with
  `real_time_factor:=0.5` for perfectly smooth output.
* When the robot reaches the exit, it turns to face the exit sign and the video holds the last frame
  for 2 s. Three files are written:
  * `my_demo.mp4`: the full run at 1×.
  * `my_demo_fast.mp4`: 8× (`record_fast:=N`). It is still 30 fps, because frames are dropped, not
    stretched.
  * `my_demo_ending.mp4`: the last 35 s at 1×, like an "ending clip".
* The `*_final.png` file is the last frame (good as a thumbnail).

## Suggested LinkedIn cut

* Use `*_fast.mp4` (the whole escape in about 1.5 minutes) or `*_ending.mp4`.
* Caption the speed-up ("8× time-lapse of simulated time").
* Never cut out failures silently. If a run fails, the outcome text in the video says so.

## Optional: screen-record the live GUIs

Use Xbox Game Bar (`Win + Alt + R`) or OBS while the live command above runs. Start recording before
launching, so the clip begins with an empty map.
