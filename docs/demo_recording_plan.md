# Demo recording plan

Goal: a 60–90 s clip that shows **genuine simulator output**. The robot enters an unknown maze, builds
a map, picks frontiers, recognises the exit marker, drives to it and stops. Nothing is staged or
re-rendered.

## Option 1: automatic side-by-side recording (no screen capture needed)

`record:=true` bridges a top-down camera that exists only in the Gazebo world and is published under
`/ground_truth/`, so the explorer never sees it. The `media_recorder` node then writes an MP4 whose
frames are composed only from live topics:

| Region | Source topic |
|---|---|
| Left: Gazebo top-down view | `/ground_truth/overhead_camera` (simulator) |
| Right: robot's view | `/map`, TF `map→base_footprint`, `/mazemind/frontiers`, `/plan`, `/mazemind/exit/detection` |
| Bottom left: robot camera + detector box | `/mazemind/exit/debug_image` |
| Bottom right: live metrics | `/mazemind/metrics` (evaluator) |

```bash
source scripts/env.sh
ros2 launch mazemind_bringup mazemind.launch.py difficulty:=showcase seed:=3 strategy:=weighted \
    headless:=true rviz:=false record:=true record_timelapse:=8 run_id:=demo \
    record_path:=$MAZEMIND_REPO/media/demo.mp4
```

This command produced `media/showcase_18m_s3_weighted.mp4`: 85 s of video, an 8× time-lapse of a 651 s
run. The robot-map panel is rotated by the spawn heading for display, so it lines up with the Gazebo
view. The README GIF was made with
`ffmpeg -i media/showcase_18m_s3_weighted.mp4 -vf "setpts=0.2*PTS,fps=8,scale=720:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer:bayer_scale=4" media/showcase_preview.gif`. When the run ends,
the last frame is held for 2 s and also saved as `*_final.png`.

## Option 2: live GUI screen recording (Gazebo left, RViz right)

1. Launch with the GUIs:
   `ros2 launch mazemind_bringup mazemind.launch.py difficulty:=medium seed:=3 strategy:=weighted`
2. Snap the Gazebo window to the left half of the screen and RViz to the right (Win + ←/→ works on
   WSLg windows).
3. RViz shows the SLAM map, LiDAR, global costmap, frontier clusters (cyan), candidate targets
   (orange, chosen = green, score labels), blacklisted regions (red discs), the Nav2 plan, the
   exit-marker estimate (green box + normal arrow), the annotated robot camera, and the live metrics
   text above the robot.
4. Record with OBS Studio (Display Capture) or the Xbox Game Bar (`Win + Alt + R`).
5. Start recording **before** launching, so the clip begins with an empty map.

## Shot list (≈ 75 s)

| Time | Shot | What to say on screen / in captions |
|---|---|---|
| 0–5 s | Empty map, robot at the start | "No map. The robot only has a simulated LiDAR, camera and wheel odometry." |
| 5–25 s | Map growing, frontier clusters, chosen target | "Frontiers = known-free cells next to unknown cells. Each one is scored for information gain, distance and safety." |
| 25–45 s | Robot turning corners, re-selecting when a target resolves | "When new scans resolve a frontier, it re-plans immediately instead of driving there." |
| 45–60 s | Green marker enters the camera, detector box turns green, star in the map | "Exit recognised by camera colour segmentation, ranged with LiDAR, confirmed over 3 observations." |
| 60–70 s | Approach and stop, `OUTCOME: EXIT_REACHED` | "It stops in front of the exit; the evaluator verifies the true position with ground truth." |
| 70–75 s | Comparison plot from `results/<campaign>/comparison.png` | "Random vs nearest vs weighted, evaluated with identical maze seeds." |

## Checklist

- [ ] Use a seed where the exit is not visible from the start (check `generate_maze --print`).
- [ ] Use `difficulty:=medium` for a longer, more interesting exploration.
- [ ] Keep the metrics panel visible; every number in it is live.
- [ ] Don't speed-ramp or cut out failures silently. If a run fails, show the reported outcome.
- [ ] Caption the time-lapse factor (Option 1 is 3×).
