# Troubleshooting

Every issue below was hit while building MazeMind on WSL2 and was diagnosed from logs or
ground-truth measurements before being fixed.

## Gazebo aborts: `OGRE EXCEPTION(9:UnimplementedException) ... GL3PlusTextureGpu::copyTo`

**Symptom:** `ign gazebo` dies with exit code -6 a few seconds after start. `/scan` and
`/camera/image_raw` never appear.

**Cause:** WSL2's D3D12 Mesa driver (`libgl1-mesa-dri` 23.2.1) reports an OpenGL version below what
Ogre2 needs for this texture copy path. Plain GLX through the GUI hits the same exception.

**Fix (automatic):** `sim.launch.py` has `render_mode:=auto`, which on WSL sets
`MESA_GL_VERSION_OVERRIDE=4.5 MESA_GLSL_VERSION_OVERRIDE=450` for the Gazebo process only. All of these
were measured at real-time factor 1.00 with LiDAR and camera at 10 Hz:

| `render_mode` | Gazebo CPU | Notes |
|---|---|---|
| `gpu_override` (WSL default) | ~73 % headless, ~220 % with GUI | GPU (D3D12) rendering |
| `software` | ~99 % headless | llvmpipe; driver-independent fallback |
| `native` | — | Use on real Linux with proper GPU drivers |

If the override ever fails on a different driver, use `render_mode:=software`.

## RViz opens as a blank white window (WSLg)

RViz's OGRE renderer on WSLg's D3D12 GPU path can show only a white window. `mazemind.launch.py`
starts RViz with software OpenGL (llvmpipe) on WSL by default (`rviz_render_mode:=auto`). If you start
RViz yourself, run `LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d <config>`. Also, don't run an experiment campaign
at the same time: RViz becomes unresponsive when the CPU is saturated.

## SLAM map becomes corrupted mid-run (duplicated, smeared walls)

**Symptom:** RViz shows walls duplicated at an offset and stripy free/occupied regions. Nav2 then fails
to plan to obviously reachable places, and the SLAM pose is metres away from the true pose.

**Diagnosis:** `scripts/snapshot_maps.py` showed the global costmap (built from an earlier `/map`) still
correct while the latest `/map` was warped. The SLAM log contained
`WARNING: Logging before InitGoogleLogging()`, which means the Ceres optimiser ran; SLAM Toolbox only
runs it after accepting a loop closure.

**Cause:** false loop closure. Maze corridors look alike, so the scan matcher accepted a wrong closure
and the pose graph was optimised into a wrong shape.

**Fix:** `do_loop_closing: false` in `config/slam_toolbox.yaml`. With accurate odometry (next section),
scan matching alone kept the mean SLAM error at a few centimetres (the evaluator reports
`localization_error_m` for every run). For larger mazes or noisy odometry, re-enable loop closure with
stricter thresholds (`loop_match_minimum_response_fine`, `loop_match_minimum_chain_size`) and check
the localisation-error metric.

## Wheel odometry under-reports rotation by ~9 %

**Measured** (ground truth vs `/odom`): commanded +90° gave odom 90.0° but a true rotation of 98.4°,
and a full turn was off by 37°.

**Cause:** flat 2 cm wheel cylinders contact the ground at their rim edges, so the effective track
width differs from the `wheel_separation` given to the DiffDrive plugin.

**Fix:** sphere collision geometry for the wheels (the visual is still a cylinder). After the fix,
odometry and ground truth agree to 0.1° over ±450° of rotation and to 1 mm over 0.8 m.

Side effect: wheel odometry is now near-ideal on the flat simulated floor. Slip only appears when
pushing against a wall. This is a simulation simplification (see the README's *Limitations*).

## Nav2 aborts a goal instantly; planner logs `worldToMap failed ... size_x,size_y: 39,39`

**Cause:** Nav2's global costmap follows the SLAM map with a short lag. The explorer evaluated a freshly
grown map and chose a frontier the costmap did not cover yet, so NavFn failed at once.

**Fix:** the explorer subscribes to `/global_costmap/costmap` and defers targets outside its bounds
(`costmap_margin`). Deferred targets don't count as failures, and they don't count toward the
"exploration complete" decision. The NavFn `worldToMap failed` lines can still appear when its
goal-tolerance search touches the costmap edge; on their own they are harmless.

## New goal aborted immediately after a preemption ("failed after 0.0s")

**Symptom:** right after the explorer replaced a goal (target resolved or a better one found), the
*new* goal ended as ABORTED within ~30 ms. Nav2's log showed `Client requested to cancel the goal`
followed by `Aborting handle`.

**Cause:** the explorer cancelled the old `NavigateToPose` goal and immediately sent the new one. The
cancel reached Nav2's action server after the new goal had become active, and aborted it.

**Fix:** a replaced navigation goal is no longer cancelled. Nav2's BT navigator preempts it natively
(`Received goal preemption request`). Cancels are deferred to the end of the control tick, and only
go out for goals that are not replaced by a new navigation goal (timeouts, stuck, finish, switching to
a `spin`).

## NavFn: "Failed to create a plan from potential when a legal potential was found"

**Symptom:** a reachable goal fails, followed by ~20 s of recovery behaviours (clear costmaps, spin,
wait, back up) and eventually blacklisting of a region that was actually reachable.

**Fix:** the global planner is now `nav2_smac_planner/SmacPlanner2D` (`cost_travel_multiplier: 2.0`,
`allow_unknown: true`). This also removes the NavFn `worldToMap failed` log spam.

## Exploration declared complete although the maze is not explored

**Diagnosis:** the explorer now lists every remaining frontier and why it was rejected in the outcome
reason (`unreachable`, `blacklisted`, `too close`, `outside Nav2 costmap`).

**Cause seen:** at long range adjacent LiDAR rays diverge by more than one 5 cm cell, which leaves
unknown slivers that cut a corridor into disconnected free islands.

**Fix:** `gap_fill_cells: 2` treats unknown cells within 10 cm of known-free space as traversable for
reachability.

## Exit marker visible but never validated ("blob: no TF", "few LiDAR points")

**Symptom:** a run maps ~98 % of the maze and ends with `EXPLORATION_COMPLETE_EXIT_NOT_FOUND`.

**Diagnosis:** the robot was respawned at the viewpoint from the failed run, and the detector's
annotated camera image was captured. The marker was clearly segmented, but the frame was labelled
`blob: no TF`. Seven of about ten frames had been discarded because TF could not serve the exact image
timestamp yet (sensor stamps can be slightly newer than the latest odometry TF). The detector's
throttled `diagnostics:` log line then showed a second loss: narrow blobs (≈13 px, oblique views)
covered only ~3 LiDAR bearings, and trimming 2 px per edge left 2 points, below the required 3.

**Fixes:**

* TF lookup tries the exact stamp first, then falls back to the latest transform if it is ≤ 0.1 s away
  (counted as `tf_fallbacks`).
* Every camera frame is processed (10 Hz), so a brief pass still gives enough frames.
* No edge trimming for narrow blobs, and `min_points: 2`. Three-frame validation still prevents
  spurious detections.

After the fixes, the static test logged `no_tf: 0`, and the previously failing run found the exit.

## Lessons from campaign v1 (60 runs), fixed in v2

| v1 failure | Root cause | Fix |
|---|---|---|
| `EXPLORATION_COMPLETE` at 51 % coverage | the only frontier left failed the explorer's free-space reachability graph, although Nav2 could plan to it through unknown space | graph-unreachable frontiers are offered to Nav2 before declaring completion |
| `NAVIGATION_FAILURE` 0.52 m from the true door | marker estimate refined along the wall → approach re-targeted 3× → re-targets counted as failures | only Nav2 failures count; total approach goals capped at 8 |
| exit never validated, 94 % mapped | camera saw the marker once as a 7 px sliver; camera coverage counted floor as seen out to 3.5 m | coverage range 2.5 m |

## Exit marker localised ~0.3–0.4 m along the wall

**Cause:** the image and the most recent scan were up to 0.3 s apart while the robot rotated at up to
1.2 rad/s.

**Fix:** the detector keeps the last 6 scans, uses the one closest in time to the image (≤ 0.15 s), and
chains `lidar→map` at scan time with `map→camera` at image time.

## Processes left over between runs

If a launch is killed hard, `ign gazebo` can survive. Stale SLAM or Gazebo instances on the same
ROS domain corrupt the next run. The experiment runner:

* runs each launch in its own process group (`SIGINT` → `SIGTERM` → `SIGKILL`),
* uses a dedicated `ROS_DOMAIN_ID` (77) and a unique `IGN_PARTITION` per run,
* kills only Gazebo processes running a MazeMind-generated world afterwards.

Manual cleanup:

```bash
pkill -f "ign gazebo.*mazemind/worlds"; pkill -f slam_toolbox; pkill -f nav2
```

## Paths with spaces / Windows drive

Keep the repository wherever you like (e.g. `C:\My Projects\Maze`). `scripts/build.sh` rsyncs `src/`
into `~/mazemind_ws/src` on the Linux filesystem and builds there. That is much faster than building
on `/mnt/c` and avoids tools that mishandle spaces in paths.

## Useful diagnostics

```bash
python3 scripts/nav_goal.py 1.0 1.0                    # one Nav2 goal, prints SUCCEEDED/ABORTED
python3 scripts/snapshot_maps.py /tmp/snaps --tag now  # PNG of /map + global costmap + markers
ros2 topic echo /mazemind/explorer/status              # explorer state machine (JSON)
ros2 topic echo /mazemind/metrics                      # live ground-truth metrics (JSON)
ign topic -e -t /stats -n 3 | grep real_time_factor    # simulation speed
```
