# Algorithms

## 1. Maze generation (`mazemind_simulation/maze.py`)

**Randomised depth-first search (recursive backtracker), implemented iteratively.**

1. Start with every wall present on an `rows × cols` grid of square cells.
2. Push the start cell. Repeatedly look at the cell on top of the stack: if it has unvisited
   neighbours, pick one with the seeded RNG, remove the wall between them and push it; otherwise pop.
3. The result is a spanning tree of the cell graph, a *perfect* maze: exactly one route between any
   two cells and many dead ends (the tests verify `openings == cells − 1`).
4. *Optional loops:* a seeded `loop_fraction` of the remaining interior walls is removed, creating
   alternative routes (presets `medium` and `hard`).
5. *Exit:* by default the boundary cell farthest (BFS distance) from the start, with ties broken
   deterministically. A green door panel is mounted on the inner face of that cell's outer wall. The
   outer boundary stays closed, so the robot never leaves the maze.
6. *Validation:* closed boundary, every cell reachable, exit reachable, exit on the boundary, corridor
   width `cell_size − wall_thickness ≥ 2 (robot_radius + min_clearance)` (0.9 m ≥ 0.6 m by default).
7. *Geometry:* collinear wall segments are merged into as few boxes as possible (fewer physics entities).

Same configuration and seed → identical maze (tested). Presets: `tiny` 3×3, `easy` 4×4,
`medium` 6×6 with 10 % loops, `hard` 8×8 with 15 % loops; 1 m cells, 0.1 m walls.

## 2. Frontier exploration (`mazemind_exploration`)

All computations use the **live SLAM occupancy grid** (0.05 m cells), the SLAM pose and the camera pose.

### 2.1 Pipeline (every `replan_period`, only when a new map has arrived)

1. **Traversability.** `free = 0 ≤ v < 25`, `occupied = v ≥ 65`. The clearance of each cell is its
   Euclidean distance transform to the nearest occupied cell. A cell is traversable if it is free (or
   unknown within 2 cells of known-free space; see *gap fill*) **and** its clearance is greater than
   the robot radius (0.15 m). The outer 3-cell rim of the map array is excluded.
2. **Geodesic field.** Compiled Dijkstra (`scipy.sparse.csgraph`) on the 8-connected traversable-cell
   graph from the robot's cell. This gives the travel cost to every cell, reachability, and the path
   for any target. Each step costs `length × (1 + 2 p)`, where `p ∈ [0, 1]` grows as clearance falls
   from 0.35 m to the robot radius. Paths therefore keep to corridor centres, like Nav2's
   inflation-aware planner. (A pure shortest path touches every inner corner at exactly the robot
   radius, which made the path-bottleneck feature ~0 for almost every candidate in a pilot.)
   "Travel cost" below means this clearance-weighted distance, for every strategy.
3. **Frontier cells.** Free cells with a 4-connected unknown neighbour (cells outside the array count
   as unknown).
4. **Clustering.** 8-connected components (`scipy.ndimage.label`). Clusters smaller than
   `min_frontier_size` (8 cells = 0.4 m) are dropped.
5. **Target per cluster.** The cluster cell closest to the centroid (always on the frontier, even for
   L-shaped clusters). If that cell isn't traversable, use the closest traversable cluster cell, else
   the nearest traversable cell within 0.4 m.
6. **Filtering (identical for all strategies).** Drop targets that are unreachable, blacklisted, or
   closer than 0.35 m along the path. Every rejection and its reason is logged, and the reasons are
   included in the final outcome.
7. **Feature extraction** (below), then **ranking** by the active strategy.
8. **Goal dispatch** as an asynchronous `NavigateToPose` action, followed by monitoring.

### 2.2 Visual search: "visual frontiers"

**Problem found in a pilot:** the LiDAR sees 360° out to 8 m, but the exit can only be recognised by
the forward camera (75° FOV, about 3.5 m useful range). LiDAR frontiers vanish long before the camera
has looked down every corridor. In the pilot (`results/pilot_medium_v1_lidar_frontiers_only`), 2 of 3
runs mapped 94–96 % of the maze and finished with `EXPLORATION_COMPLETE_EXIT_NOT_FOUND`: the exit was
mapped by LiDAR but never seen by the camera.

**Mechanism (`visual_coverage.py`):**

1. At 5 Hz, cast 48 rays across the camera's horizontal FOV from the camera pose (TF `map →
   camera_link`) over the SLAM map, out to 2.5 m. (Initially 3.5 m; in campaign v1 one run marked
   the exit's surroundings as seen from a viewpoint where the marker was a 7 px sliver.) Rays stop at the first occupied cell, and that wall
   cell is marked as seen. Unknown cells are passed through but not marked (inside LiDAR range they
   are gaps between rays, not hidden structure).
2. Seen cells are stored as world-anchored integer keys, so the record survives SLAM map growth and
   origin shifts.
3. **Visual frontiers** are connected regions of known-free, traversable floor the camera has not seen,
   at least `min_unseen_cells` (60 cells = 0.15 m²) in size. Each gets a target (the region cell
   closest to its centroid) and joins the candidate list with `kind = 'visual'`.
4. Information gain treats unseen floor as unknown, so the same score applies to both kinds.
5. If a visual target is closer than `look_distance` (0.6 m), the robot turns toward the region with the
   Nav2 `spin` action instead of driving.
6. Exploration is complete only when **no** LiDAR frontier **and** no visual frontier remains.

### 2.3 Strategies

| Strategy | Rule |
|---|---|
| `random` | Uniform choice among valid candidates (RNG seeded with the maze seed). Never preempts its goal. |
| `nearest` | Minimum geodesic travel cost. Preempts only if a new target is ≥ 0.5 m closer. |
| `weighted` | Maximum score (below). Preempts only if a new score beats the current one by ≥ 0.25. |

### 2.4 Weighted score

```
Score(F) =  w_info      · info_n(F)
          + w_size      · size_n(F)
          + w_clearance · clearance_n(F)
          + w_success   · success_estimate(F)
          − w_distance  · travel_cost_n(F)
          − w_revisit   · revisit_penalty(F)
          − w_failure   · failure_penalty(F)
```

| Feature | Definition | Normalisation | Default weight | Why |
|---|---|---|---|---|
| `info` | Unknown cells **visible** from the target: 72 rays out to 2 m, each stopping at the first occupied cell. Unknown space behind known walls does not count, which matters in mazes where the space behind a wall is another corridor. | ÷ max over candidates | 1.0 | Prefer targets that reveal the most new space. |
| `size` | Frontier cluster size (cells). | ÷ max | 0.3 | Wide openings are usually corridors, not slivers. |
| `clearance` | Distance from the target to the nearest obstacle. | ÷ 0.45 m (half a corridor), capped at 1 | 0.3 | Targets near walls cause planner/controller trouble. |
| `success_estimate` | `clip((bottleneck − r_robot) / (0.35 − r_robot), 0, 1)`, where `bottleneck` is the minimum clearance along the geodesic path. A heuristic traversability proxy, **not** a calibrated probability. | already [0, 1] | 0.5 | Prefer paths without tight squeezes. |
| `travel_cost` | Geodesic path length (m). | ÷ max | 1.2 | Strongest term: back-tracking is the main cost of maze exploration. |
| `revisit_penalty` | Attempts (reached / failed) already made within 0.5 m ÷ 3. | already [0, 1] | 0.8 | A frontier that survives visits is probably unobservable. |
| `failure_penalty` | Failures within 0.5 m ÷ 2. | already [0, 1] | 1.5 | Avoid regions where Nav2 already failed. |

**Why divide by the maximum and not min–max?** Min–max stretches tiny differences: two targets at
1.0 m and 1.1 m would get distances 0 and 1. Dividing by the maximum keeps ratios, so 1.0 m vs 1.1 m
becomes 0.91 vs 1.0.

**Weight rationale.** Distance carries the largest weight because unnecessary back-tracking dominates
travel in a tree-shaped maze. Information gain is next; the ray-cast version stops large but hidden
unknown regions from attracting the robot. The safety terms (`clearance`, `success`) act as
tie-breakers. The failure penalty is high so a single Nav2 failure is enough to shift the choice
elsewhere. All weights live in `config/exploration.yaml`. They were set from this reasoning and were
**not** tuned on the evaluation seeds.

### 2.5 Goal monitoring and recovery

| Mechanism | Rule |
|---|---|
| Duplicate goals | A goal is only sent when there is none active, or when replacing the active one. Stale action callbacks are ignored by sequence number. |
| Resolution | If no frontier cell remains within 0.4 m of the active target, the goal is cancelled as `resolved` (not a failure) and the next target is selected immediately. |
| Anti-oscillation | Minimum 4 s commitment, plus the switch margins above. `random` never switches. |
| Timeout | `30 s + 8 s/m × planned cost` → `timed_out` (failure). |
| Stuck | Moved < 0.15 m in 20 s while a goal is active → cancel, `stuck` (failure). |
| Nav2 abort / rejection | Recorded as `failed`. |
| Blacklist | A region (0.5 m) with ≥ 2 failures, or ≥ 3 attempts that did not resolve it, is excluded. |
| Nav2 behaviours | The default BT (replanning, clear costmaps, spin, back-up, wait) runs inside each goal. The explorer records `number_of_recoveries`. |
| Navigation failure | 6 consecutive failed goals → `NAVIGATION_FAILURE`. |
| Sensor failure | No `/scan` for 10 s → `SIMULATION_ERROR`. |
| Nav2 unavailable | No action server for 30 s → `SIMULATION_ERROR`. Not ready within 120 s of startup (wall clock) → `SIMULATION_ERROR`. |
| Graph-unreachable fallback | Before declaring completion, a LiDAR frontier that the free-space graph calls *unreachable* is sent to Nav2 once (Nav2 plans through unknown space). Its failures are blacklisted as usual. Added after a v1 run stopped at 51 % coverage. |
| Completion | 4 consecutive evaluations (≥ 1 s apart) with no valid candidate → `EXPLORATION_COMPLETE_EXIT_NOT_FOUND`, reporting how many frontiers remain and why each was rejected. |
| Time limit | `time_limit` simulated seconds after the system is ready → `TIMEOUT`. |

### 2.6 Gap fill

At long range, adjacent LiDAR rays diverge by more than a map cell (1° ≈ 7 cm at 4 m). That leaves
unknown slivers inside a corridor which split one free region into disconnected islands. A frontier
can then look unreachable even though Nav2, which plans through unknown space, could reach it. Unknown
cells within 2 cells (0.1 m) of known-free space therefore count as traversable when computing
reachability and cost. Paths cannot wander through larger unexplored regions.

## 3. Exit recognition (`exit_perception.py`, `exit_detector_node.py`)

**Option A (camera marker) with LiDAR ranging: autonomous.**

1. **Segmentation.** BGR → HSV threshold (H 45–85, S ≥ 120, V ≥ 60), 3×3 morphological opening,
   largest connected component. It must cover ≥ 0.2 % of the image. The marker is the only saturated
   green surface in the world (walls are grey).
2. **Time-aware fusion.** Take the scan closest in time to the image (≤ 0.15 s apart). Transform its
   points `lidar → map` at the scan time and `map → camera` at the image time. Robot rotation between
   the two captures therefore doesn't shift image columns relative to LiDAR bearings.
3. **Association.** Project the points with the pinhole model `u = f_x · x / z + c_x` and keep those
   whose column falls inside the blob's horizontal extent and that lie within 4 m. At least 3 are needed.
4. **Pose.** Marker centre = median of the selected points in the map frame. The wall direction comes
   from an SVD line fit; the normal is oriented toward the camera.
5. **Plausibility.** The blob width must be 0.15–1.8× the width a 0.6 m marker would have at the
   measured range.
6. **Validation.** 3 observations within 4 s whose positions lie within 0.3 m of their mean. After
   that, consistent observations refine the estimate (running mean) and outliers are ignored.
7. **Explorer reaction.** Stop selecting frontiers, cancel the active goal, and navigate to the point
   0.40 m in front of the marker along its normal, facing it. If the estimate moves by more than
   0.3 m, the approach re-targets. Only Nav2 failures count against the approach (max 3), and the total
   number of approach goals is capped at 8. In v1, re-targets counted as failures, and one run was
   declared failed 0.52 m from the true door.
8. **Arrival.** The robot stands in front of the estimated marker: ≤ 0.70 m along its normal
   and ≤ 0.30 m sideways. (A plain distance test let a pilot run stop in the neighbouring cell,
   2 cm outside the exit cell. Ground-truth verification caught it.) This must be ≥ the
   approach offset + Nav2's 0.25 m goal tolerance; a pilot run that used 0.6 m failed at 0.63 m. Then publish zero
   velocity and report `EXIT_REACHED`.
9. **Independent verification** (evaluator, ground truth): `exit_success = true` only if the true robot
   position is inside the exit cell and within 1.0 m of the true door centre when the explorer reports
   `EXIT_REACHED`. Detection accuracy is also recorded as the distance between the estimated and true
   marker centre.

Not implemented: option B (beacon) and option C (external signal). If a run reaches the exit, it found
the exit itself.

## 4. Terminal outcomes

| Outcome | Meaning |
|---|---|
| `EXIT_REACHED` | Marker detected, approached, robot within 0.70 m (verified separately with ground truth). |
| `EXPLORATION_COMPLETE_EXIT_NOT_FOUND` | No valid frontier remains and no exit was recognised. |
| `TIMEOUT` | Exploration time limit reached. |
| `NAVIGATION_FAILURE` | 6 consecutive goal failures, or the detected exit could not be reached in 3 attempts. |
| `SIMULATION_ERROR` | Sensor data stale, Nav2 unavailable, startup failure, or (in the runner) no result produced. |
