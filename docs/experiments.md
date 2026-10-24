# Experiments

## Protocol

* **Identical conditions:** every strategy runs on the same maze seeds, with the same parameters,
  robot, Nav2/SLAM configuration and time limit. Runs are interleaved by seed
  (`seed0:random, seed0:nearest, seed0:weighted, seed1:…`) so slow drifts in machine load affect all
  strategies alike.
* **Fresh simulation per run:** `run_experiments` launches the full stack for every run in its own
  process group, ROS domain and Gazebo partition, then tears it all down. No state carries over.
* **Identical candidate filtering:** all strategies see the same candidate list (reachable,
  non-blacklisted, LiDAR and visual frontiers). Only the selection rule differs, plus switching
  behaviour: `random` never preempts its goal; the others use the hysteresis in `algorithm.md`.
* **Seeded randomness:** the `random` strategy's RNG is seeded with the maze seed, so runs are
  reproducible up to simulator and timing nondeterminism. Gazebo, SLAM and Nav2 timing is *not*
  bit-exact between runs; repeated runs on one seed can differ.
* **Ground truth only in the evaluator.** Metrics come from `/ground_truth/*` and the maze JSON. The
  robot never receives them.

## Metric definitions

| Metric | Definition |
|---|---|
| `exit_success` | Explorer reported `EXIT_REACHED` **and** the true robot position (Gazebo pose) is inside the exit cell and ≤ 1.0 m from the true door centre. |
| `final_status` | Explorer's terminal outcome (`EXIT_REACHED`, `EXPLORATION_COMPLETE_EXIT_NOT_FOUND`, `TIMEOUT`, `NAVIGATION_FAILURE`, `SIMULATION_ERROR`). The runner records `SIMULATION_ERROR` if the stack produced no result. |
| `time_to_first_map_s` | Simulated time from Gazebo start to the first `/map` message. |
| `time_to_exit_s` | Simulated time from the moment the explorer is ready (map + TF + Nav2) to `EXIT_REACHED`. |
| `distance_gt_m` | Length of the **true** trajectory: sum of ground-truth position increments ≥ 1 mm at 20 Hz. |
| `distance_ratio` | `distance_gt_m` ÷ length of the cell-centre shortest route from start to exit (successful runs). Can be < 1 because the robot cuts corners and stops ~0.5 m before the door. |
| `coverage` | Fraction of the maze's traversable floor (ground-truth free cells at the SLAM resolution, eroded by one cell to drop wall-edge cells) that the SLAM map marks known-free (value 0–24). SLAM cells are looked up by transforming each ground-truth cell centre through the spawn pose (the SLAM map frame is anchored at the spawn pose). Reported at the end of the run. A successful run stops once the exit is reached, so its coverage is usually **below** 100 %. |
| `goals_sent` / `goals_failed` | Navigation goals dispatched (`NavigateToPose` and look-around `Spin`). Failed = Nav2 abort or rejection, timeout, or stuck detection. Goals cancelled because their frontier was already resolved, or because a better target preempted them, are **not** failures and are counted separately. |
| `recoveries` | Sum of Nav2's `number_of_recoveries` feedback over all goals (spin / back-up / wait / costmap clearing inside the behaviour tree). |
| `collision_events` | Physical contacts between the chassis collision and any non-ground entity, from Gazebo's contact sensor. A new event starts after > 0.5 s without contact. |
| `near_collision_events` | Downward crossings of true clearance (robot body edge to nearest wall box) below **0.05 m**, re-armed above 0.07 m. `min_clearance_m` is the smallest clearance in the run. |
| `localization_error_m` | Once per second: distance between the SLAM pose (transformed to the world frame) and the true pose. Reported as mean/median/max. |
| `exit_detection.error_m` | Distance from the robot's validated exit-marker estimate (world frame) to the true marker centre. |

Statistics: mean ± sample standard deviation, median, n. Success rates get 95 % Wilson score
intervals, which behave sensibly for small n and for 0 % or 100 %.

## Campaigns

<!-- CAMPAIGNS_START -->
All numbers below come from `results/easy_main/` and `results/medium_main/` (campaign v2, final code).
Full tables, per-run JSON, CSV and plots are in those folders.

### Setup

| | easy | medium |
|---|---|---|
| Maze | 4×4 cells, 1 m cells (4 m × 4 m), perfect maze | 6×6 cells (6 m × 6 m), 10 % extra openings (loops) |
| Seeds | 0–9 | 0–9 |
| Strategies | random, nearest, weighted | random, nearest, weighted |
| Runs | 30 | 30 |
| Time limit | 420 s simulated | 600 s simulated |
| Exit | farthest boundary cell from the start, green marker | same |

Runs used real-time factor ≈ 1.0 on a Ryzen 9 270 laptop under WSL2. Total wall time was 0.6 h (easy) and
1.2 h (medium). One medium run (`random`, seed 1) hit a Nav2 lifecycle startup hang before the robot
moved. It was automatically re-run, and the failed attempt is kept in
`runs/medium_s001_random.attempt1.json`.

### Results (n = 10 seeds per strategy)

| Maze | Strategy | Exit success (verified) | Distance to exit (m) | Time to exit (s, sim) |
|---|---|---|---|---|
| easy | random | 10/10 | 12.5 ± 3.6 | 61.6 ± 14.0 |
| easy | nearest | 10/10 | 12.5 ± 1.9 | 67.5 ± 12.5 |
| easy | **weighted** | 10/10 | **11.1 ± 1.4** | **55.2 ± 8.2** |
| medium | random | 10/10 | 43.0 ± 10.7 | 186.1 ± 43.3 |
| medium | nearest | 10/10 | **23.0 ± 6.7** | 111.7 ± 29.3 |
| medium | weighted | 9/10 | 23.4 ± 5.8 (n = 9) | **102.1 ± 23.1** (n = 9) |

Mean ± sample std over successful runs. The 95 % Wilson intervals are 72–100 % for 10/10 and 60–98 %
for 9/10, so **success rates do not separate the strategies at this sample size**.

### Paired comparison (same maze, both runs successful)

| Maze | Comparison | Distance difference | Time difference |
|---|---|---|---|
| easy | weighted vs nearest | weighted 1.4 m shorter; shorter on 8/10 mazes (p = 0.020) | 12.3 s faster; faster on **10/10** (p = 0.002) |
| easy | weighted vs random | 1.3 m shorter; 6/10 (p = 0.32) | 6.4 s faster; 7/10 (p = 0.19) |
| medium | weighted vs random | 21.3 m shorter; **9/9** (p = 0.004) | 91.4 s faster; 9/9 (p = 0.004) |
| medium | nearest vs random | 20.0 m shorter; **10/10** (p = 0.002) | 74.4 s faster; 10/10 (p = 0.002) |
| medium | weighted vs nearest | 0.6 m shorter on average; shorter on 5/9 (p = 0.73) | 13.7 s faster; 7/9 (p = 0.36) |

p: two-sided Wilcoxon signed-rank test on per-seed differences.

### Safety and perception (all 60 runs)

| Metric | easy | medium |
|---|---|---|
| Physical contact events (Gazebo contact sensor) | 0 | 0 |
| Near-collisions (true clearance < 5 cm) | 0 | 0 |
| Smallest true clearance in any run | 10.2 cm | 6.5 cm |
| SLAM pose error vs ground truth: mean of run means / worst sample | 2.6 cm / 17.2 cm | 5.9 cm / 44.1 cm |
| Exit-marker localisation error (median) | 7.7 cm | 16.4 cm |
| Failed navigation goals | 0 | 0 |

### Interpretation

* **Informed selection matters as mazes grow.** On the 6×6 mazes, both `nearest` and `weighted` needed
  about half the distance and time of `random`, and were better on every paired maze. On the 4×4 mazes
  the LiDAR sees most of the maze from a few places, so `random` is barely worse.
* **The weighted score beats nearest-frontier on small mazes and ties it on medium mazes.** On easy
  mazes it was faster on all 10 mazes and shorter on 8. On medium mazes the distances are statistically
  indistinguishable. It was faster on 7 of 9 mazes, but that is not significant with n = 9. **We cannot
  claim the weighted score is better in general from these data.**
* **The one `weighted` failure is perception, not selection.** On medium seed 9 the validated
  exit-marker estimate was 0.48 m off along the wall (typical: 0.08–0.16 m). The robot stopped in the
  neighbouring cell 1.04 m from the true door and correctly reported `EXIT_REACHED` relative to its
  estimate. The ground-truth check rejected it. Likely cause: an oblique, partial first view biased the
  running-mean estimate. Proposed fix (not yet tested): re-estimate the marker from close-range
  observations during the approach.
* `nearest` ends with lower coverage on medium mazes (82.5 % vs 86–91 %). Runs stop when the exit is
  reached, so lower coverage here means less unnecessary exploration, not worse mapping.

### Showcase runs (18 m × 18 m, not part of the comparison)

`difficulty:=showcase`, seed 3: a 12×12 grid of 1.5 m cells, 10 % loops, 16 dead ends, cell-centre
shortest route 57 m, `weighted` strategy, recorded with `record:=true` (`results/showcase/`, `media/`).

| Take | Outcome (verified) | Distance | Time (sim) | Goals / failed / recoveries | Contacts / near | SLAM error mean / max | Marker error |
|---|---|---|---|---|---|---|---|
| 1 | EXIT_REACHED ✔ | 134.5 m | 525.5 s | 74 / 0 / 0 | 0 / 0 | 0.24 m / 0.51 m | 0.35 m |
| 2 (video) | EXIT_REACHED ✔ | 170.3 m | 651.0 s | 91 / 0 / 0 | 0 / 0 | 0.11 m / 0.32 m | 0.33 m |

Both takes ran on the same maze with the same code. The 27 % distance difference is run-to-run
variability from nondeterministic simulator, SLAM and Nav2 timing. On this larger maze the SLAM pose
error grows to decimetres without loop closure, and the marker estimate is correspondingly less
accurate (still within the arrival tolerances).

### Development history (v1 → v2)

Campaign v1 (`results/*_v1`, 60 runs) gave 30/30 on easy and 27/30 on medium (9/10 per strategy). Each
of the 3 medium failures was root-caused and fixed (premature completion with a graph-unreachable
frontier; exit re-targets counted as failures; camera coverage range too long; see
`troubleshooting.md`). The three failing cases were re-run successfully, and then both campaigns were
re-run in full as v2. Earlier pilots (`results/pilot_*`) document the visual-search, SLAM, planner and
preemption fixes. The aborted campaign `results/easy_main_aborted_tf_bug` is kept for transparency:
one of its runs was invalidated by a test process killing its exit detector.
<!-- CAMPAIGNS_END -->

## Proposed future tests (not run)

* Larger mazes (`hard`, 8×8 with loops) and a sensitivity sweep of the score weights.
* Odometry noise injection, plus loop closure re-enabled with stricter thresholds, to measure SLAM
  robustness.
* Multiple exits / decoy colours to stress the perception validation.
* Real-time-factor > 1 to speed up campaigns. Check first that SLAM and Nav2 timing hold up.
