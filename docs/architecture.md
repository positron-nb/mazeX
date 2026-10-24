# Architecture

MazeMind runs entirely in simulation. Everything below runs in **ROS 2 Humble** inside
**WSL2 Ubuntu 22.04**, with **Gazebo Ignition Fortress 6.18** (the Gazebo officially paired
with Humble through `ros_gz` 0.244).

## Verified software versions

| Component | Version (as installed and tested) |
|---|---|
| OS | Ubuntu 22.04.5 LTS on WSL2 (kernel 6.6.114.1-microsoft-standard-WSL2), WSLg display |
| ROS 2 | Humble (Debian packages, built Aug 2026) |
| Gazebo | Ignition Gazebo Fortress `libignition-gazebo6` 6.18.0 |
| ros_gz (`ros_gz_sim`, `ros_gz_bridge`) | 0.244.25 |
| Nav2 (`navigation2`) | 1.1.20 |
| SLAM Toolbox | 2.6.10 |
| RViz2 | 11.2.28 |
| Python / NumPy / SciPy / OpenCV / Matplotlib | 3.10.12 / 1.24.3 / 1.8.0 / 4.8.0 / 3.5.1 |

No Gazebo Classic packages are used: the whole stack is Fortress + `ros_gz`.

## Data flow

```
                          ┌──────────────────────── Gazebo (Ignition Fortress) ─────────────────────────┐
 maze generator ─► .sdf ─►│ maze walls + green exit marker     robot: DiffDrive, gpu_lidar, camera,    │
        │                 │                                      contact sensor, OdometryPublisher      │
        │                 └───────────────┬───────────────────────────────┬────────────────────────────┘
        │                                 │ ros_gz_bridge                  │ ros_gz_bridge
        │            /scan /odom /tf /joint_states /camera/*      /ground_truth/odom  /ground_truth/contacts
        │                                 │ /clock  ◄── /cmd_vel           │
        ▼                                 ▼                                ▼
  ground-truth .json          ┌─────────────────────────┐        ┌──────────────────────┐
        │                     │ SLAM Toolbox (online)   │──/map──►│                      │
        │                     │  map → odom TF          │        │                      │
        │                     └──────────┬──────────────┘        │                      │
        │                                │ /map, TF               │   evaluator          │
        │                     ┌──────────▼──────────────┐        │ (ONLY node with       │
        │                     │ Nav2 (planner, RPP,     │        │  ground truth)        │
        │                     │ costmaps, behaviours)   │        │                      │
        │                     └──────────▲──────────────┘        │  coverage, distance, │
        │                     NavigateToPose action               │  contacts, SLAM error│
        │                     ┌──────────┴──────────────┐ status │  exit verification   │
        │                     │ explorer (frontiers,    │────────►│                      │
        │                     │ strategies, recovery)   │ outcome│                      │
        │                     └──────────▲──────────────┘        └──────────┬───────────┘
        │                     /mazemind/exit/detection                       │
        │                     ┌──────────┴──────────────┐                    ▼
        │                     │ exit_detector (camera   │          results/<campaign>/runs/*.json
        │                     │ colour + LiDAR ranging) │
        │                     └─────────────────────────┘
        └──────────────────────────────────────────────────────────────► evaluator (file path parameter)
```

## Packages

| Package | Contents |
|---|---|
| `mazemind_description` | Xacro URDF of the differential-drive robot plus Gazebo sensor and system plugins; `description.launch.py` (robot_state_publisher). |
| `mazemind_simulation` | Maze generator (`maze.py`), SDF writer, ground-truth rasteriser, `generate_maze` CLI, `sim.launch.py` (Gazebo + spawn + bridge). |
| `mazemind_exploration` | Frontier detection, traversability/geodesic planning, the three selection strategies, goal memory and stuck detection, the `explorer` node, exit perception and the `exit_detector` node. |
| `mazemind_evaluation` | Ground-truth metrics, the `evaluator` node, `run_experiments` and `analyze_results`. |
| `mazemind_bringup` | `mazemind.launch.py` (full stack), Nav2 / SLAM / exploration parameters, RViz config. |

## Robot (simulation only)

* Cylindrical chassis, radius 0.14 m, height 0.08 m, mass 2 kg, with inertia tensors from geometry.
* Two drive wheels (r = 0.035 m, separation 0.20 m) on continuous joints. They have sphere collisions
  so the ground contact lies on the wheel mid-plane (see *Troubleshooting → odometry*).
* Front and rear frictionless caster spheres.
* `gpu_lidar`: 360 beams over 360°, 0.12–8 m, σ = 1 cm Gaussian noise, 10 Hz, mounted 0.1175 m above ground.
* RGB camera: 320×240, 1.3 rad HFOV, 10 Hz, forward-facing, publishing in the `camera_optical_link` frame.
* Contact sensor on the chassis collision (ground truth, evaluation only).
* `DiffDrive` system: `/cmd_vel` in, `/odom` and `odom → base_footprint` out.
* `OdometryPublisher` system: the true world pose on `/ground_truth/odom` (evaluation only).

TF tree: `map → odom` (SLAM Toolbox) → `base_footprint` (DiffDrive) → `base_link` → wheels, `lidar_link`,
`camera_link → camera_optical_link` (robot_state_publisher).

## Information boundary

The exploration stack must not see the maze layout. This is enforced in four ways:

1. **Separate data path.** The generated ground-truth JSON's path is passed only to the `evaluator` node's
   parameters (`mazemind.launch.py`). The explorer and exit detector get identical parameters for
   every maze.
2. **Separate topics.** Ground-truth data lives under `/ground_truth/*`. Only `mazemind_evaluation`
   subscribes to those topics.
3. **No dependency.** `mazemind_exploration` has no dependency on `mazemind_simulation` or
   `mazemind_evaluation`. Its only test-time use of the generator is the offline integration test,
   which uses it to build a fake world.
4. **Automated check.** `test_information_boundary.py` fails if exploration source code references
   `ground_truth`, the simulation or evaluation packages, world/model state topics, maze files or
   Gazebo message types.

The exit is recognised from camera pixels and LiDAR ranges, so exit discovery is **autonomous**, not
assisted.

## Run lifecycle

1. `mazemind.launch.py` generates (or reuses) the maze for the requested seed and difficulty, writes
   `~/.mazemind/worlds/maze_*.sdf/.json`, and starts Gazebo, spawn, bridge and robot_state_publisher.
2. After 3 s it starts SLAM Toolbox, after 5 s Nav2, and after 8 s the exit detector and explorer.
3. The explorer waits for `/map`, the `map → base_footprint` TF and the Nav2 action server, then starts
   its clock (`exploration_start`).
4. The explorer publishes a single latched outcome on `/mazemind/outcome`. The evaluator then writes
   `runs/<run_id>.json` and exits. With `shutdown_on_finish:=true` that shuts the whole launch down.
5. `run_experiments` starts each run in its own process group, ROS domain and Gazebo partition, then
   kills the group and sweeps any MazeMind Gazebo stragglers before the next run.
