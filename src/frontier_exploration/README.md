# frontier_exploration

Frontier-based autonomous exploration for a GPS-denied UAV, built on
ROS 2 Humble, Nav2, Google Cartographer, and ArduPilot SITL with Gazebo.

An Iris quadcopter with a 360° 2D lidar takes off inside an unmapped
maze, explores it on its own, and lands when nothing is left to map —
with **no GPS anywhere in the loop**. ArduPilot's EKF3 fuses
Cartographer's SLAM pose as ExternalNav for position, velocity, and yaw.

**Validated end to end in SITL**: the maze comes out **complete** —
367–368 m² of a ~390 m² navigable interior — across three consecutive
runs, after which the vehicle lands and disarms by itself. Peak
localization error against Gazebo ground truth stays under 1 m in a
typical run, and every coverage figure here is quoted with a
ground-truth drift check beside it, because the project's highest
recorded coverage and its worst divergence looked identical on paper
(see `docs/INTEGRATION_NOTES.md`).

Goal counts vary widely between runs of identical configuration — 25
to 58 for the same finished map — so treat coverage and the failure
counts as the reproducible numbers and goal counts as noise.

## How it works

```
   Gazebo (maze + 2D lidar)
        │  /scan, /odometry, /clock            ros_gz bridge
        ▼
   ┌───────────────┐   /odometry/filtered   ┌──────────────────┐
   │ odom_sanitizer├───────────────────────▶│  Cartographer    │
   └───────────────┘                        │  (2D SLAM)       │
                                            └────┬────────┬────┘
                                          /map   │        │ /tf (odom→base_link)
                        ┌──────────────────────◀─┘        └─▶┌────────────┐
                        │ frontier_explorer                  │ pose_relay │
                        └───────┬──────────┘                 └──────┬─────┘
                    NavigateToPose  │                    /ap/v1/tf  │
                                    ▼                               ▼
                            ┌──────────────┐  /ap/v1/cmd_vel  ┌──────────────┐
                            │     Nav2     ├─────────────────▶│  ArduPilot   │
                            └──────────────┘                  │  SITL, EKF3  │
                                                              │  ExternalNav │
                                                              └──────────────┘
```

### Frontier detection — `frontier_search.py`

Pure NumPy, no ROS dependencies, so it is unit-testable in isolation.

A frontier cell is a free cell of the occupancy grid that borders
unknown space. Cells are clustered by 8-connectivity, and each cluster
above a minimum size becomes a candidate. Its goal is placed on a
frontier cell with real clearance from walls, nearest the centroid among
those — frontier cells border unknown space, which usually abuts a wall,
so the centroid-nearest cell is often inside the costmap's inscribed
radius where the planner cannot place the vehicle at all.

Two details matter on real Cartographer maps and are the difference
between this working and finding nothing at all:

- **Unknown dilation.** Cartographer never places free cells directly
  against unknown ones; a band of intermediate-probability cells always
  separates them. Plain free/unknown adjacency therefore finds *zero*
  frontiers. Unknown space is grown toward free space (default 4 cells,
  masked at walls) before intersecting with it. The band is not a fixed
  width — thin where the lidar swept closely, ten cells or more where
  an area was glimpsed from a distance — so too short a reach leaves
  whole corridors invisible.
- **Line-of-sight filtering.** A reach as wide as a wall lets frontiers
  leak *through* one — a 0.2 m maze wall is 4 cells at 0.05 m/cell — so
  the vehicle is sent into one corridor to map the next. Each candidate
  is verified with a Bresenham line to nearby unknown cells; it
  survives only if some line reaches unknown space without crossing a
  barrier.
- **Two thresholds, not one.** The ray test uses `LOS_WALL_MIN` (65),
  deliberately lower than the `WALL_MIN` (90) that defines a wall
  everywhere else. These are different questions and no single value
  answers both: growing unknown must be permissive or the fog over a
  corridor's opening blocks it, while testing for a barrier must be
  strict or a half-observed wall stops nothing. Fog has to count as
  passable for the first and as a barrier for the second.

Fragments of a frontier band are merged when within `merge_gap` cells of
each other, so a broken one-cell-wide band is one cluster rather than a
dozen noise-sized ones.

### Exploration policy — `explorer_node.py`

A state machine (`WAIT_INTERFACES → SET_MODE → ARM → TAKEOFF → CLIMB →
EXPLORE → LAND → DONE`) driving the vehicle through the ArduPilot DDS
services, plus a frontier policy re-evaluated every `eval_period`
seconds as the map updates:

- The top-ranked candidate is dispatched to Nav2 as a
  `NavigateToPose` goal.
- **Two ranking modes**, in `rank_candidates()`. While any candidate
  sits in ground the vehicle has not flown, order is: somewhere new
  first, then anything revealing at least `min_unknown_gain_m2`, then
  newest-discovered (depth-first), then nearest by travel cost. New
  frontiers appear where the vehicle is currently revealing space, so
  preferring the newest drives it down one branch and falls back to
  the most recently deferred opening when that branch ends.
- Once *every* candidate lies in already-flown ground, the vehicle is
  collecting leftovers and depth-first order carries no information —
  worse, it is exploitable, since a frontier that can never be cleared
  re-registers as "newly discovered" every cycle and holds the lead
  forever. Ranking then switches to unknown revealed per metre flown,
  which cannot be starved that way because an unclearable frontier
  does not grow.
- Travel cost throughout is a breadth-first expansion **around walls**,
  not straight-line distance. In a maze the two differ wildly —
  measured on a real map, the nearest frontier by straight line needed
  33 m of flying while another at the same apparent distance needed
  18 m. An unroutable candidate is ranked last rather than dropped: a
  coarse estimate that is wrong costs a detour one way and ends the
  mission early the other.
- **Preemption**: a goal whose frontier has been mapped away while in
  transit is cancelled and replaced.
- **Blacklisting**: goals that Nav2 aborts, that time out, or that the
  vehicle *reaches* while the frontier survives (proof that standing
  there does not map it) are blacklisted so the policy moves on.
- **Retry**: if only blacklisted frontiers remain, the blacklist is
  cleared for another attempt (twice) before exploration is declared
  finished — a frontier may be reachable from a vantage point discovered
  later.
- **Boundary**: candidates outside `bound_*` are ignored, and the bounds
  sit inside the outer wall. Note this guard is evaluated in the map
  frame, so it fails exactly when SLAM diverges — it is a convenience,
  not a safety mechanism. `worlds/maze_closed.sdf` is the real remedy:
  the stock maze has a 3 m gap in its east wall, and outside the walls a
  2D lidar has nothing to scan-match against, so the pose estimate
  diverges and takes the map and EKF with it.
- When no reachable candidate remains for `empty_evals_before_land`
  consecutive evaluations, the vehicle switches to LAND and the node
  reports the run summary once disarmed.

Frontier cells, candidate goals, and the active goal are published as
RViz markers on `/frontier_explorer/frontiers`.

### GPS-denied state estimation — `pose_relay.py`, `config/frontier_ekf3.parm`

The GPS driver and the simulated GPS are removed entirely
(`GPS1_TYPE 0`, `SIM_GPS1_ENABLE 0`). Cartographer's `odom → base_link`
transform is relayed from `/tf` to `/ap/v1/tf`, where ArduPilot's DDS
interface feeds it into `AP_VisualOdom` and EKF3 consumes it as
ExternalNav (`EK3_SRC1_POSXY/VELXY/VELZ/YAW = 6`, altitude from the
barometer).

With no GPS fix there is nothing to anchor the filter, so at startup the
explorer sends **both** `SET_GPS_GLOBAL_ORIGIN` and `MAV_CMD_DO_SET_HOME`
over MAVLink. Both are required: without home, arming fails with
"AHRS: waiting for home".

### Supporting nodes

- **`odom_sanitizer.py`** — the `ros_gz` odometry bridge occasionally
  emits consecutive messages with identical timestamps, which trips a
  fatal `CHECK` inside Cartographer (`map_by_time.h`). This republishes
  `/odometry` as `/odometry/filtered` with strictly increasing stamps.
- **cmd_vel relay** (launch-only `topic_tools relay`) — the stock
  `twist_stamper` publishes to `/ap/cmd_vel`, but this ArduPilot build
  subscribes under `/ap/v1/cmd_vel`.
- **`scripts/drift_check.py`** — prints Cartographer's estimate against
  Gazebo ground truth. Worth running during any change to SLAM: a
  divergent run still looks purposeful on screen while the map quietly
  rots, and area figures *above* the true maze size are the tell.

### SLAM configuration — `config/cartographer.lua`

Two departures from the upstream `ardupilot_cartographer` config, both
forced by the maze:

- `ceres_scan_matcher` translation/rotation weights are restored to
  Cartographer's defaults (10 / 40). Upstream uses 0.2 / 5, roughly
  fifty times weaker, which lets the solution slide along a corridor —
  scan matching is degenerate along the corridor axis, and the estimate
  drifted over 6 m in one direction while staying centimetre-accurate
  in the other.
- Global loop closure is **disabled** (`optimize_every_n_nodes = 0`).
  Every corridor looks like every other corridor, so the matcher finds
  convincing but wrong correspondences and each accepted one rewrites
  the trajectory. A deliberate trade: local drift now accumulates
  uncorrected, which is tolerable in a bounded 20 m maze and would not
  be in a large or looping environment.

## Requirements

A built [ArduPilot ROS 2 workspace](https://ardupilot.org/dev/docs/ros2.html)
with `ardupilot_gz` and `ardupilot_ros`, on ROS 2 Humble with Gazebo
Harmonic, plus `ros-humble-navigation2`, `ros-humble-cartographer-ros`,
`ros-humble-twist-stamper`, `ros-humble-topic-tools`, and
`python3-pymavlink`.

## Run

```bash
cd ~/ardu_ws
colcon build --packages-select frontier_exploration
source install/setup.bash
ros2 launch frontier_exploration explore.launch.py
```

The copter arms, climbs to 2 m, explores the maze, and lands by itself.
A full run takes roughly 10–15 minutes of wall time.

Launch arguments: `gui:=false` (no Gazebo GUI), `rviz:=false`,
`takeoff_alt:=2.5`.

To watch the algorithm rather than just the vehicle, add the
`/frontier_explorer/frontiers` MarkerArray display in RViz: frontier
cells in cyan, candidate goals as orange spheres, active goal as a green
cylinder.

## Key parameters

| Parameter | Default | Purpose |
|---|---|---|
| `takeoff_alt` | 2.0 | Exploration altitude (m), below the 3.25 m maze walls |
| `unknown_dilation` | 4 | Cells to grow unknown space toward free space. Set to the thinnest wall in cells (0.2 m = 4 at 0.05 m/cell): shorter and openings behind a wide fog band are invisible, longer and fog over an unconfirmed wall becomes a frontier on its far side |
| `min_frontier_size` | 10 | Smallest cluster treated as a real frontier |
| `min_goal_clearance` | 0.5 | Keep goals this far (m) from walls, or the planner cannot place the vehicle there at all |
| `gain_radius` | 6.0 | Radius (m) over which unknown area is counted when scoring a frontier |
| `goal_invalidate_dist` | 1.5 | A goal is preempted when no frontier cell remains this close (m); kept in step with Nav2's `xy_goal_tolerance` |
| `goal_timeout` | 90.0 | Blacklist a goal not reached within this many seconds |
| `blacklist_radius` | 2.5 | Candidates within this distance (m) of a blacklisted point are skipped. Large frontiers get a much narrower exclusion — see `large_frontier_blacklist_radius` |
| `los_occupied_min` | 65 | Occupancy that blocks a *sight line*, deliberately below the 90 that defines a wall. One constant could not serve both: growing unknown must be permissive, testing for a barrier must be strict |
| `empty_evals_before_land` | 5 | Consecutive empty evaluations before landing |
| `bound_min/max_x/y` | ±9.9 | Exploration boundary in the map frame — just inside the ±10 m outer wall. Take the extent from the world file, never from a map: a map cannot tell you how big the world is when the unexplored part is what you are measuring |
| `ap_ns` | `/ap/v1` | ArduPilot DDS namespace |

Nav2 tuning lives in `config/navigation.yaml`. Two settings there
matter more than the rest. `xy_goal_tolerance` is 1.5 m and
`yaw_goal_tolerance` is unconstrained: a frontier is a region to get
the sensor near, not a pose to hit, and with a 360° lidar the heading
is irrelevant. Demanding tighter arrival caused the vehicle to hover
short of goals it had effectively reached, cycle through Nav2
recoveries and abort — arrivals rose from 4% to 81% of goals when this
was relaxed. The inflation radius
(1.0 m) and speed cap (0.4 m/s) are deliberately conservative: with the
stock values the copter clips maze corners and ArduPilot triggers its
crash detector (`Crash: Disarming: AngErr=44>30`).

Three further settings there are not tuning but corrections, and each
cost several runs to find:

- `lethal_cost_threshold: 90`. At the stock 50 the costmap treats
  Cartographer's fog as solid wall — two thirds of its "walls" were
  fog — and with a 1 m inflation the planner refuses freshly explored
  ground entirely. Measured, no threshold in the fog band separates
  wall from free (cells at 80–89 resolve free about twice as often as
  wall), which is why the next two matter more than this one.
- `obstacle_max_range: 20.0` / `raytrace_max_range: 25.0`. The Nav2
  defaults are 2.5 m and 3.0 m against a 30 m lidar, so beyond 2.5 m
  the planner knew about walls only through the probability guess
  above.
- `min_obstacle_height: -5.0` / `max_obstacle_height: 10.0`, on **both**
  costmaps. The defaults are 0.0–2.0 m applied to the observation's z
  in the costmap frame — fine for a ground robot at 0.2 m, fatal for a
  copter flying at exactly `takeoff_alt` 2.0 m. Every scan point was
  silently discarded and the obstacle layer contributed *nothing*: the
  count of cells it added over the static map was exactly zero.

## Tests

```bash
python3 -m pytest src/frontier_exploration/test -q
```

38 tests covering frontier detection (including the Cartographer grey
rim and thin-wall leak cases that caused real failures), clustering,
line-of-sight filtering, ranking in both modes, and grid-to-world
conversion.

Run them from the package directory. `install/` is a plain copy rather
than a symlink install, so once `install/setup.bash` is sourced,
`import frontier_exploration` resolves to the *installed* copy and
running pytest from the workspace root tests the last build instead of
the working tree.

## Further reading

`docs/INTEGRATION_NOTES.md` documents the non-obvious ArduPilot/ROS 2
integration issues found while building this, each with the symptom that
surfaced it and the fix.
