# frontier_exploration

Frontier-based autonomous exploration for a GPS-denied UAV, built on
ROS 2 Humble, Nav2, Google Cartographer, and ArduPilot SITL with Gazebo.

An Iris quadcopter with a 360° 2D lidar takes off inside an unmapped
maze, explores it on its own, and lands when nothing is left to map —
with **no GPS anywhere in the loop**. ArduPilot's EKF3 fuses
Cartographer's SLAM pose as ExternalNav for position, velocity, and yaw.

**Validated end to end in SITL**: a complete run maps 257.6 m² of maze
interior — essentially all of it — reaching 49 of 60 dispatched goals
with no navigation stalls, then lands and disarms by itself. Peak
localization error against Gazebo ground truth was 1.63 m.

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
  against unknown ones; a ~2-cell rim of intermediate-probability cells
  always separates them. Plain free/unknown adjacency therefore finds
  *zero* frontiers. Unknown space is dilated (default 3 cells) before
  intersecting with free space.
- **Line-of-sight filtering.** Dilating by 3 cells lets frontiers leak
  through thin walls — a 0.2 m maze wall is only 4 cells thick — so the
  vehicle chases unknown space it can never reach from this side. Each
  candidate is verified with a Bresenham line to nearby unknown cells;
  the frontier survives only if some line reaches unknown space without
  crossing an occupied cell.

Fragments of a frontier band are merged when within `merge_gap` cells of
each other, so a broken one-cell-wide band is one cluster rather than a
dozen noise-sized ones.

### Exploration policy — `explorer_node.py`

A state machine (`WAIT_INTERFACES → SET_MODE → ARM → TAKEOFF → CLIMB →
EXPLORE → LAND → DONE`) driving the vehicle through the ArduPilot DDS
services, plus a frontier policy re-evaluated every `eval_period`
seconds as the map updates:

- The highest-utility candidate is dispatched to Nav2 as a
  `NavigateToPose` goal.
- **Utility, not proximity**: candidates are scored by unknown area
  revealed per metre actually flown, with distance measured by a
  breadth-first expansion around walls rather than in a straight line.
  In a maze the two differ wildly — measured on a real map, the nearest
  frontier by straight line needed 33 m of flying while another at the
  same apparent distance needed 18 m.
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
| `unknown_dilation` | 3 | Cells to dilate unknown space; must stay below the thinnest wall thickness in cells |
| `min_frontier_size` | 10 | Smallest cluster treated as a real frontier |
| `min_goal_clearance` | 0.5 | Keep goals this far (m) from walls, or the planner cannot place the vehicle there at all |
| `gain_radius` | 6.0 | Radius (m) over which unknown area is counted when scoring a frontier |
| `goal_invalidate_dist` | 1.5 | A goal is preempted when no frontier cell remains this close (m); kept in step with Nav2's `xy_goal_tolerance` |
| `goal_timeout` | 90.0 | Blacklist a goal not reached within this many seconds |
| `blacklist_radius` | 0.8 | Candidates within this distance (m) of a blacklisted point are skipped |
| `empty_evals_before_land` | 5 | Consecutive empty evaluations before landing |
| `bound_min/max_x/y` | ±9.5 | Exploration boundary in the map frame — must sit *inside* the ±10 m outer wall |
| `ap_ns` | `/ap/v1` | ArduPilot DDS namespace |

Nav2 tuning lives in `config/navigation.yaml`. Two settings there
matter more than the rest. `xy_goal_tolerance` is 1.5 m and
`yaw_goal_tolerance` is unconstrained: a frontier is a region to get
the sensor near, not a pose to hit, and with a 360° lidar the heading
is irrelevant. Demanding tighter arrival caused the vehicle to hover
short of goals it had effectively reached, cycle through Nav2
recoveries and abort — arrivals rose from 4% to 81% of goals when this
was relaxed. The inflation radius
(0.7 m) and speed cap (0.4 m/s) are deliberately conservative: with the
stock values the copter clips maze corners and ArduPilot triggers its
crash detector (`Crash: Disarming: AngErr=44>30`).

## Tests

```bash
python3 -m pytest src/frontier_exploration/test -q
```

14 tests covering frontier detection (including the Cartographer grey
rim and thin-wall leak cases that caused real failures), clustering,
line-of-sight filtering, and grid-to-world conversion.

## Further reading

`docs/INTEGRATION_NOTES.md` documents the non-obvious ArduPilot/ROS 2
integration issues found while building this, each with the symptom that
surfaced it and the fix.
