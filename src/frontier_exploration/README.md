# frontier_exploration

Frontier-based autonomous exploration for a GPS-denied UAV, built on
ROS 2 Humble, Nav2, Google Cartographer, and ArduPilot SITL with Gazebo.

An Iris quadcopter with a 360° 2D lidar takes off inside an unmapped
maze, explores it on its own, and lands when nothing is left to map —
with **no GPS anywhere in the loop**. ArduPilot's EKF3 fuses
Cartographer's SLAM pose as ExternalNav for position, velocity, and yaw.

**Validated end to end in SITL**: a complete run maps ~224 m² of maze
interior (the full navigable area), then lands and disarms by itself.

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
above a minimum size becomes a candidate whose goal is the frontier cell
nearest the cluster centroid (so goals always sit on known-free ground
even for concave clusters).

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
services, plus a nearest-first frontier policy re-evaluated every
`eval_period` seconds as the map updates:

- The candidate nearest the vehicle is dispatched to Nav2 as a
  `NavigateToPose` goal.
- **Momentum**: when a goal is invalidated because its area got mapped
  in transit, the successor is chosen nearest the *old goal* rather than
  nearest the vehicle, so the copter keeps pushing outward instead of
  flapping between opposite sides of the map.
- **Preemption**: a goal whose frontier has been mapped away while in
  transit is cancelled and replaced.
- **Blacklisting**: goals that Nav2 aborts, that time out, or that the
  vehicle *reaches* while the frontier survives (proof that standing
  there does not map it) are blacklisted so the policy moves on.
- **Retry**: if only blacklisted frontiers remain, the blacklist is
  cleared for another attempt (twice) before exploration is declared
  finished — a frontier may be reachable from a vantage point discovered
  later.
- **Boundary**: candidates outside `bound_*` are ignored. The maze world
  has an opening in its outer wall, and without this the vehicle leaves
  and explores the unbounded world outside forever.
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
| `goal_invalidate_dist` | 1.0 | A goal is preempted when no frontier cell remains this close (m) |
| `goal_timeout` | 90.0 | Blacklist a goal not reached within this many seconds |
| `blacklist_radius` | 0.8 | Candidates within this distance (m) of a blacklisted point are skipped |
| `empty_evals_before_land` | 5 | Consecutive empty evaluations before landing |
| `bound_min/max_x/y` | ±11.0 | Exploration boundary in the map frame |
| `ap_ns` | `/ap/v1` | ArduPilot DDS namespace |

Nav2 tuning lives in `config/navigation.yaml`. The inflation radius
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
