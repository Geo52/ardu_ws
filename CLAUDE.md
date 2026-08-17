# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this workspace is

A ROS 2 Humble colcon workspace for frontier-based autonomous exploration
of a maze by a GPS-denied UAV, in ArduPilot SITL + Gazebo Harmonic.

`src/frontier_exploration` is the only original package; everything else
under `src/` (`ardupilot`, `ardupilot_gz`, `ardupilot_ros`, `ros_gz`,
`micro_ros_agent`, `sdformat_urdf`, `ardupilot_sitl_models`) is an
upstream checkout providing the simulation stack, and most of it is
untracked in git. Changes belong in `src/frontier_exploration` unless
there is a specific reason otherwise.

## Commands

```bash
# Build (from the workspace root)
source /opt/ros/humble/setup.bash
colcon build --packages-select frontier_exploration
source install/setup.bash

# Full run: Gazebo + SITL + Cartographer + Nav2 + explorer.
# Arms, climbs to 2 m, explores, lands by itself. 10-15 min wall time.
ros2 launch frontier_exploration explore.launch.py
ros2 launch frontier_exploration explore.launch.py gui:=false rviz:=false takeoff_alt:=2.5

# Tear down every process a run starts, and verify none survived.
# Always run this between runs (see "Orphans" below).
src/frontier_exploration/scripts/stop_run.sh

# Localization sanity check, mid-run: Cartographer vs Gazebo ground truth.
python3 src/frontier_exploration/scripts/drift_check.py
```

### Tests

Run them from the **package directory**, not the workspace root:

```bash
cd src/frontier_exploration
python3 -m pytest test -q                                  # all 38
python3 -m pytest test/test_frontier_search.py::test_no_frontiers_in_fully_known_grid -q
python3 -m pytest test -q -k rank                           # by name
```

This matters: `install/` is a plain copy, not a symlink install, so once
`install/setup.bash` is sourced, `import frontier_exploration` resolves
to the *installed* copy. Running pytest from the workspace root therefore
tests the last build, not the working tree. Running it from
`src/frontier_exploration` puts the source ahead on `sys.path`.

Tests cover `frontier_search.py` only — it is deliberately pure NumPy with
no ROS imports so it can be tested and iterated offline against captured
`/map` snapshots. `explorer_node.py` has no test coverage; a
method-existence smoke check is the cheap guard against refactor breakage
killing the node mid-flight.

## Architecture

```
Gazebo (maze + 2D lidar)
  /scan, /odometry, /clock  ──▶ odom_sanitizer ──▶ Cartographer (2D SLAM)
                                                     │        │
                                    /map ────────────┘        └──▶ /tf (odom→base_link)
                                     │                                    │
                          frontier_explorer                          pose_relay
                                     │ NavigateToPose                     │ /ap/v1/tf
                                     ▼                                    ▼
                                   Nav2 ──▶ twist_stamper ──▶ cmd_vel_altitude_hold
                                                                  │ /ap/v1/cmd_vel
                                                                  ▼
                                                  ArduPilot SITL, EKF3 ExternalNav
```

Nodes (entry points in `setup.py`):

- **`explorer_node.py`** — the policy. State machine
  `WAIT_INTERFACES → SET_MODE → ARM → TAKEOFF → CLIMB → EXPLORE → LAND →
  DONE` on a 1 Hz `_tick`, plus `_evaluate` every `eval_period` seconds
  that re-detects frontiers, ranks them, dispatches to Nav2, preempts
  goals whose frontier has been mapped away, and blacklists ones the
  vehicle reaches without clearing. Talks to ArduPilot over DDS services
  under `ap_ns` and sets the EKF origin over MAVLink at startup.
- **`frontier_search.py`** — pure NumPy detection, clustering, goal
  placement, and ranking. No ROS.
- **`pose_relay.py`** — Cartographer `odom→base_link` from `/tf` to
  `/ap/v1/tf`, where `AP_VisualOdom` feeds EKF3.
- **`odom_sanitizer.py`** — republishes `/odometry` as
  `/odometry/filtered` with strictly increasing stamps; duplicate stamps
  are a fatal `CHECK` inside Cartographer.
- **`altitude_hold_relay.py`** — bridges `/ap/cmd_vel` to `/ap/v1/cmd_vel`
  and fills in `linear.z`, which Nav2 (2D) never sets.

Config: `config/cartographer.lua` (scan-matcher weights restored to
Cartographer defaults, global loop closure disabled — corridors all look
alike), `config/navigation.yaml` (Nav2), `config/frontier_ekf3.parm`
(GPS removed, `EK3_SRC1_*` = ExternalNav), `worlds/maze_closed.sdf` (the
stock maze's east wall has a 3 m gap the explorer escapes through).

### Frontier detection, in one paragraph

Textbook free-cell-adjacent-to-unknown finds *zero* frontiers on a
Cartographer grid, because a band of intermediate-probability cells always
separates free from unknown. Unknown space is grown toward free space
(`unknown_dilation`, currently 4 — set to the wall thickness in cells)
before intersecting with it, and because that reaches as far as a 0.2 m
wall, each candidate is verified with a Bresenham line to nearby unknown
cells (`frontier_sees_unknown`). Goals are placed on a cell with real wall
clearance (`min_goal_clearance`), not on the centroid-nearest cell.

**The ray test uses its own threshold.** `LOS_WALL_MIN` (65) is
deliberately below `WALL_MIN` (90): growing unknown must be permissive or
the fog over a corridor's opening blocks it, while testing for a barrier
must be strict or a half-observed wall stops nothing. One constant serving
both sent the vehicle into one corridor to map the next.

### Ranking has two modes

`rank_candidates()` in `frontier_search.py`. While any frontier in
unvisited ground remains: new ground first, then newest-discovered
(depth-first), then nearest. Once every candidate is in already-flown
ground, it switches to cleanup ordering by unknown revealed per metre
flown — depth-first is starvable there, because an unclearable frontier
re-registers as "newly discovered" every cycle and holds the lead forever
(integration note 22).

## Working conventions in this repo

- **`docs/INTEGRATION_NOTES.md` is the authoritative record** — 22 numbered
  entries, each a real failure with its symptom and fix. Read the relevant
  entry before touching SLAM config, Nav2 tuning, the DDS namespace, or
  the ranking rules; most of the non-obvious code exists because of one of
  them. `docs/PROJECT_REPORT.md` is the narrative companion.
- **When a parameter sweep shows every value failing in a different
  direction, stop sweeping.** It means one constant is answering two
  questions that have diverged. This has happened four times here
  (integration notes 18, 24–27, 26, 29): distance-to-wall vs
  distance-to-fog, fog vs wall in the costmap, a ground robot's sensor
  height applied to a flying one, and dilation reach vs sight line. Each
  time the search for a better value failed because none existed.
- **Quote coverage with a ground-truth drift reading beside it.** The
  project's best result and its worst divergence (run 35, 332.9 m² of
  fabricated map) look identical on paper. `scripts/drift_check.py`.
- **Thresholds are chosen from measured data, and the comment says so.**
  Parameter declarations in `explorer_node.py:77-209` carry multi-paragraph
  rationale (which run, which map, what was measured). Preserve that when
  changing a value, and measure before picking a new one — three separate
  bugs in this project trace to a number that merely sounded reasonable.
- **`use_sim_time` is off where it isn't needed.** Gazebo publishes
  `/clock` at ~1 kHz and each subscribed rclpy node burns ~half a core,
  which starves Nav2's control loop. `pose_relay`, `odom_sanitizer` and the
  altitude relay run without it; the explorer needs it (it compares map
  stamps against its own clock) and so does `twist_stamper`.
- **Orphans.** `ros2 launch` does not reliably take its children down.
  Duplicate `frontier_explorer` nodes preempt each other's goals through
  the same action server, which presents as "Nav2 aborts every goal".
  Run `scripts/stop_run.sh` and check `ros2 node list | sort | uniq -d`
  before starting a run.
- **A rebuild does not reload a running node.** Restarting just the
  explorer mid-flight is supported and is the fast iteration loop.
- **Change one thing per run.** Runs cost 10-15 minutes and a run with two
  changes in it attributes nothing.
- **Long runs need `systemd-inhibit --what=handle-lid-switch:sleep:idle`** —
  a host suspend freezes Cartographer, EKF3 loses its only position source,
  and the vehicle crashes.

## Simulation gotchas that cost real time

- **AP_DDS is under `/ap/v1`, not `/ap`.** Services `arm_motors`,
  `mode_switch`, `experimental/takeoff`; topics `tf`, `cmd_vel`,
  `pose/filtered`, `status`. Upstream docs and launch files still say
  `/ap/*`. `ros2 topic info /ap/v1/cmd_vel` showing publisher *and*
  subscriber counts of 1 is the check that the command path is connected.
- **`eeprom.bin` in the workspace root silently overrides the `.parm`
  defaults.** Any parameter ever set explicitly in this workspace wins on
  later runs. When behaviour contradicts `config/frontier_ekf3.parm`, read
  the *live* value over MAVLink; delete `eeprom.bin` or pass `wipe:=True`
  to start clean.
- **Arming with no GPS needs both `SET_GPS_GLOBAL_ORIGIN` and
  `MAV_CMD_DO_SET_HOME`** — origin alone leaves "AHRS: waiting for home".
- **`Crash: Disarming: AngErr=NN>30` is three different failures.** Read
  what precedes it: EKF chatter means the position solution was lost, no
  EKF messages plus low accel means an actual wall strike.
- **The exploration boundary (`bound_*`) is evaluated in the map frame**,
  so it fails exactly when SLAM diverges. It is a divergence trap, not a
  geofence; the sealed world file is what keeps the vehicle inside.
