# Integration notes

Non-obvious problems hit while bringing up frontier exploration on
ArduPilot SITL + ROS 2 Humble + Cartographer + Nav2, with the symptom
that surfaced each one and the fix. Most cost more time than the
algorithm itself, and none of them are in the tutorials.

Environment: ROS 2 Humble, Gazebo Harmonic (gz-sim 8), ArduPilot with
`AP_DDS` over micro-ROS/UDP, `ardupilot_gz` maze world, Iris + 2D lidar.

---

## 1. AP_DDS lives under `/ap/v1`, not `/ap`

**Symptom.** Service clients for `/ap/arm_motors` never became ready, so
the explorer sat in `WAIT_INTERFACES` forever. More insidiously,
velocity commands were published to `/ap/cmd_vel` with no subscriber:
Nav2 looked healthy, the vehicle simply never moved.

**Cause.** This ArduPilot build exposes a versioned DDS namespace. The
services are `/ap/v1/arm_motors`, `/ap/v1/mode_switch`,
`/ap/v1/experimental/takeoff`; the topics are `/ap/v1/tf`,
`/ap/v1/cmd_vel`, `/ap/v1/pose/filtered`, `/ap/v1/status`. Documentation
and the stock `ardupilot_cartographer` launch files still use `/ap/*`.

**Fix.** An `ap_ns` parameter (default `/ap/v1`) on the explorer, and a
`topic_tools relay` from `/ap/cmd_vel` to `/ap/v1/cmd_vel` in the launch
file, since the stock `twist_stamper` invocation is otherwise fine.

**Diagnosis tip.** `ros2 topic info /ap/v1/cmd_vel` showing
`Publisher count: 1, Subscription count: 1` is the check that the whole
command path is connected. A count of 0 on either side localises the
break immediately.

---

## 2. GPS-denied arming needs origin *and* home

**Symptom.** `Arm: AHRS: waiting for home`, repeatedly, despite EKF3
reporting `origin set` and `is using external nav data`.

**Cause.** With GPS disabled there is no fix to derive home from.
`SET_GPS_GLOBAL_ORIGIN` anchors the EKF but does not set home, and
Copter refuses to arm without one.

**Fix.** The explorer sends both over MAVLink at startup:
`SET_GPS_GLOBAL_ORIGIN` (retried until `GPS_GLOBAL_ORIGIN` is echoed
back), then `MAV_CMD_DO_SET_HOME` with an explicit location (param1 = 0,
lat/lon/alt in params 5/6/7) until acknowledged.

---

## 3. SITL's `eeprom.bin` silently overrides the defaults file

**Symptom.** `Arm: AHRS: EK3 sources require GPS`, even though
`frontier_ekf3.parm` was passed via `defaults:=` and set
`EK3_SRC1_POSXY 6`. Reading the live parameter showed `3` (GPS).

**Cause.** SITL persists parameters in `eeprom.bin` in the launch
working directory. Any parameter ever set explicitly in that workspace
wins over the defaults file on subsequent runs, and the mismatch is
silent — some parameters from the file applied, this one did not.

**Fix.** Set the value over MAVLink once (which persists it), or delete
`eeprom.bin` / pass `wipe:=True` to start genuinely clean. Worth
checking the *live* values with `param_request_read` rather than
trusting the `.parm` file whenever behaviour contradicts configuration.

---

## 4. Cartographer aborts on duplicate odometry timestamps

**Symptom.** `cartographer_node` died with SIGABRT:

```
F map_by_time.h:43] Check failed: data.time > std::prev(trajectory.end())->first
  (621355970032000000 vs. 621355970032000000)
```

**Cause.** The `ros_gz` odometry bridge occasionally emits consecutive
messages carrying the same stamp. Cartographer's `MapByTime` requires
strictly increasing time and treats a violation as fatal, not as a
message to drop.

**Fix.** `odom_sanitizer.py` republishes `/odometry` as
`/odometry/filtered`, dropping any message whose stamp is not strictly
greater than the previous one; Cartographer subscribes to the filtered
topic.

**Rejected alternative.** Setting `use_odometry = false` also avoids the
crash, but see the next entry.

---

## 5. Scan-only Cartographer breaks ExternalNav, and the failure looks like a takeoff bug

**Symptom.** Takeoff was accepted (`DDS: Request for Takeoff : SUCCESS`)
but the vehicle stayed at z ≈ −0.02 m and eventually auto-disarmed. The
only hint was `PreArm: VisOdom: not healthy` earlier in the log.

**Cause.** With `use_odometry = false`, Cartographer's pose extrapolator
updates only on scan matches, so the `odom → base_link` transform
becomes too sparse and jittery for `AP_VisualOdom` to consider healthy.
EKF3 then has no usable position source and Copter will not climb.

**Fix.** Keep `use_odometry = true` and feed it the sanitized odometry
topic. The lesson: a stalled pose stream presents as a *flight control*
failure several layers away from its cause.

---

## 6. Nav2 goal results arrive after the goal is gone

**Symptom.** A goal was cancelled and replaced, then the *previous*
goal's `ABORTED` result landed and blacklisted the *new* goal's
coordinates — poisoning good frontiers at random.

**Cause.** `NavigateToPose` results are asynchronous. Cancelling does
not prevent a late result callback for the old goal handle.

**Fix.** Every dispatch increments `_goal_seq`; the send and result
callbacks capture that value and return early when it no longer matches
the current sequence.

---

## 7. Frontier detection finds nothing on a real Cartographer map

**Symptom.** The vehicle took off and landed twelve seconds later,
announcing "No reachable frontiers remain" with a map that was clearly
mostly unexplored.

**Cause.** Textbook frontier detection — a free cell 4-adjacent to an
unknown cell — returns exactly **zero** cells on a Cartographer
occupancy grid. Cartographer separates free space from unknown with a
rim of intermediate-probability cells, so free and unknown are never
neighbours.

Measured on a captured map by dilating unknown space before
intersecting:

| Dilation (cells) | Frontier cells found |
|---|---|
| 1 | 0 |
| 2 | 290 |
| 3 | 1052 |
| 4 | 1803 |

**Fix.** `detect_frontier_cells(..., unknown_dilation=3)`.

**Method note.** The productive move was capturing `/map` to a `.npy`
file mid-run and iterating on the detector offline against real data,
rather than re-flying to test each hypothesis. Both regression tests for
this case came from that snapshot.

---

## 8. Dilation leaks frontiers through thin walls

**Symptom.** After the dilation fix, the vehicle reached frontier after
frontier that never disappeared from the map — `Frontier persists after
arrival` roughly once every ten seconds — and coverage crawled.

**Cause.** Maze walls are 0.2 m thick, i.e. 4 cells at 0.05 m/cell.
Dilating unknown space by 3 cells reaches *through* a wall, so free
cells on this side looked adjacent to unknown space on the far side.
No amount of hovering can map that unknown space, so the goal stayed
pending until blacklisted.

**Fix.** `frontier_sees_unknown()`: a Bresenham line from the candidate
goal to each of the nearest unknown cells; the candidate survives only
if one line reaches unknown space without passing an occupied cell.

Effect on two captured maps:

| Map snapshot | Clusters before | After LOS filter |
|---|---|---|
| mid-run | 9 | 2 |
| stalled run | 9 | 1 |

---

## 9. Reaching a frontier that does not clear = livelock

**Symptom.** Goals #4 through #24 all dispatched to the same point,
each reported "Frontier reached" within two seconds, forever.

**Cause.** The vehicle arrived, the frontier survived (see above), and
nearest-first immediately re-selected it as the closest candidate.

**Fix.** Record the goal on arrival; if that frontier is still present
at the next evaluation, blacklist it. Arriving is proof that presence
alone will not map it.

---

## 10. Greedy nearest-first zigzags

**Symptom.** Consecutive goals alternated between opposite corners:
`(3.2, 3.4) → (3.4, 6.8) → (5.2, 2.0) → (0.8, 4.2)`. The copter spent
its time in transit rather than at frontiers.

**Cause.** Frontiers usually recede *outward* as the lidar sweeps. Each
time a goal was invalidated, the new nearest-to-vehicle candidate was
often behind it.

**Fix.** Momentum: after an invalidation, prefer the candidate nearest
the *invalidated goal* (within `_continue_radius`, 3 m) over the one
nearest the vehicle.

---

## 11. The maze has a door

**Symptom.** "It's out of the maze." Coverage kept increasing while the
vehicle explored open world outside the walls, and would never finish
because unbounded empty space always presents new frontiers.

**Fix.** `bound_min/max_x/y` (default ±11 m) filters candidates by
position in the map frame. Not a workaround for a bug — any real
exploration mission needs a boundary.

---

## 12. Nav2 clips corners and ArduPilot's crash detector fires

**Symptom.** `Crash: Disarming: AngErr=44>30, Accel=0.0<3.0` mid-flight,
followed by `Arm: Leaning` refusing to re-arm.

**Cause.** With stock inflation and a 0.5–0.7 m/s speed cap, paths hug
maze corners closely enough that the copter strikes a wall.

**Fix.** In `config/navigation.yaml`: `inflation_radius: 0.7`,
`cost_scaling_factor: 3.0` on both costmaps, `max_vel_x/y: 0.4`,
`max_speed_xy: 0.5`, and `allow_unknown: false` on the planner so paths
never route through unmapped space.

---

## Operational lessons

**Orphaned processes are the sneakiest failure mode.** `ros2 launch`
killed via its parent does not always take its children with it. Four
`frontier_explorer` nodes from previous runs once ran concurrently, each
preempting the others' goals through the same action server —
presenting as "Nav2 aborts every goal instantly". Verify with
`ros2 node list` (duplicate node names) and tear down by explicit PID
before every run:

```bash
ros2 node list | sort | uniq -d   # duplicates = orphans still alive
```

**`--symlink-install` does not reload a running node.** Python sources
are symlinked, but an already-running process keeps the old code in
memory. Rebuilding while the stack flies changes nothing until the node
restarts.

**Mid-flight node restarts are worth supporting.** Being able to kill
and relaunch just the explorer, without landing and rebuilding the whole
world, shortened the iteration loop enormously. Two small provisions
make it work: treat an already-set EKF origin as success (ArduPilot
broadcasts `GPS_GLOBAL_ORIGIN` on the first set only, so re-request it
explicitly), and skip takeoff when already at altitude, or the second
takeoff command climbs on top of the current height.

**A method-existence smoke test catches refactor breakage cheaply.**
An `AttributeError` on a helper renamed during a refactor killed the
explorer *in flight*, several minutes into a run. Unit tests did not
cover the ROS node. A three-line assertion that the expected methods
exist runs in the same second as the build.
