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

## 12. Three different crashes that all print the same message

`Crash: Disarming: AngErr=NN>30` is ArduPilot's crash detector, and it
is the *last* line of three unrelated failures. Diagnosing it means
reading what came immediately before it, not the message itself.

**(a) EKF failsafe.** The give-away is EKF chatter preceding it:

```
EKF3 lane switch 1  ->  EKF variance: position lost
  ->  EKF Failsafe: changed to Land Mode  ->  EKF3 IMU0 stopped aiding
  ->  Crash: Disarming: AngErr=64>30
```

The vehicle lost its position solution and fell; the attitude error is
a consequence, not a cause. Look upstream at the ExternalNav pose
(entries 13 and 14 below).

**(b) The host suspended.** Closing a laptop lid freezes every process
at once. Cartographer stops publishing, EKF3 sees its only position
source vanish, and the same failsafe chain runs. Nothing in the
software can survive this — run long simulations under
`systemd-inhibit --what=handle-lid-switch:sleep:idle`.

**(c) An actual wall strike.** No EKF messages at all, just arm,
takeoff, then `AngErr=85>30, Accel=0.1<3.0`. The low acceleration
alongside a large attitude error means the vehicle is lodged against
geometry rather than falling freely.

**On inflation, honestly.** Only (c) is a navigation-tuning problem,
and it is a genuine tradeoff rather than a bug with a right answer.
Measured on this maze, a quarter of the navigable area has under 0.6 m
of wall clearance, so `inflation_radius: 0.7` puts entire corridors
inside the cost gradient and the controller stalls with `Failed to make
progress`. Dropping to 0.5 removes the stalls and produced a wall
strike within a minute. The setting shipped is 0.7 — occasional stalls
are cheaper than crashes — but the real problem is that DWB is a
ground-robot controller flying a multicopter: it assumes velocity
commands are tracked almost immediately, while the copter banks and
overshoots corners. Matching DWB's acceleration limits to the vehicle,
or using a controller aware of its dynamics, is the correct fix and
was not attempted here.

---

## 13. Scan matching slides down corridors when the prior is under-weighted

**Symptom.** Exploration looked excellent — 205 m² mapped in eight
minutes — while Cartographer's pose quietly diverged from ground truth:
3.1 m, then 4.6, 5.0, 6.2 m, growing monotonically. Critically, the
error was almost entirely along **one axis**; the perpendicular axis
stayed accurate to a few centimetres throughout.

**Cause.** That asymmetry is the fingerprint of scan-matching
degeneracy. Travelling down a long corridor, the lidar sees the same
two parallel walls no matter how far along you are, so the geometry
does not constrain longitudinal position at all. The motion prior is
what holds the estimate steady through such stretches — and the
inherited `ardupilot_cartographer` config weights it at

```lua
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.2  -- default 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight    = 5    -- default 40
```

roughly fifty times below Cartographer's defaults, so the optimiser is
free to slide the solution along the degenerate direction.

**Fix.** Restore the defaults (10 / 40).

**Caveat worth stating.** `use_odometry = true`, and in simulation the
odometry the `ros_gz` bridge supplies is Gazebo ground truth. Leaning
harder on that prior is therefore flattering: on real hardware the
prior would be IMU or visual-inertial odometry, which is standard
practice but not free. The SLAM here is not purely lidar-derived.

**Why it went unnoticed for so long.** Nothing in the ROS logs
complains. The map looks plausible in RViz, the vehicle flies
purposefully, and the explorer happily plans against a corrupted map —
it had been silently degrading every run in the session, including one
that "succeeded".

---

## 14. Watch localization against ground truth, not the flight

The single most useful diagnostic added to this project compares
Cartographer's `map -> base_link` against Gazebo's true model pose and
alarms past a threshold (`docs`-adjacent helper, run every 45 s):

```
truth=(-7.93,5.79) slam=(-4.81,5.80) error=3.12m
```

Two things this catches that watching the simulation cannot. First,
silent divergence: the vehicle looks fine while the map rots. Second,
the *shape* of the error, which localises the cause — pure single-axis
growth means corridor degeneracy (entry 13), whereas a transient spike
in the direction of travel that recovers when the vehicle settles is
just pose latency and is harmless.

Expect transients of 1–1.5 m at 0.4 m/s; alarm above that.

---

## 15. Judging the map too soon after arriving

**Symptom.** Every frontier the vehicle reached was immediately
condemned as "persists after arrival" and blacklisted, one every
couple of seconds — a machine for discarding good frontiers.

**Cause.** `/map` is published at **1 Hz**, but the check ran on the
next evaluation ~1 s after arrival, so it usually tested the *same map
that predated the arrival* — evidence gathered before the vehicle got
there and saw anything.

**Fix.** Only run the check once a map whose stamp is newer than the
arrival time is in hand, with a timeout so a stalled map topic cannot
block exploration.

---

## 16. Everything with `use_sim_time` pays for a 1 kHz clock

**Symptom.** Three trivial Python nodes each sat at ~47% CPU
regardless of how much data they actually handled — suspiciously
uniform. Nav2 then logged `Control loop missed its desired rate of
20.0000Hz` and the vehicle stalled with `Failed to make progress`.

**Cause.** Gazebo publishes `/clock` at **1001 Hz** (`max_step_size`
0.001, real-time factor 1). Every node with `use_sim_time: true`
subscribes and runs a callback on all of them; in `rclpy` that
overhead dwarfs the node's real work.

**Fix.** Turn `use_sim_time` off wherever it isn't needed. `pose_relay`
forwards transforms with their original stamps and only reads the
clock to rate-limit; `odom_sanitizer` compares stamps carried by the
messages. Neither needs it. Nodes that stamp outgoing data (the
explorer's goals and markers, `twist_stamper`) genuinely do — and note
the explorer *must* keep it, since it compares map header stamps
against its own clock (entry 15).

Also worth trimming at the source: Cartographer defaulted to
`pose_publish_period_sec = 5e-3` (200 Hz) feeding a relay that
throttles to 50 Hz anyway.

---

## 17. An exploration boundary must sit inside the walls

**Symptom.** The vehicle left the maze entirely, and once outside, in
open ground with no walls in lidar range, the pose estimate diverged
by more than 10 m and never recovered.

**Cause.** The maze spans ±10 m and the boundary was set to ±11 m — a
metre *outside* the outer wall — so goals beyond the maze opening were
legal. Worse, the boundary is evaluated in the map frame, so once SLAM
diverges the check is being applied in a frame that no longer
corresponds to the world: the guard fails exactly when it is needed.

**Fix.** ±9.5 m, comfortably inside the wall. A geofence enforced on a
drifting estimate is not a real geofence; the only robust version of
this check would use a source independent of the estimate it protects.

---

## 18. Frontier goals land where the robot is forbidden to be

**Symptom.** The vehicle would fly to within ~0.7 m of its goal, hover
facing a wall, burn through four Nav2 recoveries and abort the goal.

**Cause.** A frontier borders unknown space, and in a maze unknown
space usually abuts a wall — so the frontier cell nearest the cluster
centroid is often within the costmap's *inscribed radius*
(`robot_radius`, 0.35 m) of one. The planner cannot place the vehicle
there at all, the path stops short, and the controller grinds toward a
pose it can never occupy.

Measured across captured maps, this was the common case, not an edge
case: on one map **all six** goals sat 0.05–0.10 m from a wall, and on
another six of eleven did.

**Fix.** Rank a cluster's cells by clearance and take the nearest to
the centroid among those with real room (`min_goal_clearance`, 0.5 m),
falling back to the roomiest cell in genuinely tight spots. Frontier
counts are unchanged by this — goals move, clusters are not lost.

**Why an earlier attempt at this failed.** It measured clearance with
`occupied_min = 65`, so it was computing distance to the *fog* around
frontiers rather than to walls, and produced incoherent results that
looked like a broken idea rather than a broken threshold. Same root
cause as entry 7. A constant reused with the wrong meaning.

---

## 19. Two selection rules, and the worse one won

**Symptom.** Goal choices looked poor despite a carefully measured
utility function.

**Cause.** An earlier "momentum" heuristic — continue toward whichever
frontier is nearest the goal just invalidated, by straight-line
distance — ran *ahead* of the utility ranking and returned early. It
had been added to stop Euclidean nearest-first zigzagging, a problem
later solved properly by ranking on travel cost and information gain.
It was never removed. Since most goals end as "already mapped", the
crude rule was overriding the measured one on the majority of
decisions.

**Fix.** Delete it. One selection rule: unknown area revealed per metre
actually flown.

**Lesson.** A heuristic added to compensate for a weak cost function
becomes actively harmful once the cost function is fixed. Patches
deserve removal dates.

---

## 20. Don't demand a heading from a vehicle with a 360-degree sensor

**Symptom.** As entry 18 — arrival never registered, recoveries, abort.

**Cause.** The explorer set each goal's yaw to the bearing from
wherever the vehicle happened to be at dispatch, and Nav2's goal
checker required arrival within 0.25 m *and* 0.25 rad of it. By arrival
that heading is stale and arbitrary. With `acc_lim_y = 0` DWB must yaw
the copter into it, which is slow, so the vehicle can sit inside the
position tolerance indefinitely without satisfying the angular one.

**Fix.** `yaw_goal_tolerance: 3.15` — unconstrained. The lidar sees the
same thing whichever way the vehicle faces, so the heading requirement
bought nothing and cost a manoeuvre that frequently failed. Position
tolerance also relaxed to 0.5 m: a frontier is a region to get the
sensor near, not a pose to hit.

---

## 21. Nav2 recoveries assume a ground robot

Recoveries are what the behaviour tree runs when navigation fails —
clear the costmaps, spin in place, back up, wait — before retrying and
eventually aborting. They are worth understanding here because **a
recovery spin looks exactly like the vehicle being stuck**, and much of
what appeared to be a controller struggling to turn was in fact Nav2
having already declared failure.

They also suit the vehicle poorly. A spin is slow on a copter with
limited yaw rate; a backup moves it blind toward whatever is behind it;
and the maze frequently lacks room for either — one attempt logged
`Collision Ahead - Exiting Spin`. Trimming the list to `wait` plus
costmap clearing is worth considering, so a transient failure costs a
pause rather than a risky manoeuvre.

---

## 22. Depth-first order can be starved by a frontier that never clears

**Symptom.** Late in run 65 the vehicle ping-ponged 10 m up and down a
single wall face at x ≈ -6.9 — goals at y = -5.04, 4.81, -1.04, -2.64,
5.31, 5.71, -4.94 in that order — for 30 consecutive evaluation cycles,
then landed at 267.6 m², 20 m² short of the previous run. The largest
frontier on the map, a 1045-cell opening into the completely unexplored
west corridor, was never selected once.

**Cause.** Frontier identity is by proximity: a cluster within
`frontier_match_radius` (2.0 m) of a remembered one keeps its discovery
sequence number, anything else is "newly discovered" and gets the next
one. That is sound for a frontier that recedes as you map toward it.
It breaks for a frontier whose unknown sits behind an *unconfirmed*
wall, which can never be cleared: it is rebuilt every cycle with a
centroid that shifts further than the match radius, so it re-registers
as new, and the depth-first rule — prefer the newest — hands it the
lead again. Recency is self-renewing for exactly the frontiers that
deserve it least.

**What it was not.** The obvious suspects were both wrong, and each
cost real time. The region was not walled off: run 45's saved map has
it 85% free, so it is reachable maze interior. And detection was not at
fault either — replaying the captured run-65 map, the current detector
*does* produce that 1045-cell frontier, and adding a travel-reachability
filter (only seed from unknown that touches the robot's connected
observed-passable component) yields an identical candidate set, 8
clusters with the same goals. The bug was entirely in ranking.

**Fix.** `rank_candidates()` in `frontier_search.py`. While any
unvisited frontier remains, order is unchanged: new ground, then
newest-discovered, then nearest. Once every candidate lies in
already-flown ground the vehicle is backtracking for leftovers, where
depth-first order carries no information — so rank by cluster size,
then distance. That cannot be starved, because a frontier that stays
unclearable does not grow. Regression tests assert the west-corridor
case survives 40 consecutive re-registrations of the unclearable one.

**Generalisation.** Any "prefer the most recent" rule needs either a
bound on re-registration or a mode where it stops applying. Otherwise
whatever regenerates fastest wins, and what regenerates fastest is
usually what is broken.

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

**Change one thing per run.** Runs cost 10–15 minutes. Two parameters
were once changed together, the run crashed, and neither could be
attributed — costing another full run purely to isolate. With
expensive experiments the temptation to batch changes is exactly
backwards.

**Thresholds picked by intuition are a recurring source of bugs.**
Three separate failures in this project trace to a number chosen
because it sounded reasonable: an occupancy of 65 for "wall" (real
walls are 99-100, and the fog around a frontier has a median of 81, so
every ray was blocked and every frontier rejected); a boundary of ±11 m
around a ±10 m maze; and an inflation radius of 0.7 m in corridors
where a quarter of the space has under 0.6 m of clearance. In every
case the map data needed to choose correctly was already available.
Measure the distribution first.

**Capture the map to a file and iterate offline.** Snapshotting `/map`
to `.npy` and testing the detector against real data turned
hypothesis-per-flight (10+ minutes) into hypothesis-per-second, and
every regression test for the subtle detector bugs came from those
snapshots.

**A method-existence smoke test catches refactor breakage cheaply.**
An `AttributeError` on a helper renamed during a refactor killed the
explorer *in flight*, several minutes into a run. Unit tests did not
cover the ROS node. A three-line assertion that the expected methods
exist runs in the same second as the build.
