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

## 23. A dilation wider than a wall invents frontiers on the far side

**Symptom.** Run 70 landed having mapped 275.2 m² and left 25.1 m² of
the south-east corridor — 91% of it — untouched, after dispatching
twelve goals to `(7.76, -3.75)` and neighbouring points. Each reported
`Frontier reached` within seconds and then `Frontier receded rather
than cleared`, forever.

**Cause.** That goal sits *south* of the wall at y = -3.5, in corridor
the vehicle had already mapped. It was attacking the unexplored
corridor through a wall. `unknown_dilation` had been raised to 6 while
the maze's walls are 0.2 m — 4 cells at 0.05 m/cell — so the dilation
reaches clean through them and marks free cells on this side as
bordering the unknown on the far side. Entry 8 is the same failure;
raising the reach from 3 to 6 to see across wide fog banks reopened it.

The corridor's real opening, at its west end near `(-1.4, -1.5)`, was
detected the whole time as a 21-cell cluster — 93× smaller than the
false one, so it never won a ranking.

**Fix.** Back to `unknown_dilation = 3`. Replaying run 70's final map:

| Dilation | Clusters | Pointing into the ground actually missed |
|---|---|---|
| 2 | 3 | 3 (under-detects overall) |
| 3 | 12 | 4 |
| 4 | 8 | 2 |
| 6 | 12 | 1 |

**Caveat.** This narrows the false attractor rather than removing it:
goals hard against a wall face still appear at 3. The line-of-sight
filter is what is supposed to catch them, and it lets them through
wherever the mapped wall has an unobserved gap for a ray to slip.

**Update: 3 was too short, and the setting is now 4.** Runs 74 and 75
both landed with the same 50.4 m² southern corridor 97% unknown, and
replaying run 75's final map at the moment it chose to land shows the
detector was not being overruled — it had nothing to offer:

| Dilation | Clusters on the whole map | Into the missed corridor |
|---|---|---|
| 3 | 1 | 0 |
| 4 | 12 | 1 (66 cells) |
| 5 | 18 | 1 (165 cells) |
| 6 | 19 | 1 (257 cells) |

The corridor's opening lies behind a fog band wider than three cells,
so at reach 3 it is invisible. 4 is the least reach that sees it, and
equals the wall thickness rather than the 1.5× that caused the
through-wall goals above. Run 76 then mapped the whole maze —
368.2 m² with unknown cells down 62% against the previous best.

**The general shape.** Too short and openings behind fog are
invisible; too long and fog over an unconfirmed wall becomes a
frontier on its far side. Both failures cost a corridor, and the
parameter has to be set to the environment's thinnest wall, not to
how much fog you wish you could see across.

**Generalisation worth keeping.** Any reach across unknown space needs
a bound tied to the thinnest barrier in the environment, not to how
much fog you want to see across. The two requirements conflict, and
the wall wins — a frontier you cannot reach is worse than one you
cannot see, because you will fly at it repeatedly.

---

## 24. The costmap reads Cartographer's fog as wall

**Symptom.** Run 71 stranded. Every goal aborted with `status 6` about
21 s after dispatch, from a vehicle hovering in open space with 1.0 m
of clearance, while `Failed to make progress` never appeared once. It
landed at 328.5 m² with the whole south-east quadrant visible on the
map and unreachable.

**Cause.** `Failed to create a plan` × 28: the *global planner* was
refusing, so the controller never received a path — which is why the
controller-side warning is absent and why the vehicle simply sat
there. Comparing `/global_costmap/costmap` against `/map` at the same
instant, goals reading free in the map (0 and 20) carried cost 99,
inscribed, in the costmap.

`navigation.yaml` had `lethal_cost_threshold: 50`. Measured on run
71's final map:

| Occupancy | Cells | What it is |
|---|---|---|
| 0-25 | 131367 | free |
| 26-49 | 2506 | light fog |
| 50-89 | 7562 | **fog, treated as lethal wall** |
| 90-100 | 3839 | real wall |

Two thirds of what the costmap called wall was fog, and with
`inflation_radius: 1.0` each of those 7562 cells painted a metre-wide
halo — 47945 cells at inscribed cost against 11401 genuinely lethal.

**Fix.** `lethal_cost_threshold: 90`. Simulating NavFn's constraint
against that same map — dilate everything called lethal by the 0.35 m
inscribed radius, then flood-fill from where the vehicle was stranded:

| Threshold | Plannable area | Refused goals now reachable |
|---|---|---|
| 50 | 244.0 m² | 0/4 |
| 65 | 253.0 m² | 2/4 |
| 80 | 258.5 m² | 4/4 |
| 90 | 268.6 m² | 4/4 |

Run 72 then mapped 368.8 m² of a ~390 m² navigable maze with **zero**
Nav2 aborts, against 16 in run 70 and 13 in run 71.

**Why it hid for so long.** The failure scales with how well
exploration works. Fog is what a map is made of before it settles, so
the harder the vehicle pushes into new ground the more of the costmap
becomes false wall. Run 70 never triggered it because it never got
anywhere new; fixing entry 23 is what exposed it.

**Same root as entry 18.** A constant reused with the wrong meaning.
There it was `occupied_min = 65` measuring distance to fog instead of
to walls; here it is Nav2's lethal threshold set below the fog band.
Both were picked as "about half of 100".

---

## 25. ...and the over-correction routes through walls

**Symptom.** Watching run 72 in RViz: planned paths crossing walls.

**Cause.** Entry 24's fix, overshot. At `lethal_cost_threshold: 90`
everything reading 50-89 is passable, and a wall seen only at distance
or at a grazing angle has not yet reached 90. The planner cannot see
walls it has only partially observed.

**Fix, not yet applied.** 80. The replay table in entry 24 shows it
unlocking the same 4/4 refused goals as 90 while keeping the 80-89
band as obstacle. The 10 extra points bought no access and gave away a
third of the evidence for a wall's existence.

**Status.** Run 72 flew clean — no crash detector, no collision, no
EKF failsafe — but that is one run, and the trade is real: the more
permissive the threshold, the more the planner will confidently route
through geometry it has merely glimpsed. Occupancy is evidence, and
this parameter sets how much evidence a wall needs before it is
allowed to stop you.

**The shape of the pair.** Entries 24 and 25 are one decision seen
from both sides — believe fog and you cannot move, disbelieve it and
you fly into things. Anything between 80 and 89 buys access at the
price of wall fidelity, and the right value is the one where the fog
band ends, measured on a real map, not a round number.

---

## 26. The obstacle layer was switched off by its height filter

**Symptom.** After entry 25, the planner still routed through walls at
every threshold tried — 50, 80 and 90 — and setting the obstacle
layer's ranges to sensor range changed nothing at all.

**Cause.** The layer was contributing nothing whatsoever. Comparing
`/global_costmap/costmap` against `/map`, the count of cells lethal in
the costmap but *not* wall in the static map — everything the obstacle
layer adds — was exactly **0**, and the costmap's lethal total (2834)
equalled the map's wall count (2834) to the cell.

`ObstacleLayer` defaults to `min_obstacle_height: 0.0` and
`max_obstacle_height: 2.0`, applied to the observation's z in the
costmap frame. That is written for a ground robot whose lidar sits at
0.2 m. This vehicle **flies at `takeoff_alt`, 2.0 m**, so every scan
point landed exactly at the ceiling of the window and was discarded.
The local costmap set the same 2.0 explicitly, so the controller was
equally blind.

**Fix.** `min_obstacle_height: -5.0`, `max_obstacle_height: 10.0` on
both costmaps. Live scan marks went from 0 to 559 immediately on the
next run.

**Why the ranges mattered too, but second.** The defaults are
`obstacle_max_range: 2.5` and `raytrace_max_range: 3.0` against a 30 m
lidar, so even with the height filter open the planner would only have
learned about obstacles within 2.5 m. Both settings are wrong for this
vehicle; the height filter is what made the layer inert.

**Generalisation.** A costmap layer that silently contributes nothing
looks exactly like a costmap layer that is working — no warning, no
error, everything simply relies on the static map. The check is one
query: count the cells the layer adds over the static map, and if it
is zero, the layer is off, whatever the config says.

---

## 27. Fog cannot be classified by any threshold

**Method.** Take a mid-run map and the final map from the *same* run
(so the frames align), and ask what each fog cell became.

| Fog band | Cells | → free | → wall | → unresolved |
|---|---|---|---|---|
| 26-49 | 2231 | 73% | 3% | 24% |
| 50-64 | 3762 | 40% | 7% | 53% |
| 65-79 | 1555 | 33% | 18% | 50% |
| 80-89 | 1351 | 36% | 19% | 45% |

**Result.** No value in the band separates wall from free. Even at
80-89 — the most wall-like fog there is — a cell is nearly twice as
likely to resolve free as to resolve wall.

**Consequence.** Entries 24 and 25 were both attempts to find a
threshold that works, and neither could have succeeded: 50 blocked
everything, 90 believed nothing, 80 failed in both directions at once.
The parameter is being asked a question the data cannot answer.

**What actually resolves it** is not a better threshold but a
different source of evidence — live scans marking what the sensor is
looking at right now (entry 26). Occupancy probability answers "what
has this cell usually looked like"; the planner needs "is there a wall
there now", and only the sensor knows.

---

## 28. "Routing through walls" was mostly planning through the unknown

**Symptom.** Paths visibly crossing walls in RViz, reported across
four consecutive runs and blamed in turn on the lethal threshold
(entry 25) and the inert obstacle layer (entry 26).

**Cause.** Neither, mostly. Sampling the live `/plan` against the map
settles it in one query:

```
plan: 544 poses
  through KNOWN WALL (map >= 90):    0
  through fog (50-89):               5
  through UNKNOWN (unmapped):      221
  through free:                    318
```

Not one pose crossed a known wall. 41% of the path ran through
*unexplored* space, because `track_unknown_space` defaults to false
and unknown cells are therefore treated as **free**. The planner draws
confident straight lines across regions it has never seen, and where a
wall happens to stand in one, the path crosses it. On screen that is
indistinguishable from ignoring a wall it can see.

**Status.** Unresolved. `track_unknown_space: true` is the candidate,
but frontier goals sit on the unknown boundary by definition, so the
planner must still be permitted to enter unknown space or the run-71
stranding returns in a new form.

**Lesson, and it cost four runs.** Two plausible mechanisms were
available and both were real defects, so fixing them felt like
progress and the symptom persisting felt like the fix being
insufficient. The question "what is the path actually crossing" was
answerable at any point in about forty seconds. Reach for the
observation that discriminates between hypotheses before fixing the
one that seems most likely.

---

## 29. One constant answering two questions, for the fourth time

**Symptom.** Watching run 77: "it's using a frontier marker in
corridor 2 to map corridor 1." Goals dispatched into an
already-mapped corridor, pointing at unknown space on the far side of
the wall beside it. Arrive, frontier survives, three arrivals, blacklist.

**Cause.** `unknown_dilation` is 4 and the maze walls are 4 cells
thick (entry 23), so unknown in one corridor reaches exactly far
enough to touch free space in the next. The line-of-sight filter
exists to reject precisely this — but it took its wall threshold from
the same `occupied_min` as everything else, 90, and a wall the vehicle
has seen once from a distance sits in the fog band well below that.
It blocks no ray, so the false frontier survives.

**Why no threshold fixed it.** Sweeping the shared constant at
dilation 4, against two captured maps holding the two failure cases:

| `occupied_min` | corridor behind fog found | through-wall goal gone |
|---|---|---|
| 50 | no | yes |
| 65 | no | yes |
| 75 | no | yes |
| 82 | no | partly |
| 90 | **yes** | **no** |

Nothing satisfies both, because the constant is being asked two
incompatible questions. *Growing* unknown toward free space must be
permissive or the fog band over a corridor's opening stops the unknown
from ever reaching a free cell. *Testing whether a barrier stands
between a goal and its unknown* must be strict or a half-seen wall
blocks nothing. Fog has to count as passable for one and as a barrier
for the other.

**Fix.** Split them: `LOS_WALL_MIN = 65` for the ray test,
`WALL_MIN = 90` everywhere else, threaded through `find_frontiers` as
`los_occupied_min`. With the dilation mask held at 90, tightening only
the ray test to 65 dropped 8 of 12 clusters on run 75's map, and every
one sat hard against an outer wall or across the interior wall at
y = -3.5. The four that survived were interior, including the corridor
a shorter dilation cannot see at all.

**Result.** Run 78 mapped the maze complete in **25 goals** — the
fewest of any run, against 35 and 51 for the same map — with **zero**
frontiers dismissed after arrival, the signature this was aimed at. It
needed no goal timeouts and no persistence dismissals at all; its 3
aborts and 61 planner refusals all landed in the endgame, after the
map was already complete, and belong to the inflation problem below.

**The pattern, now four times over.** `occupied_min = 65` measuring
distance to fog instead of wall (entry 18). `lethal_cost_threshold`
asked to separate fog from wall when the data cannot (entries 24-27).
Nav2's obstacle-layer height window written for a ground robot and
applied to a flying one (entry 26). And this. Every time, a single
number was serving two purposes that had quietly diverged, and every
time the search for a better value failed because no value existed.
When a sweep shows every setting failing in a different direction,
stop sweeping and ask what two things the parameter is being asked to
mean.

**Still open.** `inflation_radius: 1.0` puts roughly a third of the
navigable area at inscribed cost, and it is what the endgame refusals
are. But the count across three runs of otherwise identical
configuration went 112, 5, 61 — so it depends heavily on where the
vehicle happens to finish, and one run's figure is not evidence.

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

**Simulate the planner's constraint before flying the fix.** The
offline-replay habit above applies past the detector. Entry 24's fix
was chosen by reimplementing NavFn's rule against a captured map —
dilate every cell the costmap calls lethal by the inscribed radius,
flood-fill from the vehicle, ask which goals survive — and the model
reproduced the observed failure exactly (0/4 goals reachable at the
shipped threshold, matching a 100% abort rate). Agreement on the
*broken* configuration is what makes the predicted fix trustworthy;
without it the sweep is just four numbers. It also priced the
alternatives in the same pass, which is where the evidence that 80
suffices came from.

**Distrust a coverage figure that has no ground-truth check beside
it.** Run 72's 368.8 m² would look like the run 35 divergence (332.9
m², a fabrication) without the drift check running alongside it. Two
numbers, always together: what you mapped, and how far the estimate
was from truth when you mapped it.
