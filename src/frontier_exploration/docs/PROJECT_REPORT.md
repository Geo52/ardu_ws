# Frontier exploration: what was built and what was learned

A narrative companion to the code. `README.md` describes how to run the
system; `INTEGRATION_NOTES.md` catalogues individual bugs with symptoms
and fixes. This document explains the shape of the problem, why it was
harder than it looks, and where things actually stand.

---

## 1. What the system does

An Iris quadcopter with a 360° 2D lidar is placed in an unmapped maze in
Gazebo. With **no GPS anywhere in the loop**, it takes off, explores the
maze on its own, and lands when nothing worth mapping remains.

Four pieces, each of which can fail independently:

| Layer | Role | Software |
|---|---|---|
| SLAM | Build the map, estimate pose | Cartographer (2D) |
| State estimation | Fuse that pose for flight control | ArduPilot EKF3, ExternalNav |
| Navigation | Drive the vehicle to a point | Nav2 (NavFn + DWB) |
| Exploration | Decide *which* point | `frontier_exploration` (this package) |

Only the fourth is original work. Most of the effort went into the fact
that the other three were configured for a different vehicle, a
different sensor regime, or a different environment.

---

## 2. The core algorithm

A **frontier** is the boundary between mapped-free and unmapped space —
the edge of what you know. Fly to one, and by definition you learn
something. Repeat until none remain, and you have explored everything
reachable.

That is the whole idea, and it is deceptively simple. The interesting
parts are all in the details:

**Detecting frontiers.** Textbook definition: a free cell adjacent to
an unknown cell. On real Cartographer maps this finds *exactly zero*,
because Cartographer never places free cells directly against unknown
ones — a band of partially-observed cells always separates them. The
detector has to reach across that band, but not through walls.

**Choosing between them.** Distance, information gain, discovery order,
and whether the vehicle has already been there all matter, and they
conflict. Several policies were tried; the current one is depth-first
(see §4).

**Knowing when to stop.** "No frontiers remain" is the natural
termination condition, and it is what makes the automatic landing work.
It is also easy to trigger spuriously, which ends a mission early.

---

## 3. Why it was hard: four categories of failure

Framing the session's problems this way is more useful than a
chronological list.

### 3.1 Inherited configuration written for something else

The largest single category. The upstream `ardupilot_cartographer` and
Nav2 defaults are sane for their original context and wrong here:

- **Scan matcher prior weights** at 0.2/5 against Cartographer's
  defaults of 10/40. Fifty times too weak, which let the pose estimate
  *slide along a corridor* — scan matching is geometrically degenerate
  along the corridor axis, so with a near-zero prior weight the
  solution drifted over 6 m in one direction while staying
  centimetre-accurate in the other.
- **Lateral acceleration limit of 0**, describing a differential-drive
  robot. A quadcopter is holonomic; with this it had to stop and yaw at
  every corner.
- **A goal heading requirement**, meaningless for a 360° sensor but
  expensive to satisfy, which prevented arrival from ever registering.
- **`/clock` at 1 kHz** from Gazebo, with every `use_sim_time` node
  paying a callback for each tick. Three trivial Python nodes were
  burning ~47% CPU each on nothing, starving Nav2's control loop.

### 3.2 Assumptions that held in one place and not another

- A **fixed 3-cell reach** across the partially-observed band. Measured
  once, in an area the lidar had swept closely. Where an area was
  glimpsed from a distance the band runs ten cells or more, and those
  regions produced *no frontier at all* — a third of the maze was
  invisible to the detector while the vehicle recycled the few crisp
  frontiers it could see.
- An **occupancy threshold of 65** for "wall" when ray casting. Real
  walls read 99–100; the fog around a frontier has a median of 81. Every
  ray was blocked by fog, so every frontier was rejected and a run ended
  after 34 seconds.
- **Exploration bounds of ±11 m** around a ±10 m maze — a metre outside
  the wall, which let the vehicle leave through the maze's opening.
  Outside, a 2D lidar has nothing to scan-match against, so the pose
  diverged past 10 m and took the map, the EKF and the bounds check
  itself down with it. The correction to ±9.5 then overshot the other
  way and quietly amputated a third of the maze for six runs (§7) —
  the same parameter, wrong in both directions, because it was never
  once checked against the world file.

### 3.3 Mechanisms that fought each other

Each was added to fix something real, and then interacted badly:

- Arrival tolerance was raised to 1.5 m to fix a genuine problem
  (arrivals never registering, so goals died in recovery loops — this
  took arrivals from 4% to 81% of goals). But "arrived" no longer meant
  "close enough to clear the frontier", which silently turned the
  persistence check into a **corridor-abandonment machine**: one bite
  of each corridor, then blacklist.
- A **momentum heuristic** was added to stop Euclidean nearest-first
  zigzagging. Travel-cost ranking later solved that properly, but the
  heuristic stayed and kept *overriding* the better policy on most
  decisions.
- A **progress watchdog** meant to break loops could not distinguish
  "stuck" from "flying a long way to a distant frontier", so it
  blacklisted the far corridor it was on its way to.

The pattern: a rule reasoning about one thing (a frontier, a timer)
while the failure lives in its interaction with something else
(tolerance, transit time).

### 3.4 The vehicle is not what Nav2 expects

Nav2 is a 2D ground-robot stack. Two consequences that cost the most
time:

- **Nothing controlled altitude.** Nav2 commands x, y and yaw and
  leaves the vertical channel at zero; nothing else filled it in. The
  vehicle sagged out of the air — six times in one run — and a takeoff
  re-issued to an already-airborne copter does not climb, which hung
  the mission outright.
- **DWB plans trajectories the copter cannot track.** Its recoveries
  (spin in place, back up) assume a robot that can pivot cheaply and
  reverse safely; in a 2 m corridor one attempt logged `Collision Ahead
  - Exiting Spin`. Much of what looked like the controller struggling
  was in fact Nav2 having already declared failure and started a
  recovery.

---

## 4. The exploration policy, and how it got there

Four policies were tried in order. Each fixed the previous one's
failure and introduced its own:

1. **Euclidean nearest-first.** Straight-line distance is a poor proxy
   in a maze: a frontier 3 m away through a wall may need a 20 m
   detour. Measured on a real map — the nearest candidate by straight
   line required 33 m of flying while another at the same apparent
   distance required 18 m.
2. **Travel cost + information gain**, scored as unknown area revealed
   per metre actually flown, with distance from a breadth-first
   expansion around walls. Better, but it abandons a half-explored
   corridor: little unknown area remains in a narrow space, so a large
   distant region outscores it and the vehicle leaves mid-corridor.
3. **Nearest by travel, information gain breaking ties.** Removes that
   failure, since whatever you are exploring is the nearest thing to
   you. Still had no memory of *which* opening was uncovered last, so
   two similar corridors traded the lead whenever one was partly mapped.
4. **Depth-first (current).** Each frontier keeps a discovery sequence
   across evaluation cycles, matched by proximity so it survives
   centroid drift. The newest is chosen. New frontiers appear where the
   vehicle is currently revealing space, so this drives it down one
   branch; when the branch dead-ends, the newest surviving frontier is
   whatever was deferred most recently — which is the backtrack.

Layered on top:

- **Visited-area avoidance.** Residual fog is left along the flight
  path itself, so traversed corridors keep generating legitimate
  frontiers. These rank behind everything in new ground and are
  collected only once nothing new remains.
- **Corridor persistence.** A frontier is blacklisted only after
  surviving three arrivals *at the same spot*. One survival means it
  receded as you mapped toward it, which is progress.
- **Goal placement.** On a known-free cell with real wall clearance —
  frontier cells sit inside the costmap's inscribed radius surprisingly
  often, where the planner cannot place the vehicle at all — and among
  those, the one facing the most unknown.

---

## 5. Results

The maze is **20 × 20 m, 400 m² gross**, taken from the world file
(`Wall_1` at y = −10 spans the full width). Every coverage figure
below that predates run 69 was quoted against a denominator of
~290 m², measured from occupancy grids that were themselves missing
the unexplored third — see §7. Treat the older percentages as
flattering; the absolute areas are sound.

| Run | Coverage | Outcome | Note |
|---|---|---|---|
| 16 | 224 m² | landed | first complete mission |
| 44 | 257.6 m² | landed | 49/60 goals reached |
| 45 | **291.7 m²** | landed | 65/72 goals, peak drift 0.25 m |
| 52 | 266.4 m² | landed | first run with zero altitude sags |
| 63 | 231 m² | stalled | fogged corridors undetected |
| 64 | 288.1 m² | plateaued | fog-aware detection; broke the 231 stall |
| 65 | 267.6 m² | landed | 42/52 goals; starved on an unclearable frontier |
| 67 | 271.6 m² | stopped | starvation fixed; west corridor still out of bounds |
| 68 | 322.9 m² | stopped | bounds widened in x; west corridor + SW block ~95% |
| 69 | 299.5 m² | stopped | **southern corridor traversed, 98%**; drift 0.07 m |

One cautionary result: **run 35 reported 332.9 m²** and looked like a
record. It was a corrupted map — localization had diverged past 7 m and
the same physical space was drawn twice. Coverage exceeding the known
maximum is a symptom, not an achievement.

---

## 6. Verification, and why it mattered

Two habits caught more than any amount of reasoning:

**Compare against ground truth, not against appearances.**
`scripts/drift_check.py` prints Cartographer's estimate beside Gazebo's
true pose. A divergent run looks purposeful on screen while the map
quietly rots; twice a run was reported as a success before this check
existed. The *shape* of the error localises the cause: growth along a
single axis means corridor degeneracy, while a transient spike in the
direction of travel that settles is harmless pose latency.

**Capture the map and iterate offline.** Snapshotting `/map` to `.npy`
turned hypothesis-per-flight (ten-plus minutes) into
hypothesis-per-second. Every subtle detector fix — the fog band, the
wall threshold, the speck filter — was found and validated this way,
and every regression test came from a real captured map rather than a
synthetic one.

---

## 7. Where it stands

The system reliably takes off, explores most of the maze with a
centimetre-accurate map, and lands on its own. Best complete run:
291.7 m² with an autonomous landing. Most recent: 288.1 m².

**The real blocker was the exploration boundary, and it hid for six
runs.** `bound_min_x = -9.5` on a maze whose free space reaches
x = −10.0 discarded a strip around the entire perimeter — 14.0 m² of
already-mapped floor — as goal candidates. Inside that strip sat the
1021-cell frontier opening into the west corridor, the largest on the
map, dropped silently every evaluation cycle. The bounds filter was
the one place a frontier left the pipeline without logging anything,
so the symptom was only ever "it never goes there."

Widening it (run 68) immediately produced the west corridor and the
SW block at ~95%, and revealed that the maze extends to y = −10, not
y = −7: an interior wall (`Wall_4`, x ∈ [−7, 10]) stops 3 m short of
the west wall, leaving a 60 m² southern corridor whose only entrance
is that gap. Run 69 traversed it end to end at 98%.

**Two bad measurements, same mistake.** The "~290 m² of navigable
space" figure and a later `bound_min_y = -7.0` were both derived from
occupancy grids — and those grids stopped where they did *because the
region beyond had never been explored*. Sizing a world from a map of
it is circular whenever the unexplored part is what you are trying to
size. The world file was authoritative and available the whole time.

**The earlier fixes were real but not the cause.** Detection was never
the problem here: replaying the captured run-65 map, the detector
already produced that 1021-cell frontier, and a travel-reachability
filter yields an identical candidate set. The depth-first starvation
was genuine and measured, but fixing it moved coverage 267.6 → 271.6,
not into the corridor. Finding a culprit that explains the observed
behaviour is not the same as finding the one that matters.

**Superseded diagnosis, kept for the record.** It is the west corridor —
x ∈ [−10, −7.4], running north-south from y ≈ 1.5 down to y ≈ −7, plus
the area it opens into in the southwest. Run 45 mapped it (85% free in
that box); runs 44, 64 and 65 left it at 0–1%. So it is genuinely
reachable maze interior, not a sealed void, and the earlier guess that
a wall stood between the vehicle and it was wrong.

**Why later runs miss it: depth-first order can be starved.** Frontier
identity is by proximity — a cluster within `frontier_match_radius` of
a remembered one keeps its sequence number. A frontier whose unknown
lies behind an unconfirmed wall *never clears*, so it is rebuilt every
cycle with a centroid that shifts further than that radius, registers
as newly discovered, and wins the newest-first tiebreak again. On run
65 an unclearable line at x ≈ −6.9 held the lead for 30 consecutive
evaluations while the largest frontier on the map — a 1045-cell
opening straight into the west corridor — was never once selected. The
vehicle ping-ponged 10 m up and down a wall face until it landed at
267.6 m².

Detection was never the problem here: measured on the captured run-65
map, the current detector *does* offer that 1045-cell frontier, and
adding a travel-reachability filter changes the candidate set not at
all (8 clusters either way, identical goals). The failure was entirely
in ranking.

**Fix applied:** once every remaining frontier lies in already-flown
ground, the vehicle is backtracking to collect leftovers and
depth-first order no longer means anything. In that mode candidates are
ranked by how much they reveal, then by distance — which cannot be
starved, because a frontier that stays unclearable does not grow.
`rank_candidates()` in `frontier_search.py`, with regression tests.

**Also unresolved:** DWB is still a ground-robot controller flying a
multicopter. Occasional stalls and the recovery behaviours are symptoms
of that mismatch, and no amount of parameter tuning removes it — the
honest fix is a controller aware of the vehicle's dynamics.

---

## 8. Lessons worth keeping

- **A number better than the known maximum is a bug, not a triumph.**
- **Thresholds picked by intuition are a recurring source of failure.**
  Three separate bugs traced to one: 65 for "wall", ±11 m for the
  maze bounds, 0.7 m for inflation. In every case the data needed to
  choose correctly was already in the maps.
- **Change one thing per run when a run costs ten minutes.** Batching
  changes twice cost a whole run to disentangle afterwards.
- **A heuristic added to compensate for a weak cost function becomes
  harmful once the cost function is fixed.** Patches need removal
  dates.
- **Don't let an approximation make an irreversible decision.** A
  coarse reachability estimate used as a hard filter discarded three of
  four real frontiers and ended a mission on a map 90 m² explored.
  Ranking wrong costs a detour; vetoing wrong ends the run.
- **Watching the vehicle is not verification.** It flies convincingly
  while the map is being destroyed.
- **Check an old success before theorising about a failure.** Two
  sessions were spent assuming the missing region was walled off. One
  comparison against run 45's saved map — which had mapped it at 85% —
  showed it was reachable all along and moved the search from detection
  to ranking, where the bug actually was.
- **Any "prefer the most recent" rule can be starved by something that
  regenerates.** The unclearable frontier was not selected because it
  looked good; it was selected because it kept looking *new*. Recency
  needs a bound, or a mode where it stops applying.
- **Never size a world from a map of it.** Both the 290 m² figure and
  a bound of y = −7.0 came from occupancy grids that stopped where
  exploration stopped, not where the maze does. The reasoning is
  circular exactly when the unexplored part is the thing in question,
  and the world file settled it in one query.
- **A filter that drops candidates must say so.** The bounds check was
  the only silent one in the pipeline, and it cost six runs. It now
  warns when it rejects a frontier larger than anything it kept —
  which caught a second instance within ten minutes of being added.
- **Prefer recovering a candidate to discarding it.** A 734-cell
  cluster was thrown away because the single cell chosen to represent
  it fell 3 cm outside the boundary, while hundreds of its cells sat
  in open space. Re-pick, don't reject.
