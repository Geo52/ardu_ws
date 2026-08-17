"""Frontier detection and clustering over an occupancy grid.

Pure NumPy — no ROS dependencies — so it can be unit tested in isolation.

Grid convention (nav_msgs/OccupancyGrid): -1 unknown, 0..100 occupancy
probability. A frontier cell is a free cell with at least one unknown
4-neighbour. Frontier cells are clustered by 8-connectivity; each cluster
above a minimum size becomes a candidate frontier whose goal is the
frontier cell nearest the cluster centroid (the centroid itself may fall
in unknown or occupied space for concave clusters).
"""

import math
from collections import deque
from dataclasses import dataclass, replace
from typing import List, Tuple

import numpy as np

UNKNOWN = -1

# Occupancy at or above which a cell is treated as a solid wall for
# ray casting. Measured on Cartographer maps of the maze: transition
# cells around a frontier reach ~80-95, real walls 99-100.
WALL_MIN = 90


@dataclass
class Frontier:
    """A cluster of frontier cells on the occupancy grid."""

    cells: np.ndarray  # (N, 2) array of (row, col) indices
    centroid: Tuple[float, float]  # (row, col), may lie off the cluster
    goal_cell: Tuple[int, int]  # frontier cell nearest the centroid
    size: int


def _dilate4(mask: np.ndarray, iterations: int) -> np.ndarray:
    """Binary dilation with a 4-connected structuring element."""
    out = mask.copy()
    for _ in range(iterations):
        grown = out.copy()
        grown[1:, :] |= out[:-1, :]
        grown[:-1, :] |= out[1:, :]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        out = grown
    return out


def detect_frontier_cells(
    grid: np.ndarray,
    free_max: int = 25,
    unknown_dilation: int = 1,
    occupied_min: int = WALL_MIN,
) -> np.ndarray:
    """Return a boolean mask of frontier cells.

    A frontier cell is free (0 <= p <= free_max) and within
    ``unknown_dilation`` cells (4-connectivity) of an unknown cell.

    Cartographer never places free cells directly against unknown space:
    there is always a 2-cell rim of intermediate-probability cells
    between them, so ``unknown_dilation`` must be at least 2 on its maps
    (keep it below the thinnest wall's thickness in cells, or frontiers
    will leak through walls).
    """
    free = (grid >= 0) & (grid <= free_max)
    unknown = grid == UNKNOWN
    # Grow the unknown region through anything that is not a wall.
    #
    # The band of partially-observed cells between free space and
    # unknown is not a fixed width. Where the lidar swept closely it is
    # a couple of cells; where an area was glimpsed from a distance or
    # at a grazing angle it runs ten cells or more. A fixed-radius
    # dilation finds no frontier at all in those places, so a corridor
    # seen only from afar is never offered as a destination and the
    # vehicle exhausts the few crisp frontiers it can see while a third
    # of the maze sits unexplored.
    #
    # Masking the growth by walls keeps the reach generous without ever
    # crossing to the far side of one, which also makes the thin-wall
    # leak the line-of-sight filter was written for impossible.
    passable = grid < occupied_min
    reach = unknown.copy()
    for _ in range(unknown_dilation):
        reach = _dilate4(reach, 1) & (passable | unknown)
    return free & reach


def cluster_frontier_cells(
    mask: np.ndarray, min_size: int = 8, merge_gap: int = 1
) -> List[Frontier]:
    """Group frontier cells into clusters.

    Frontier bands are often a single cell wide and break into small
    fragments; fragments within ``merge_gap`` cells of each other are
    treated as one cluster (the flood fill runs on a dilated copy of the
    mask, but clusters keep only real frontier cells). Clusters smaller
    than ``min_size`` cells are discarded as noise.
    """
    bridged = _dilate4(mask, merge_gap) if merge_gap > 0 else mask
    visited = np.zeros_like(bridged, dtype=bool)
    rows, cols = mask.shape
    frontiers: List[Frontier] = []

    seeds = np.argwhere(mask)
    for r0, c0 in seeds:
        if visited[r0, c0]:
            continue
        # BFS flood fill over the 8-neighbourhood of the bridged mask.
        queue = deque([(r0, c0)])
        visited[r0, c0] = True
        cells = []
        while queue:
            r, c = queue.popleft()
            if mask[r, c]:
                cells.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < rows and 0 <= cc < cols:
                        if bridged[rr, cc] and not visited[rr, cc]:
                            visited[rr, cc] = True
                            queue.append((rr, cc))

        if len(cells) < min_size:
            continue

        cell_arr = np.array(cells)
        centroid = cell_arr.mean(axis=0)
        # Snap the goal to the frontier cell nearest the centroid so the
        # goal is always on known-free ground.
        dists = np.linalg.norm(cell_arr - centroid, axis=1)
        goal_cell = tuple(int(v) for v in cell_arr[np.argmin(dists)])
        frontiers.append(
            Frontier(
                cells=cell_arr,
                centroid=(float(centroid[0]), float(centroid[1])),
                goal_cell=goal_cell,
                size=len(cells),
            )
        )

    return frontiers


def _line_cells(r0, c0, r1, c1):
    """Integer cells on the segment (r0,c0)->(r1,c1), Bresenham."""
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 >= r0 else -1
    sc = 1 if c1 >= c0 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            return cells
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc


def frontier_sees_unknown(
    grid: np.ndarray,
    cell: Tuple[int, int],
    occupied_min: int = WALL_MIN,
    window: int = 8,
) -> bool:
    """True if unknown space is visible from ``cell`` without crossing a wall.

    A thin wall (fewer cells thick than the unknown dilation) can make
    free cells look like frontiers even though their unknown space lies
    on the other side of the wall and can never be mapped from here.
    This checks a straight line from the cell to each nearby unknown
    cell; the frontier counts only if some line stays wall-free.

    ``occupied_min`` must describe a *wall*, not merely "probably not
    free". Around a frontier the partially-observed cells run to a
    median of ~80, while solid maze walls sit at 99-100; a threshold of
    65 blocks every ray on fog alone and rejects all frontiers, ending
    exploration prematurely. Erring high is also the safer direction:
    admitting a false frontier costs one visit and is then blacklisted
    on arrival, whereas rejecting a real one can end the mission.
    """
    r0, c0 = cell
    rows, cols = grid.shape
    r_lo, r_hi = max(0, r0 - window), min(rows, r0 + window + 1)
    c_lo, c_hi = max(0, c0 - window), min(cols, c0 + window + 1)
    local = grid[r_lo:r_hi, c_lo:c_hi]
    unknown = np.argwhere(local == UNKNOWN)
    if unknown.size == 0:
        return False
    # Try nearest unknown cells first.
    order = np.argsort(
        (unknown[:, 0] + r_lo - r0) ** 2 + (unknown[:, 1] + c_lo - c0) ** 2
    )
    for idx in order[:24]:
        r1, c1 = unknown[idx][0] + r_lo, unknown[idx][1] + c_lo
        blocked = False
        for r, c in _line_cells(r0, c0, r1, c1)[1:-1]:
            if grid[r, c] >= occupied_min:
                blocked = True
                break
        if not blocked:
            return True
    return False


def find_frontiers(
    grid: np.ndarray,
    free_max: int = 25,
    min_size: int = 8,
    unknown_dilation: int = 1,
    occupied_min: int = WALL_MIN,
    require_line_of_sight: bool = False,
    min_goal_clearance: float = 0.0,
    max_goal_candidates: int = 40,
    face_unknown_radius: int = 0,
) -> List[Frontier]:
    """Detect and cluster frontiers on an occupancy grid."""
    mask = detect_frontier_cells(
        grid,
        free_max=free_max,
        unknown_dilation=unknown_dilation,
        occupied_min=occupied_min,
    )
    frontiers = cluster_frontier_cells(mask, min_size=min_size)
    if not frontiers:
        return frontiers

    # Goal placement and line of sight are resolved together, because
    # doing them in sequence is order-dependent: moving the goal cell
    # changes which cell the visibility test examines.
    clearance = None
    if min_goal_clearance > 0:
        clearance = obstacle_clearance(
            grid, occupied_min=occupied_min,
            max_cells=int(min_goal_clearance) + 5,
        )

    free_mask = None  # built lazily, only if a goal needs nudging
    facing_unknown = None
    if face_unknown_radius > 0:
        facing_unknown = unknown_gain(grid, face_unknown_radius)

    selected = []
    for f in frontiers:
        order = _candidate_order(
            f.cells, f.centroid, clearance, min_goal_clearance,
            facing_unknown=facing_unknown,
        )
        for idx in order[:max_goal_candidates]:
            cell = tuple(int(v) for v in f.cells[idx])
            if not require_line_of_sight or frontier_sees_unknown(
                grid, cell, occupied_min=occupied_min,
                window=unknown_dilation + 5,
            ):
                # Visibility is judged at the frontier cell; the goal
                # the vehicle is actually sent to may need to sit a
                # little further from the wall to be plannable.
                if clearance is not None and clearance[cell] < min_goal_clearance:
                    if free_mask is None:
                        free_mask = (grid >= 0) & (grid <= free_max)
                    cell = nudge_to_clearance(
                        clearance, free_mask, cell, min_goal_clearance,
                    )
                selected.append(replace(f, goal_cell=cell))
                break
    return selected


def nudge_to_clearance(clearance, free, cell, min_cells, max_steps=10):
    """Walk a goal cell away from walls until the planner can use it.

    Frontier cells border unknown space, and unknown space nearly
    always abuts a wall, so the chosen goal often sits inside the
    costmap's inscribed radius. NavFn will not place the robot there
    and returns no path at all: `compute_path_to_pose` aborts, the
    vehicle stops with nothing to follow (Nav2 reports distance
    remaining 0 while it is still halfway to the goal), and the
    behaviour tree grinds through spin/wait/backup recoveries for
    roughly half a minute before giving up. Run 70 lost 23 recovery
    cycles to this.

    Ranking already prefers roomy cells, but falls back to the least
    cramped cell available when a cluster has no good one -- which is
    still unplannable. Stepping the goal a few cells up the clearance
    gradient, staying on known-free ground, keeps the frontier usable
    instead of discarding it or dispatching a goal that cannot work.
    """
    rows, cols = clearance.shape
    r, c = int(cell[0]), int(cell[1])
    for _ in range(max_steps):
        if clearance[r, c] >= min_cells:
            break
        best = (clearance[r, c], r, c)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                if not free[nr, nc]:
                    continue
                if clearance[nr, nc] > best[0]:
                    best = (clearance[nr, nc], nr, nc)
        if (best[1], best[2]) == (r, c):
            break  # local maximum: nothing better adjacent
        r, c = best[1], best[2]
    return r, c


def _candidate_order(
    cells: np.ndarray,
    centroid,
    clearance,
    min_clearance: float,
    facing_unknown=None,
):
    """Rank a cluster's cells as goal candidates.

    Two things decide the order.

    Clearance first, because frontier cells border unknown space and
    unknown space usually abuts a wall: the cell nearest the centroid
    is often inside the robot's inscribed radius, where the planner
    cannot place the vehicle at all. The path then stops short and the
    controller grinds toward a pose it can never occupy.

    Among cells with real room, prefer the one facing the most unknown
    rather than the one nearest the cluster's middle. The goal has to
    stay on a known-free cell — moving it into the unknown outright
    plants it inside unmapped wall, which the planner will happily
    route through — but choosing the free cell with the most unexplored
    space around it puts the end of the route against the largest patch
    of unmapped ground instead of the middle of the boundary.
    """
    dists = np.linalg.norm(cells - np.asarray(centroid), axis=1)
    if facing_unknown is not None:
        facing = facing_unknown[cells[:, 0], cells[:, 1]]
        # Negative so that argsort puts the most unknown-facing first.
        primary = -facing.astype(float)
    else:
        primary = dists

    if clearance is None or min_clearance <= 0:
        return np.argsort(primary)
    room = clearance[cells[:, 0], cells[:, 1]]
    roomy = np.flatnonzero(room >= min_clearance)
    tight = np.flatnonzero(room < min_clearance)
    return np.concatenate(
        [roomy[np.argsort(primary[roomy])], tight[np.argsort(-room[tight])]]
    )


# Travel cost at or above which a frontier is treated as having no
# coarse route. The explorer substitutes a sentinel rather than
# dropping such a frontier, so this must stay below that sentinel and
# far above any real distance across the maze (its diagonal is ~28 m).
UNROUTABLE_COST = 1e5


def _routable(cost) -> bool:
    return math.isfinite(cost) and cost < UNROUTABLE_COST


# Metres added to a frontier's travel cost when scoring cleanup work,
# so that two nearby frontiers are separated mostly by how much they
# reveal rather than by a few metres of approach.
CLEANUP_DISTANCE_BIAS = 5.0


def _cleanup_value(gain: int, cost: float) -> float:
    """Unknown cells revealed per metre flown to reach them.

    Ranking cleanup on raw size makes distance a tiebreaker only, so
    the vehicle chases the single biggest frontier anywhere on the map
    and then the next biggest, which is usually on the opposite side.
    Run 68 dispatched goals at x = 9.67, then -10.04, then 9.71 --
    three 20 m crossings of an already-mapped maze in a row.

    Dividing by distance restores the trade-off without reopening the
    starvation this ordering was introduced to fix: an unclearable
    271-cell frontier 2 m away scores 39 against the 1045-cell west
    corridor's 61 at 12 m, so the corridor still wins, while a
    500-cell frontier 20 m off loses to a 300-cell one 3 m away.
    """
    if not _routable(cost):
        return 0.0
    return gain / (max(cost, 0.0) + CLEANUP_DISTANCE_BIAS)


def _gain(t):
    """Unknown cells a candidate would reveal.

    Falls back to cluster size when the caller supplies no gain, which
    keeps the ordering meaningful for callers that have not computed
    one -- but size is a poor stand-in, see below.
    """
    return t[6] if len(t) > 6 and t[6] is not None else t[5].size


def rank_candidates(tagged, min_gain=0):
    """Order frontier candidates for dispatch.

    ``tagged`` holds ``(been_there, age, cost, x, y, frontier[, gain])``
    per candidate, where ``been_there`` is 1 if the goal sits in ground
    the vehicle has already flown, ``age`` is the discovery sequence
    number, ``cost`` is travel distance in metres, and ``gain`` is the
    count of unknown cells near the goal.

    **Rank by what a frontier reveals, not by how many cells it has.**
    The two come apart badly. Measured across two captured maps, the
    frontiers worth flying to revealed 2191-6313 unknown cells while
    fog banded along the maze's outer wall revealed 195-836 -- a clean
    separation. Cluster size does not track that at all: a 1871-cell
    frontier revealed 11.4 m2 while a 10-cell one revealed 9.8 m2, and
    a 452-cell one revealed 1.3 m2 of nothing. Ranking on size sends
    the vehicle to long thin fog bands smeared along walls it has
    already flown past, which is exactly what "it keeps going back to
    places it has been" looks like from the outside.

    ``min_gain`` demotes -- never drops -- candidates revealing less
    than that many unknown cells, so they are still collected once
    everything worthwhile is gone.

    While any unvisited frontier remains, order is: somewhere new
    first, then newest-discovered (depth-first), then nearest.

    Once every candidate is in already-flown ground the vehicle is
    backtracking to collect leftovers, and depth-first order stops
    meaning anything -- it also becomes exploitable. A frontier whose
    unknown lies behind an unconfirmed wall can never be cleared, so it
    is rebuilt every cycle with a centroid that shifts by more than the
    match radius, re-registers as newly discovered, and wins
    newest-first forever. Measured on run 65: an unclearable line at
    x = -6.9 held the lead for 30 consecutive evaluations while the
    largest frontier on the map -- a 1045-cell opening into the
    unexplored west corridor -- was never once selected. Ranking
    cleanup by how much each frontier reveals cannot be starved that
    way, because a frontier that stays unclearable does not grow.

    Reachability still leads that key. The caller marks a frontier with
    no coarse route by giving it an enormous ``cost`` rather than
    dropping it -- a coarse estimate must not veto, since ranking it
    wrong costs a detour while vetoing it wrong can end the run early.
    Sorting on size alone silently discarded that signal: a large
    unreachable frontier outranked a small reachable one, and run 67
    spent 4 of its first 21 goals dispatching to frontiers the planner
    could not route to, each burning a wait-and-backup recovery cycle
    before being blacklisted. Ordering unroutable candidates last keeps
    both properties.
    """
    if not tagged:
        return []
    backtracking = all(t[0] for t in tagged)
    if backtracking:
        return sorted(
            tagged,
            key=lambda t: (not _routable(t[2]), -_cleanup_value(_gain(t), t[2])),
        )
    # Depth-first, but never into a frontier that reveals nothing.
    return sorted(
        tagged,
        key=lambda t: (t[0], _gain(t) < min_gain, -t[1], t[2]),
    )


def obstacle_clearance(
    grid: np.ndarray, occupied_min: int = WALL_MIN, max_cells: int = 20
) -> np.ndarray:
    """Distance in cells from each cell to the nearest wall.

    Bounded iterative dilation: cheap, and only needs to be accurate
    near obstacles. ``occupied_min`` must mean *wall* — using a low
    threshold measures distance to the fog around frontiers instead,
    which is what made an earlier attempt at clearance-aware goals
    behave incoherently.
    """
    reached = grid >= occupied_min
    dist = np.full(grid.shape, float(max_cells))
    dist[reached] = 0.0
    for d in range(1, max_cells):
        grown = reached.copy()
        grown[1:, :] |= reached[:-1, :]
        grown[:-1, :] |= reached[1:, :]
        grown[:, 1:] |= reached[:, :-1]
        grown[:, :-1] |= reached[:, 1:]
        dist[grown & ~reached] = float(d)
        reached = grown
    return dist


def unknown_gain(
    grid: np.ndarray, radius_cells: int
) -> np.ndarray:
    """Count of unknown cells within a square window of each cell.

    A proxy for how much new territory standing at a cell would reveal.
    Computed with an integral image so the whole map costs one pass
    regardless of the window size.
    """
    unknown = (grid == UNKNOWN).astype(np.int32)
    integral = unknown.cumsum(axis=0).cumsum(axis=1)
    integral = np.pad(integral, ((1, 0), (1, 0)))

    rows, cols = grid.shape
    r = int(radius_cells)
    rr = np.arange(rows)
    cc = np.arange(cols)
    r0 = np.clip(rr - r, 0, rows)[:, None]
    r1 = np.clip(rr + r + 1, 0, rows)[:, None]
    c0 = np.clip(cc - r, 0, cols)[None, :]
    c1 = np.clip(cc + r + 1, 0, cols)[None, :]

    return (
        integral[r1, c1] - integral[r0, c1] - integral[r1, c0] + integral[r0, c0]
    )


def travel_distances(
    grid: np.ndarray,
    start: Tuple[int, int],
    free_max: int = 25,
    downsample: int = 2,
    max_steps: int = 800,
    occupied_min: int = WALL_MIN,
) -> np.ndarray:
    """Distance in cells from ``start`` to every free cell, around walls.

    Straight-line distance is a poor ranking for a maze: a frontier a
    few metres away through a wall can need a long detour, while one
    further off down an open corridor is genuinely closer. This is a
    breadth-first expansion over traversable cells, so the resulting
    ranking follows the route the vehicle must actually fly.

    The grid is coarsened by ``downsample`` first; a block is blocked
    if it contains any occupied cell, which stops the expansion
    squeezing through walls while still admitting unknown space. It
    must admit unknown space: frontier cells border unknown by
    definition, so requiring a wholly-free block would report every
    frontier as unreachable. This also matches the planner, which runs
    with ``allow_unknown: true`` for the same reason.

    Ranking does not need fine resolution, and coarsening keeps this
    cheap (~6 ms on a full maze map). Returns distances in
    original-grid cells; unreachable cells are ``inf``.
    """
    rows, cols = grid.shape
    d = max(1, int(downsample))
    r_blocks, c_blocks = rows // d, cols // d
    if r_blocks == 0 or c_blocks == 0:
        d, r_blocks, c_blocks = 1, rows, cols

    blocked = grid >= occupied_min
    coarse = blocked[: r_blocks * d, : c_blocks * d]
    coarse = ~coarse.reshape(r_blocks, d, c_blocks, d).any(axis=(1, 3))

    sr, sc = min(start[0] // d, r_blocks - 1), min(start[1] // d, c_blocks - 1)
    dist = np.full((r_blocks, c_blocks), np.inf)
    frontier = np.zeros((r_blocks, c_blocks), dtype=bool)
    frontier[sr, sc] = True
    if not coarse[sr, sc]:
        # The vehicle may sit on a partially-occupied block; seed anyway.
        coarse = coarse.copy()
        coarse[sr, sc] = True
    dist[frontier] = 0.0

    reached = frontier.copy()
    for step in range(1, max_steps):
        grown = reached.copy()
        grown[1:, :] |= reached[:-1, :]
        grown[:-1, :] |= reached[1:, :]
        grown[:, 1:] |= reached[:, :-1]
        grown[:, :-1] |= reached[:, 1:]
        grown &= coarse
        new = grown & ~reached
        if not new.any():
            break
        dist[new] = float(step)
        reached = grown

    # Back to original-grid cells: one coarse step is d cells.
    return np.kron(dist, np.ones((d, d))) [:rows, :cols] * d


def cell_to_world(
    cell: Tuple[float, float],
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> Tuple[float, float]:
    """Convert a (row, col) grid index to world (x, y) at the cell centre."""
    row, col = cell
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution
    return x, y
