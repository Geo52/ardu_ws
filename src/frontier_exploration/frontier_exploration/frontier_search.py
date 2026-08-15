"""Frontier detection and clustering over an occupancy grid.

Pure NumPy — no ROS dependencies — so it can be unit tested in isolation.

Grid convention (nav_msgs/OccupancyGrid): -1 unknown, 0..100 occupancy
probability. A frontier cell is a free cell with at least one unknown
4-neighbour. Frontier cells are clustered by 8-connectivity; each cluster
above a minimum size becomes a candidate frontier whose goal is the
frontier cell nearest the cluster centroid (the centroid itself may fall
in unknown or occupied space for concave clusters).
"""

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
    grid: np.ndarray, free_max: int = 25, unknown_dilation: int = 1
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
    return free & _dilate4(unknown, unknown_dilation)


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
) -> List[Frontier]:
    """Detect and cluster frontiers on an occupancy grid."""
    mask = detect_frontier_cells(
        grid, free_max=free_max, unknown_dilation=unknown_dilation
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

    selected = []
    for f in frontiers:
        order = _candidate_order(f.cells, f.centroid, clearance, min_goal_clearance)
        for idx in order[:max_goal_candidates]:
            cell = tuple(int(v) for v in f.cells[idx])
            if not require_line_of_sight or frontier_sees_unknown(
                grid, cell, occupied_min=occupied_min,
                window=unknown_dilation + 5,
            ):
                selected.append(replace(f, goal_cell=cell))
                break
    return selected


def _candidate_order(cells: np.ndarray, centroid, clearance, min_clearance: float):
    """Rank a cluster's cells as goal candidates.

    Frontier cells border unknown space, and unknown space usually
    abuts a wall, so the cell nearest the centroid is often within the
    robot's inscribed radius of one. The planner then cannot place the
    vehicle at the goal: the path stops short and the controller grinds
    toward a pose it can never occupy, leaving the copter hovering at a
    wall. Prefer cells with real room around them, nearest-to-centroid
    among those; fall back to the roomiest available in tight spots.
    """
    dists = np.linalg.norm(cells - np.asarray(centroid), axis=1)
    if clearance is None or min_clearance <= 0:
        return np.argsort(dists)
    room = clearance[cells[:, 0], cells[:, 1]]
    roomy = np.flatnonzero(room >= min_clearance)
    tight = np.flatnonzero(room < min_clearance)
    return np.concatenate(
        [roomy[np.argsort(dists[roomy])], tight[np.argsort(-room[tight])]]
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
