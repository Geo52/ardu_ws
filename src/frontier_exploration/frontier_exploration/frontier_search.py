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
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

UNKNOWN = -1


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
    occupied_min: int = 65,
    window: int = 8,
) -> bool:
    """True if unknown space is visible from ``cell`` without crossing a wall.

    A thin wall (fewer cells thick than the unknown dilation) can make
    free cells look like frontiers even though their unknown space lies
    on the other side of the wall and can never be mapped from here.
    This checks a straight line from the cell to each nearby unknown
    cell; the frontier counts only if some line stays wall-free.
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
    occupied_min: int = 65,
    require_line_of_sight: bool = False,
) -> List[Frontier]:
    """Detect and cluster frontiers on an occupancy grid."""
    mask = detect_frontier_cells(
        grid, free_max=free_max, unknown_dilation=unknown_dilation
    )
    frontiers = cluster_frontier_cells(mask, min_size=min_size)
    if require_line_of_sight:
        frontiers = [
            f
            for f in frontiers
            if frontier_sees_unknown(
                grid,
                f.goal_cell,
                occupied_min=occupied_min,
                window=unknown_dilation + 5,
            )
        ]
    return frontiers


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
