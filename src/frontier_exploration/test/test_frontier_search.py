import numpy as np

from frontier_exploration.frontier_search import (
    cell_to_world,
    cluster_frontier_cells,
    detect_frontier_cells,
    find_frontiers,
)


def make_grid(rows, cols, fill=-1):
    return np.full((rows, cols), fill, dtype=np.int8)


def test_no_frontiers_in_fully_known_grid():
    grid = make_grid(10, 10, fill=0)
    assert not detect_frontier_cells(grid).any()


def test_no_frontiers_in_fully_unknown_grid():
    grid = make_grid(10, 10, fill=-1)
    assert not detect_frontier_cells(grid).any()


def test_free_unknown_boundary_is_frontier():
    grid = make_grid(10, 10, fill=-1)
    grid[:, :5] = 0  # left half free, right half unknown
    mask = detect_frontier_cells(grid)
    # Only the free column adjacent to unknown is frontier.
    assert mask[:, 4].all()
    assert not mask[:, :4].any()
    assert not mask[:, 5:].any()


def test_occupied_cells_are_not_frontier():
    grid = make_grid(10, 10, fill=-1)
    grid[:, :5] = 100  # occupied next to unknown
    assert not detect_frontier_cells(grid).any()


def test_wall_blocks_frontier():
    # free | wall | unknown: the free cells touch the wall, not unknown.
    grid = make_grid(10, 12, fill=-1)
    grid[:, :5] = 0
    grid[:, 5] = 100
    mask = detect_frontier_cells(grid)
    assert not mask.any()


def test_clustering_min_size_filters_noise():
    grid = make_grid(20, 20, fill=0)
    grid[10, 10] = -1  # single unknown cell -> tiny frontier ring
    frontiers = find_frontiers(grid, min_size=10)
    assert frontiers == []
    frontiers = find_frontiers(grid, min_size=1)
    assert len(frontiers) == 1


def test_two_separate_clusters():
    grid = make_grid(20, 20, fill=-1)
    grid[:5, :5] = 0  # free patch top-left
    grid[15:, 15:] = 0  # free patch bottom-right
    frontiers = find_frontiers(grid, min_size=3)
    assert len(frontiers) == 2


def test_goal_cell_is_on_frontier():
    grid = make_grid(30, 30, fill=-1)
    grid[:15, :] = 0
    frontiers = find_frontiers(grid, min_size=3)
    assert len(frontiers) == 1
    f = frontiers[0]
    mask = detect_frontier_cells(grid)
    r, c = f.goal_cell
    assert mask[r, c]
    assert f.size == f.cells.shape[0]


def test_cell_to_world():
    x, y = cell_to_world((0, 0), origin_x=-10.0, origin_y=-5.0, resolution=0.1)
    assert abs(x - (-9.95)) < 1e-9
    assert abs(y - (-4.95)) < 1e-9
    x, y = cell_to_world((10, 20), origin_x=0.0, origin_y=0.0, resolution=0.05)
    assert abs(x - 1.025) < 1e-9
    assert abs(y - 0.525) < 1e-9


def test_dilation_bridges_cartographer_grey_rim():
    # Cartographer separates free space from unknown with a rim of
    # intermediate-probability cells; plain adjacency finds nothing.
    grid = make_grid(10, 12, fill=-1)
    grid[:, :5] = 0  # free
    grid[:, 5:7] = 45  # 2-cell uncertain rim, columns 5-6
    mask = detect_frontier_cells(grid, unknown_dilation=1)
    assert not mask.any()
    # Free cells at col 4 are 3 steps from the unknown at col 7.
    mask = detect_frontier_cells(grid, unknown_dilation=3)
    assert mask[:, 4].all()
    assert not mask[:, :4].any()


def test_dilation_does_not_leak_through_thick_walls():
    # free | 4-cell wall | unknown: dilation 3 must not create frontiers.
    grid = make_grid(10, 14, fill=-1)
    grid[:, :5] = 0
    grid[:, 5:9] = 100
    mask = detect_frontier_cells(grid, unknown_dilation=3)
    assert not mask.any()


def test_line_of_sight_rejects_frontiers_behind_thin_walls():
    from frontier_exploration.frontier_search import find_frontiers

    # free | 2-cell wall | unknown: dilation 3 leaks through the thin
    # wall, but the LOS filter must reject the resulting frontier.
    grid = make_grid(12, 14, fill=-1)
    grid[:, :5] = 0
    grid[:, 5:7] = 100
    leaked = find_frontiers(grid, min_size=3, unknown_dilation=3)
    assert leaked  # sanity: the artifact exists without the filter
    filtered = find_frontiers(
        grid, min_size=3, unknown_dilation=3, require_line_of_sight=True
    )
    assert filtered == []


def test_line_of_sight_keeps_open_frontiers():
    from frontier_exploration.frontier_search import find_frontiers

    # free | grey rim | unknown, no wall: the frontier must survive.
    grid = make_grid(12, 14, fill=-1)
    grid[:, :5] = 0
    grid[:, 5:7] = 45
    fs = find_frontiers(
        grid, min_size=3, unknown_dilation=3, require_line_of_sight=True
    )
    assert len(fs) == 1


def test_cluster_direct_call():
    mask = np.zeros((10, 10), dtype=bool)
    mask[2, 2:8] = True  # a 6-cell line
    frontiers = cluster_frontier_cells(mask, min_size=3)
    assert len(frontiers) == 1
    assert frontiers[0].size == 6
    # Centroid of the line is its middle.
    assert abs(frontiers[0].centroid[0] - 2.0) < 1e-9
    assert abs(frontiers[0].centroid[1] - 4.5) < 1e-9
