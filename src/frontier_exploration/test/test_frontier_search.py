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


def test_thin_wall_blocks_frontiers_either_way():
    from frontier_exploration.frontier_search import find_frontiers

    # free | 2-cell wall | unknown. Dilation is masked by walls, so the
    # unknown cannot reach across even a thin one and no frontier is
    # produced. This used to leak and depend on the line-of-sight
    # filter to clean up afterwards; both paths must now stay clean.
    grid = make_grid(12, 14, fill=-1)
    grid[:, :5] = 0
    grid[:, 5:7] = 100
    assert find_frontiers(grid, min_size=3, unknown_dilation=8) == []
    assert find_frontiers(
        grid, min_size=3, unknown_dilation=8, require_line_of_sight=True
    ) == []


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


def test_line_of_sight_not_blocked_by_partially_observed_cells():
    from frontier_exploration.frontier_search import find_frontiers

    # Regression: partially-observed cells around a frontier reach ~80
    # on real Cartographer maps. Treating those as walls blocks every
    # ray and rejects every frontier, which ended a run after 34 s.
    grid = make_grid(12, 14, fill=-1)
    grid[:, :5] = 0
    grid[:, 5:7] = 80  # fog, not a wall
    fs = find_frontiers(
        grid, min_size=3, unknown_dilation=3, require_line_of_sight=True
    )
    assert len(fs) == 1


def test_travel_distance_goes_around_walls():
    from frontier_exploration.frontier_search import travel_distances

    # A wall splits the grid; the cell just beyond it is near in a
    # straight line but far to travel, via a gap at the bottom.
    grid = make_grid(40, 40, fill=0)
    grid[:36, 20] = 100  # wall with a gap at rows 36-39
    dist = travel_distances(grid, (4, 16), downsample=1)

    near_euclid = dist[4, 24]   # 8 cells away in a straight line
    same_side = dist[30, 16]    # 26 cells away, no wall between
    assert np.isfinite(near_euclid)
    assert near_euclid > same_side, "must route around the wall, not through"


def test_travel_distance_marks_unreachable():
    from frontier_exploration.frontier_search import travel_distances

    grid = make_grid(30, 30, fill=0)
    grid[:, 15] = 100  # solid wall, no gap
    dist = travel_distances(grid, (5, 5), downsample=1)
    assert np.isfinite(dist[5, 5])
    assert not np.isfinite(dist[5, 25]), "other side must be unreachable"


def test_unknown_gain_counts_nearby_unknown():
    from frontier_exploration.frontier_search import unknown_gain

    grid = make_grid(40, 40, fill=0)
    grid[:, 30:] = -1  # a block of unknown on the right
    gain = unknown_gain(grid, radius_cells=5)

    # Deep in known space, nothing unknown is in range.
    assert gain[20, 5] == 0
    # Right at the boundary, part of the window is unknown.
    assert gain[20, 28] > 0
    # Inside the unknown region, the window is mostly unknown.
    assert gain[20, 35] > gain[20, 28]


def test_unknown_gain_matches_bruteforce():
    from frontier_exploration.frontier_search import unknown_gain

    rng = np.random.default_rng(0)
    grid = rng.choice([-1, 0, 100], size=(25, 25)).astype(np.int8)
    r = 3
    gain = unknown_gain(grid, radius_cells=r)
    for (row, col) in [(0, 0), (12, 12), (24, 24), (5, 20)]:
        window = grid[max(0, row - r):row + r + 1, max(0, col - r):col + r + 1]
        assert gain[row, col] == (window == -1).sum()


def test_cluster_direct_call():
    mask = np.zeros((10, 10), dtype=bool)
    mask[2, 2:8] = True  # a 6-cell line
    frontiers = cluster_frontier_cells(mask, min_size=3)
    assert len(frontiers) == 1
    assert frontiers[0].size == 6
    # Centroid of the line is its middle.
    assert abs(frontiers[0].centroid[0] - 2.0) < 1e-9
    assert abs(frontiers[0].centroid[1] - 4.5) < 1e-9


def _cand(been_there, age, cost, size, x=0.0, y=0.0, gain=None):
    from frontier_exploration.frontier_search import Frontier

    f = Frontier(
        cells=np.zeros((size, 2), dtype=int),
        centroid=(0.0, 0.0),
        goal_cell=(0, 0),
        size=size,
    )
    # gain defaults to size so the older size-based cases keep their
    # meaning; real callers pass a measured unknown-cell count.
    return (been_there, age, cost, x, y, f, size if gain is None else gain)


def test_ranking_prefers_new_ground_then_newest():
    from frontier_exploration.frontier_search import rank_candidates

    old_new_ground = _cand(0, 1, 5.0, 10)
    newest_visited = _cand(1, 99, 1.0, 10)
    ranked = rank_candidates([newest_visited, old_new_ground])
    assert ranked[0] is old_new_ground, "unvisited ground outranks visited"

    older = _cand(0, 1, 1.0, 10)
    newer = _cand(0, 2, 9.0, 10)
    assert rank_candidates([older, newer])[0] is newer, "depth-first: newest"


def test_backtracking_cannot_be_starved_by_an_unclearable_frontier():
    from frontier_exploration.frontier_search import rank_candidates

    # Run 65: a frontier whose unknown sits behind an unconfirmed wall
    # never clears, so its centroid shifts past the match radius every
    # cycle and it re-registers as newly discovered. Under newest-first
    # it held the lead for 30 consecutive evaluations while the biggest
    # opening on the map went unvisited.
    unclearable = _cand(1, 500, 2.0, 271)
    west_corridor = _cand(1, 3, 12.0, 1045)
    ranked = rank_candidates([unclearable, west_corridor])
    assert ranked[0] is west_corridor, "cleanup must follow information"

    # Repeated re-registration must not change that.
    for seq in range(501, 540):
        ranked = rank_candidates([_cand(1, seq, 2.0, 271), west_corridor])
        assert ranked[0] is west_corridor


def test_backtracking_breaks_size_ties_by_distance():
    from frontier_exploration.frontier_search import rank_candidates

    near = _cand(1, 1, 3.0, 100)
    far = _cand(1, 9, 30.0, 100)
    assert rank_candidates([far, near])[0] is near


def test_ranking_handles_empty():
    from frontier_exploration.frontier_search import rank_candidates

    assert rank_candidates([]) == []


def test_backtracking_deprioritises_unroutable_frontiers():
    from frontier_exploration.frontier_search import rank_candidates

    # Run 67: the explorer flags a frontier with no coarse route by
    # substituting a huge cost instead of dropping it. Ranking cleanup
    # on size alone ignored that, so a big unroutable frontier beat a
    # small reachable one and 4 of the first 21 goals were dispatched
    # to places the planner could not reach.
    big_unroutable = _cand(1, 5, 1e6, 900)
    small_reachable = _cand(1, 5, 8.0, 40)
    ranked = rank_candidates([big_unroutable, small_reachable])
    assert ranked[0] is small_reachable
    assert ranked[-1] is big_unroutable, "deprioritised, never dropped"


def test_backtracking_still_prefers_size_among_routable():
    from frontier_exploration.frontier_search import rank_candidates

    # The anti-starvation property must survive the reachability key.
    unclearable = _cand(1, 500, 2.0, 271)
    west_corridor = _cand(1, 3, 12.0, 1045)
    assert rank_candidates([unclearable, west_corridor])[0] is west_corridor


def test_unroutable_infinite_cost_sorts_last():
    from frontier_exploration.frontier_search import rank_candidates

    inf_cost = _cand(1, 1, float("inf"), 5000)
    modest = _cand(1, 1, 4.0, 12)
    assert rank_candidates([inf_cost, modest])[0] is modest


def test_backtracking_does_not_cross_the_map_for_marginally_more_area():
    from frontier_exploration.frontier_search import rank_candidates

    # Run 68 dispatched goals at x=9.67, then -10.04, then 9.71: three
    # 20 m crossings of already-mapped maze in a row, because raw size
    # made distance a tiebreaker only.
    far_big = _cand(1, 1, 20.0, 500)
    near_smaller = _cand(1, 2, 3.0, 300)
    assert rank_candidates([far_big, near_smaller])[0] is near_smaller


def test_distance_weighting_does_not_reopen_starvation():
    from frontier_exploration.frontier_search import rank_candidates

    # The run-65 case must still resolve in the corridor's favour: a
    # nearby unclearable frontier must not beat a far larger opening.
    unclearable_near = _cand(1, 500, 2.0, 271)
    west_corridor = _cand(1, 3, 12.0, 1045)
    assert rank_candidates([unclearable_near, west_corridor])[0] is west_corridor


def test_cleanup_prefers_bigger_at_equal_distance():
    from frontier_exploration.frontier_search import rank_candidates

    small = _cand(1, 1, 6.0, 50)
    big = _cand(1, 1, 6.0, 800)
    assert rank_candidates([small, big])[0] is big


def test_unroutable_still_last_under_distance_weighting():
    from frontier_exploration.frontier_search import rank_candidates

    unroutable = _cand(1, 1, 1e6, 5000)
    tiny = _cand(1, 1, 15.0, 9)
    assert rank_candidates([unroutable, tiny])[0] is tiny


def test_nudge_moves_goal_out_of_the_inscribed_zone():
    from frontier_exploration.frontier_search import (
        nudge_to_clearance, obstacle_clearance,
    )

    # Corridor of free space with a wall along column 0. A goal placed
    # hard against the wall is unplannable: NavFn will not put the
    # robot inside its inscribed radius, so compute_path_to_pose
    # aborts and Nav2 burns recovery cycles going nowhere.
    grid = make_grid(30, 30, fill=0)
    grid[:, 0] = 100
    clear = obstacle_clearance(grid)
    free = (grid >= 0) & (grid <= 25)

    start = (15, 1)
    assert clear[start] < 7
    r, c = nudge_to_clearance(clear, free, start, min_cells=7)
    assert clear[r, c] >= 7, "goal must end up outside the inscribed zone"
    assert free[r, c], "and must stay on known-free ground"


def test_nudge_leaves_a_roomy_goal_alone():
    from frontier_exploration.frontier_search import (
        nudge_to_clearance, obstacle_clearance,
    )

    grid = make_grid(40, 40, fill=0)
    grid[:, 0] = 100
    clear = obstacle_clearance(grid)
    free = (grid >= 0) & (grid <= 25)
    start = (20, 25)
    assert nudge_to_clearance(clear, free, start, min_cells=5) == start


def test_nudge_terminates_when_no_room_exists():
    from frontier_exploration.frontier_search import (
        nudge_to_clearance, obstacle_clearance,
    )

    # A 3-cell-wide pocket: no cell can reach the requested clearance,
    # so the walk must stop rather than loop or wander off the grid.
    grid = make_grid(20, 20, fill=100)
    grid[9:12, 5:15] = 0
    clear = obstacle_clearance(grid)
    free = (grid >= 0) & (grid <= 25)
    r, c = nudge_to_clearance(clear, free, (10, 6), min_cells=20)
    assert free[r, c]
    assert 9 <= r <= 11


def test_find_frontiers_emits_plannable_goals():
    from frontier_exploration.frontier_search import (
        find_frontiers, obstacle_clearance,
    )

    # Free corridor beside a wall, unknown beyond: the natural goal
    # hugs the wall. With a clearance requirement it must not.
    grid = make_grid(40, 40, fill=-1)
    grid[:, 0] = 100
    grid[:, 1:14] = 0
    clear = obstacle_clearance(grid)
    fs = find_frontiers(grid, min_size=5, unknown_dilation=3,
                        min_goal_clearance=6)
    assert fs
    for f in fs:
        r, c = f.goal_cell
        assert 0 <= grid[r, c] <= 25, "goal must be on known-free ground"
        assert c > 1, f"goal at column {c} is against the wall"
        assert clear[r, c] >= 6, (
            f"goal clearance {clear[r, c]} below the requested 6"
        )


def test_ranking_ignores_big_clusters_that_reveal_nothing():
    from frontier_exploration.frontier_search import rank_candidates

    # Measured on two captured maps: fog banded along the maze's outer
    # wall forms large clusters revealing 0.5-2.1 m2, while the
    # frontiers worth flying to reveal 5.5-15.8 m2. A 452-cell wall
    # band revealed 1.3 m2; a 10-cell interior frontier revealed
    # 9.8 m2. Size must not decide this.
    wall_fog = _cand(1, 99, 3.0, 452, gain=520)
    real_opening = _cand(1, 5, 9.0, 10, gain=3939)
    assert rank_candidates([wall_fog, real_opening])[0] is real_opening


def test_depth_first_demotes_zero_value_frontiers():
    from frontier_exploration.frontier_search import rank_candidates

    # Forward exploration, both in new ground. The newest frontier wins
    # normally -- but not when it reveals nothing, which is what made
    # the vehicle ping-pong along the north wall in run 72.
    newest_but_empty = _cand(0, 99, 2.0, 300, gain=520)
    older_but_rich = _cand(0, 3, 8.0, 60, gain=6000)
    ranked = rank_candidates([newest_but_empty, older_but_rich], min_gain=1200)
    assert ranked[0] is older_but_rich
    # ...and with no threshold, depth-first ordering is unchanged.
    ranked = rank_candidates([newest_but_empty, older_but_rich], min_gain=0)
    assert ranked[0] is newest_but_empty


def test_low_gain_frontiers_are_demoted_not_dropped():
    from frontier_exploration.frontier_search import rank_candidates

    poor = _cand(0, 1, 1.0, 50, gain=100)
    ranked = rank_candidates([poor], min_gain=1200)
    assert len(ranked) == 1 and ranked[0] is poor


def test_gain_falls_back_to_size_when_absent():
    from frontier_exploration.frontier_search import rank_candidates, Frontier
    import numpy as _np

    def bare(been_there, age, cost, size):
        f = Frontier(cells=_np.zeros((size, 2), dtype=int),
                     centroid=(0.0, 0.0), goal_cell=(0, 0), size=size)
        return (been_there, age, cost, 0.0, 0.0, f)   # no gain element

    small, big = bare(1, 1, 5.0, 10), bare(1, 1, 5.0, 900)
    assert rank_candidates([small, big])[0] is big
