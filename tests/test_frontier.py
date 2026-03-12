from roboai.core.frontier import frontier_cells, frontier_regions, rank_frontier_targets, select_frontier_target
from roboai.core.occupancy_grid import FREE, OccupancyGrid


def test_frontier_detection_groups_adjacent_frontiers():
    grid = OccupancyGrid(width=6, height=6, resolution=1.0)
    for gx in range(1, 4):
        grid.set_cell(gx, 1, FREE)
        grid.set_cell(gx, 2, FREE)
    assert frontier_cells(grid)
    assert len(frontier_regions(grid)) == 1


def test_frontier_target_returns_nearest_centroid():
    grid = OccupancyGrid(width=8, height=8, resolution=1.0)
    for gx in range(1, 4):
        grid.set_cell(gx, 1, FREE)
    for gy in range(4, 7):
        grid.set_cell(6, gy, FREE)
    target = select_frontier_target(grid, robot_xy=(1.5, 1.5))
    assert target is not None
    assert target[0] < 4.0


def test_rank_frontier_targets_returns_multiple_candidates():
    grid = OccupancyGrid(width=10, height=10, resolution=1.0)
    for gx in range(1, 4):
        grid.set_cell(gx, 1, FREE)
    for gy in range(5, 8):
        grid.set_cell(7, gy, FREE)
    targets = rank_frontier_targets(grid, robot_xy=(1.5, 1.5))
    assert len(targets) >= 2
    assert targets[0][0] < 5.0
