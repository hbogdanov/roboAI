from roboai.core.occupancy_grid import FREE, OCCUPIED, OccupancyGrid
from roboai.core.planners.astar import plan_astar


def test_astar_finds_path_around_wall_gap():
    grid = OccupancyGrid(width=10, height=10, resolution=1.0)
    grid.grid[:, :] = FREE
    for gy in range(10):
        grid.set_cell(4, gy, OCCUPIED)
    grid.set_cell(4, 5, FREE)
    result = plan_astar(grid, start_xy=(1.5, 1.5), goal_xy=(8.5, 8.5), robot_radius=0.0, allow_unknown=True)
    assert result.success is True
    assert result.path[0] == (1.5, 1.5)
    assert result.path[-1] == (8.5, 8.5)
