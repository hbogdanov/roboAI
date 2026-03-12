from roboai.core.occupancy_grid import FREE, OccupancyGrid
from roboai.core.planners.rrt import plan_rrt
from roboai.core.planners.rrt_star import plan_rrt_star


def test_rrt_interface_returns_path_result():
    grid = OccupancyGrid(width=6, height=6, resolution=1.0)
    grid.grid[:, :] = FREE
    result = plan_rrt(grid, start_xy=(1.5, 1.5), goal_xy=(4.5, 4.5), robot_radius=0.0, allow_unknown=True)
    assert result.success is True
    assert len(result.path) >= 2


def test_rrt_star_interface_returns_path_result():
    grid = OccupancyGrid(width=6, height=6, resolution=1.0)
    grid.grid[:, :] = FREE
    result = plan_rrt_star(grid, start_xy=(1.5, 1.5), goal_xy=(4.5, 4.5), robot_radius=0.0, allow_unknown=True)
    assert result.success is True
    assert len(result.path) >= 2
