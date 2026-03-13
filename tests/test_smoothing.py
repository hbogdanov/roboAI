from roboai.core.occupancy_grid import FREE, OCCUPIED, OccupancyGrid
from roboai.core.planners.smoothing import path_quality, shortcut_smooth_path


def test_shortcut_smoothing_reduces_detours_in_free_space():
    grid = OccupancyGrid(width=10, height=10, resolution=1.0)
    grid.grid[:, :] = FREE
    raw_path = [(1.5, 1.5), (2.5, 1.5), (3.5, 2.5), (4.5, 3.5), (5.5, 4.5)]
    smoothed = shortcut_smooth_path(grid, raw_path, robot_radius=0.0, allow_unknown=True)
    assert smoothed == [(1.5, 1.5), (5.5, 4.5)]


def test_shortcut_smoothing_respects_blocked_cells():
    grid = OccupancyGrid(width=10, height=10, resolution=1.0)
    grid.grid[:, :] = FREE
    grid.set_cell(3, 3, OCCUPIED)
    raw_path = [(1.5, 1.5), (2.5, 2.5), (3.5, 4.5), (4.5, 5.5)]
    smoothed = shortcut_smooth_path(grid, raw_path, robot_radius=0.0, allow_unknown=True)
    assert len(smoothed) > 2


def test_path_quality_counts_turns():
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (2.0, 1.0)]
    path_length, turns = path_quality(path)
    assert path_length > 0.0
    assert turns == 2
