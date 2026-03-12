import numpy as np

from roboai.core.occupancy_grid import FREE, OCCUPIED, OccupancyGrid
from roboai.core.types import LaserScan2D, Pose2D


def test_world_grid_round_trip_is_centered():
    grid = OccupancyGrid(width=10, height=10, resolution=0.5)
    gx, gy = grid.world_to_grid(1.2, 1.7)
    x, y = grid.grid_to_world(gx, gy)
    assert abs(x - 1.25) < 1e-6
    assert abs(y - 1.75) < 1e-6


def test_scan_marks_free_and_hit_cells():
    grid = OccupancyGrid(width=20, height=20, resolution=0.5)
    pose = Pose2D(x=1.0, y=1.0, theta=0.0)
    scan = LaserScan2D(angles=np.array([0.0]), ranges=np.array([2.0]), max_range=5.0)
    grid.update_from_scan(pose, scan)
    assert grid.get_cell(2, 2) == FREE
    assert grid.get_cell(6, 2) == OCCUPIED


def test_occupied_cells_are_not_overwritten_by_later_free_updates():
    grid = OccupancyGrid(width=20, height=20, resolution=0.5)
    pose = Pose2D(x=1.0, y=1.0, theta=0.0)
    hit_scan = LaserScan2D(angles=np.array([0.0]), ranges=np.array([2.0]), max_range=5.0)
    miss_scan = LaserScan2D(angles=np.array([0.0]), ranges=np.array([5.0]), max_range=5.0)
    grid.update_from_scan(pose, hit_scan)
    grid.update_from_scan(pose, miss_scan)
    assert grid.get_cell(6, 2) == OCCUPIED
