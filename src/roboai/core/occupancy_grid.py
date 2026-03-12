from __future__ import annotations

import math

import numpy as np


UNKNOWN = -1
FREE = 0
OCCUPIED = 1


class OccupancyGrid:
    def __init__(self, width: int, height: int, resolution: float, origin: tuple[float, float] = (0.0, 0.0)):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.grid = np.full((self.height, self.width), UNKNOWN, dtype=np.int8)

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        gx = int(math.floor((x - self.origin[0]) / self.resolution))
        gy = int(math.floor((y - self.origin[1]) / self.resolution))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        x = self.origin[0] + (gx + 0.5) * self.resolution
        y = self.origin[1] + (gy + 0.5) * self.resolution
        return x, y

    def set_cell(self, gx: int, gy: int, value: int) -> None:
        if self.in_bounds(gx, gy):
            self.grid[gy, gx] = np.int8(value)

    def mark_free(self, gx: int, gy: int) -> None:
        if not self.in_bounds(gx, gy):
            return
        if self.grid[gy, gx] == OCCUPIED:
            return
        self.grid[gy, gx] = np.int8(FREE)

    def mark_occupied(self, gx: int, gy: int) -> None:
        if self.in_bounds(gx, gy):
            self.grid[gy, gx] = np.int8(OCCUPIED)

    def get_cell(self, gx: int, gy: int) -> int:
        if not self.in_bounds(gx, gy):
            return OCCUPIED
        return int(self.grid[gy, gx])

    def known_ratio(self) -> float:
        return float(np.count_nonzero(self.grid != UNKNOWN) / self.grid.size)

    def update_from_scan(self, pose, scan) -> None:
        from roboai.core.types import Pose2D

        assert isinstance(pose, Pose2D)
        rx, ry = self.world_to_grid(pose.x, pose.y)
        self.mark_free(rx, ry)

        for angle, distance in zip(scan.angles, scan.ranges):
            hit = float(distance) < float(scan.max_range) - 1e-6
            ray_len = min(float(distance), float(scan.max_range))
            end_x = pose.x + ray_len * math.cos(pose.theta + float(angle))
            end_y = pose.y + ray_len * math.sin(pose.theta + float(angle))
            ex, ey = self.world_to_grid(end_x, end_y)
            cells = self._bresenham(rx, ry, ex, ey)
            if not cells:
                continue
            free_cells = cells[:-1] if hit else cells
            for cx, cy in free_cells:
                self.mark_free(cx, cy)
            if hit:
                self.mark_occupied(ex, ey)

    def inflated_obstacles(self, radius_cells: int) -> np.ndarray:
        radius = max(0, int(radius_cells))
        occ = self.grid == OCCUPIED
        if radius == 0:
            return occ.copy()
        inflated = occ.copy()
        ys, xs = np.nonzero(occ)
        for x, y in zip(xs, ys):
            x0 = max(0, x - radius)
            x1 = min(self.width, x + radius + 1)
            y0 = max(0, y - radius)
            y1 = min(self.height, y + radius + 1)
            inflated[y0:y1, x0:x1] = True
        return inflated

    @staticmethod
    def _bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
        cells: list[tuple[int, int]] = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return cells
