import math
import numpy as np

class OccupancyGrid:
    def __init__(
        self,
        width_m=20.0,
        height_m=20.0,
        resolution=0.05,
        lo_occ=+0.85,
        lo_free=-0.40,
        lo_min=-4.0,
        lo_max=+4.0,
        origin_center=True,
    ):
        """
        width_m, height_m: map dimensions in meters
        resolution: cell size (m/cell)
        log-odds params: lo_occ (hit), lo_free (miss), clamped to [lo_min, lo_max]
        origin_center: if True, world (0,0) is grid center. If False, origin at lower-left.
        """
        self.res = float(resolution)
        self.w = int(round(width_m / resolution))
        self.h = int(round(height_m / resolution))
        if origin_center:
            self.origin_m = (-width_m / 2.0, -height_m / 2.0)
        else:
            self.origin_m = (0.0, 0.0)

        self.grid = np.zeros((self.h, self.w), dtype=np.float32)
        self.lo_occ, self.lo_free = float(lo_occ), float(lo_free)
        self.lo_min, self.lo_max = float(lo_min), float(lo_max)

    # Transformations
    def world_to_grid(self, x_m, y_m):
        gx = int((x_m - self.origin_m[0]) / self.res)
        gy = int((y_m - self.origin_m[1]) / self.res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.res + self.origin_m[0]
        y = gy * self.res + self.origin_m[1]
        return x, y

    # Main Update Function
    def update_from_scan(self, pose_xytheta, ranges, angle_min, angle_inc, range_max):
        """
        pose_xytheta: (x,y,theta) in meters/radians, world frame
        ranges: list of floats in meters
        angle_min, angle_inc, range_max: lidar model params
        """
        x, y, th = pose_xytheta

        for i, r in enumerate(ranges):
            a = th + (angle_min + i * angle_inc)
            r_c = min(float(r), float(range_max))

            steps = max(1, int(r_c / self.res))
            for s in range(steps):
                d = s * self.res
                cx = x + d * math.cos(a)
                cy = y + d * math.sin(a)
                gx, gy = self.world_to_grid(cx, cy)
                if 0 <= gx < self.w and 0 <= gy < self.h:
                    self.grid[gy, gx] += self.lo_free

            if r < range_max * 0.99:
                hx = x + r * math.cos(a)
                hy = y + r * math.sin(a)
                gx, gy = self.world_to_grid(hx, hy)
                if 0 <= gx < self.w and 0 <= gy < self.h:
                    self.grid[gy, gx] += self.lo_occ

        np.clip(self.grid, self.lo_min, self.lo_max, out=self.grid)

    # Queries and Visualization Helpers
    def is_free(self, gx, gy, thresh=0.0):
        return self.grid[gy, gx] < thresh

    def prob(self):
        """Return probabilities (0..1) view from log-odds grid, for quick viz."""
        return 1.0 - 1.0 / (1.0 + np.exp(self.grid))
