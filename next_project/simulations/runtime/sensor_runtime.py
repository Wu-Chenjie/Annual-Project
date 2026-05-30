from __future__ import annotations

import numpy as np

from core.obstacles import ObstacleField


class SensorRuntime:
    def _build_discovered_obstacle_field(self) -> ObstacleField:
        """Convert sensor-discovered occupied grid cells to an AABB obstacle field.

        Uses 3D 6-connected component labelling to cluster occupied voxels;
        each cluster becomes one AABB.  The returned field represents *only*
        what the drone fleet has observed so far — no truth-map leak.
        """
        from collections import deque

        field = ObstacleField()
        data = np.asarray(self.grid.data)
        occupied = data >= 1
        if not occupied.any():
            return field

        nx, ny, nz = data.shape
        labels = np.zeros_like(data, dtype=np.int32)
        current_label = 0

        indices = np.argwhere(occupied)
        for idx in indices:
            i, j, k = int(idx[0]), int(idx[1]), int(idx[2])
            if labels[i, j, k] != 0:
                continue
            current_label += 1
            q: deque[tuple[int, int, int]] = deque([(i, j, k)])
            labels[i, j, k] = current_label
            while q:
                ci, cj, ck = q.popleft()
                for di, dj, dk in ((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)):
                    ni, nj, nk = ci + di, cj + dj, ck + dk
                    if 0 <= ni < nx and 0 <= nj < ny and 0 <= nk < nz:
                        if occupied[ni, nj, nk] and labels[ni, nj, nk] == 0:
                            labels[ni, nj, nk] = current_label
                            q.append((ni, nj, nk))

        res = float(self.grid.resolution)
        for lbl in range(1, current_label + 1):
            mask = labels == lbl
            coords = np.argwhere(mask)
            mn = coords.min(axis=0)
            mx = coords.max(axis=0)
            min_world = self.grid.index_to_world(mn)
            max_world = self.grid.index_to_world(mx) + np.array([res, res, res], dtype=float)
            field.add_aabb(min_world, max_world)

        return field


    def _update_discovered_obstacles(self) -> None:
        """Rebuild the discovered obstacle field from the current sensor-updated grid."""
        if not getattr(self, "_planner_initial_map_unknown", False):
            return
        self._discovered_obstacles = self._build_discovered_obstacle_field()
        # Refresh FIRI refiner with discovered obstacles for correct post-process
        if hasattr(self, "firi_refiner") and self.firi_refiner is not None:
            self.firi_refiner.obstacle_field = self._discovered_obstacles
        # Refresh trajectory optimizer
        if hasattr(self, "trajectory_optimizer") and self.trajectory_optimizer is not None:
            self.trajectory_optimizer.obstacle_field = self._discovered_obstacles
        # Propagate to replanner for danger-mode SDF checks
        if hasattr(self, "replanner") and self.replanner is not None:
            self.replanner.obstacle_field = self._discovered_obstacles
            # Update GNN lazy obstacles if danger mode is active
            if self.replanner.danger_planner is not None:
                self.replanner.danger_planner._lazy_obstacles = self._discovered_obstacles
                self.replanner.danger_planner._cached_vis_graph = None


    def _channel_width_from_sensor(self, sensor_reading: np.ndarray | None) -> tuple[float, float, float] | None:
        """将六向测距读数折算为局部三轴通道宽度。"""
        if sensor_reading is None or len(sensor_reading) < 6:
            return None
        reading = np.asarray(sensor_reading, dtype=float)
        return (
            float(reading[2] + reading[3]),
            float(reading[0] + reading[1]),
            float(reading[4] + reading[5]),
        )
