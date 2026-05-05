from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.obstacles import OccupancyGrid
from core.planning.esdf import CostAwareGrid, compute_esdf


def test_compute_esdf_matches_bruteforce_for_free_voxels():
    grid = OccupancyGrid(origin=np.zeros(3), resolution=0.5, shape=(7, 7, 3))
    grid.data[3, 3, 1] = 1
    esdf = compute_esdf(grid)
    occupied = np.argwhere(grid.data >= 1)
    for idx_arr in np.argwhere(grid.data == 0):
        idx = tuple(int(v) for v in idx_arr)
        expected = min(float(np.linalg.norm(idx_arr - occ)) * grid.resolution for occ in occupied)
        assert abs(float(esdf[idx]) - expected) < 1e-9
    assert esdf[3, 3, 1] == 0.0


def test_cost_aware_grid_cost_decreases_with_clearance():
    grid = OccupancyGrid(origin=np.zeros(3), resolution=1.0, shape=(9, 3, 1))
    grid.data[0, 1, 0] = 1
    wrapped = CostAwareGrid(grid, weight=2.0, scale=1.0, cap_distance=6.0)
    near = wrapped.extra_cost((1, 1, 0))
    far = wrapped.extra_cost((5, 1, 0))
    assert near > far > 0.0
