"""FIRI 风格的半空间凸走廊精修。

实现目标：
- 使用显式半空间约束 A x <= b 构造每段路径的局部凸走廊
- 保留 seed 点集作为走廊必须包含的对象
- 通过投影而不是球形采样，得到更严谨的路径修正

说明：
- 这是面向当前项目的工程化 FIRI 实现，不包含完整 MVIE 迭代。
- 相比前一版球形 corridor，更接近论文里的 manageability 思路。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class FIRICorridor:
    """单段路径的凸走廊 A x <= b。"""

    A: np.ndarray
    b: np.ndarray
    start: np.ndarray
    goal: np.ndarray
    min_clearance: float
    ellipsoid_center: np.ndarray | None = None
    ellipsoid_shape: np.ndarray | None = None

    def contains(self, point: np.ndarray, tol: float = 1e-8) -> bool:
        point = np.asarray(point, dtype=float)
        return bool(np.all(self.A @ point <= self.b + tol))

    def project(self, point: np.ndarray, max_iter: int = 8) -> np.ndarray:
        """交替投影到半空间集合。"""
        point = np.asarray(point, dtype=float).copy()
        for _ in range(max_iter):
            changed = False
            values = self.A @ point - self.b
            for i in range(len(values)):
                violation = float(values[i])
                if violation <= 0.0:
                    continue
                normal = self.A[i]
                denom = float(np.dot(normal, normal))
                if denom <= 1e-12:
                    continue
                point = point - (violation / denom) * normal
                changed = True
            if not changed:
                break
        return point


class FIRIRefiner:
    """FIRI 风格路径精修器。"""

    def __init__(
        self,
        obstacle_field,
        min_clearance: float,
        sample_step: float = 0.2,
        max_projection_iter: int = 10,
        mvie_enabled: bool = True,
    ):
        self.obstacle_field = obstacle_field
        self.min_clearance = float(min_clearance)
        self.sample_step = float(sample_step)
        self.max_projection_iter = int(max_projection_iter)
        self.mvie_enabled = bool(mvie_enabled)

    def refine(self, path: np.ndarray, seeds: np.ndarray | None = None) -> np.ndarray:
        path = np.asarray(path, dtype=float)
        if len(path) < 2:
            return path

        if seeds is None:
            seeds = path
        seeds = np.asarray(seeds, dtype=float)
        if len(seeds) < 2:
            seeds = path

        safe_seeds = self._prepare_seeds(seeds)
        corridors = self._build_corridors(safe_seeds)
        if not corridors:
            return self._remove_duplicate_points(self._push_path_out_if_needed(path))

        refined: list[np.ndarray] = [np.asarray(safe_seeds[0], dtype=float)]
        for idx, corridor in enumerate(corridors):
            segment = self._refine_segment(corridor)
            if len(segment) > 1:
                if idx == 0:
                    segment[0] = safe_seeds[idx]
                segment[-1] = safe_seeds[idx + 1]
                refined.extend(segment[1:])
        refined_arr = np.asarray(refined, dtype=float)
        refined_arr = self._push_path_out_if_needed(refined_arr)
        refined_arr[0] = safe_seeds[0]
        refined_arr[-1] = safe_seeds[-1]
        return self._remove_duplicate_points(refined_arr)

    def _build_corridors(self, seeds: np.ndarray) -> list[FIRICorridor]:
        corridors: list[FIRICorridor] = []
        for i in range(len(seeds) - 1):
            start = np.asarray(seeds[i], dtype=float)
            goal = np.asarray(seeds[i + 1], dtype=float)
            corridor = self._build_segment_corridor(start, goal)
            corridors.append(corridor)
        return corridors

    def _build_segment_corridor(self, start: np.ndarray, goal: np.ndarray) -> FIRICorridor:
        """为路径段构造半空间凸走廊。"""
        A_rows: list[np.ndarray] = []
        b_rows: list[float] = []

        # 先加一个围绕线段的轴向包围盒，避免开放半空间太松
        seed_points = self._sample_segment(start, goal, self.sample_step)
        seed_min = np.min(seed_points, axis=0) - self.min_clearance
        seed_max = np.max(seed_points, axis=0) + self.min_clearance
        basis_normals = np.eye(3, dtype=float)
        for axis in range(3):
            n_pos = basis_normals[axis]
            n_neg = -basis_normals[axis]
            A_rows.append(n_pos)
            b_rows.append(float(seed_max[axis]))
            A_rows.append(n_neg)
            b_rows.append(float(-seed_min[axis]))

        for obs in self.obstacle_field:
            plane = self._analytic_separating_plane(obs, start, goal, seed_points)
            if plane is None:
                continue
            normal, offset = plane
            A_rows.append(normal)
            b_rows.append(offset)

        A = np.asarray(A_rows, dtype=float)
        b = np.asarray(b_rows, dtype=float)
        corridor = FIRICorridor(A=A, b=b, start=start, goal=goal, min_clearance=self.min_clearance)
        if self.mvie_enabled:
            center, shape = self._compute_mvie_or_fallback(corridor)
            corridor.ellipsoid_center = center
            corridor.ellipsoid_shape = shape
        return corridor

    def _prepare_seeds(self, seeds: np.ndarray) -> np.ndarray:
        """保留端点，修正不安全的中间种子点。"""
        seeds = np.asarray(seeds, dtype=float).copy()
        if len(seeds) <= 2:
            return self._push_path_out_if_needed(seeds)
        for i in range(1, len(seeds) - 1):
            if float(self.obstacle_field.signed_distance(seeds[i])) < self.min_clearance:
                seeds[i] = self._push_out_if_needed(seeds[i])
        return seeds

    def _refine_segment(self, corridor: FIRICorridor) -> np.ndarray:
        points = self._sample_segment(corridor.start, corridor.goal, self.sample_step)
        refined: list[np.ndarray] = [corridor.start.copy()]
        for point in points[1:-1]:
            candidate = corridor.project(point, max_iter=self.max_projection_iter)
            candidate = self._push_out_if_needed(candidate)
            candidate = corridor.project(candidate, max_iter=self.max_projection_iter)
            refined.append(candidate)
        refined.append(corridor.goal.copy())
        return np.asarray(refined, dtype=float)

    def _compute_mvie_or_fallback(self, corridor: FIRICorridor) -> tuple[np.ndarray, np.ndarray]:
        try:
            return self._compute_mvie_cvxpy(corridor)
        except Exception:
            return self._compute_inscribed_ball(corridor)

    def _compute_mvie_cvxpy(self, corridor: FIRICorridor) -> tuple[np.ndarray, np.ndarray]:
        import cvxpy as cp

        A = np.asarray(corridor.A, dtype=float)
        b = np.asarray(corridor.b, dtype=float)
        dim = 3
        B = cp.Variable((dim, dim), symmetric=True)
        center = cp.Variable(dim)

        constraints = [B >> 1e-6 * np.eye(dim)]
        for i in range(A.shape[0]):
            ai = A[i]
            constraints.append(cp.norm(B @ ai, 2) + ai @ center <= b[i])
        seed_points = self._sample_segment(corridor.start, corridor.goal, self.sample_step)
        for seed in seed_points:
            constraints.append(A @ seed <= b + 1e-8)

        problem = cp.Problem(cp.Maximize(cp.log_det(B)), constraints)
        for solver in ("CLARABEL", "SCS"):
            if solver not in cp.installed_solvers():
                continue
            problem.solve(solver=solver, verbose=False)
            if problem.status in ("optimal", "optimal_inaccurate") and B.value is not None and center.value is not None:
                return np.asarray(center.value, dtype=float), np.asarray(B.value, dtype=float)
        raise RuntimeError("MVIE solver failed")

    def _compute_inscribed_ball(self, corridor: FIRICorridor) -> tuple[np.ndarray, np.ndarray]:
        A = np.asarray(corridor.A, dtype=float)
        b = np.asarray(corridor.b, dtype=float)
        center = 0.5 * (corridor.start + corridor.goal)
        center = corridor.project(center, max_iter=self.max_projection_iter * 2)
        margins = []
        for normal, offset in zip(A, b):
            normal_norm = float(np.linalg.norm(normal))
            if normal_norm <= 1e-12:
                continue
            margins.append((float(offset - np.dot(normal, center))) / normal_norm)
        radius = max(min(margins) if margins else self.min_clearance, 1e-6)
        return center, np.eye(3, dtype=float) * radius

    def _analytic_separating_plane(
        self,
        obs,
        start: np.ndarray,
        goal: np.ndarray,
        seed_points: np.ndarray,
    ) -> tuple[np.ndarray, float] | None:
        cls_name = type(obs).__name__
        if cls_name == "Sphere":
            obs_point = self._nearest_sphere_point(obs, start, goal)
        elif cls_name == "Cylinder":
            obs_point = self._nearest_cylinder_point(obs, start, goal)
        elif cls_name == "AABB":
            obs_point = self._nearest_aabb_point(obs, start, goal)
        else:
            return None

        seed = self._closest_point_on_segment(start, goal, obs_point)
        normal = seed - obs_point
        normal_norm = float(np.linalg.norm(normal))
        if normal_norm < 1e-9:
            seed = self._push_out_if_needed(seed)
            normal = seed - obs_point
            normal_norm = float(np.linalg.norm(normal))
            if normal_norm < 1e-9:
                return None

        normal = -normal / normal_norm
        offset = float(np.dot(normal, obs_point - normal * self.min_clearance))
        if not np.all(seed_points @ normal <= offset + 1e-8):
            return None
        return normal, offset

    def _nearest_sphere_point(self, obs, start: np.ndarray, goal: np.ndarray) -> np.ndarray:
        seed = self._closest_point_on_segment(start, goal, obs.center)
        direction = seed - obs.center
        norm = float(np.linalg.norm(direction))
        if norm < 1e-9:
            direction = self._safe_segment_normal(start, goal)
        else:
            direction = direction / norm
        return obs.center + direction * obs.radius

    def _nearest_cylinder_point(self, obs, start: np.ndarray, goal: np.ndarray) -> np.ndarray:
        samples = self._sample_segment(start, goal, max(self.sample_step, 0.2))
        best_seed = samples[0]
        best_surface = None
        best_dist = float("inf")
        z_min, z_max = obs.z_range
        center_xy = np.asarray(obs.center_xy, dtype=float)
        for seed in samples:
            radial = seed[:2] - center_xy
            radial_norm = float(np.linalg.norm(radial))
            if radial_norm < 1e-9:
                direction_xy = self._safe_segment_normal(start, goal)[:2]
                direction_norm = float(np.linalg.norm(direction_xy))
                if direction_norm < 1e-9:
                    direction_xy = np.array([1.0, 0.0], dtype=float)
                else:
                    direction_xy = direction_xy / direction_norm
            else:
                direction_xy = radial / radial_norm
            surface = np.array([
                center_xy[0] + obs.radius * direction_xy[0],
                center_xy[1] + obs.radius * direction_xy[1],
                min(max(seed[2], z_min), z_max),
            ], dtype=float)
            dist = float(np.linalg.norm(seed - surface))
            if dist < best_dist:
                best_dist = dist
                best_seed = seed
                best_surface = surface
        if best_surface is None:
            return best_seed.copy()
        return best_surface

    def _nearest_aabb_point(self, obs, start: np.ndarray, goal: np.ndarray) -> np.ndarray:
        samples = self._sample_segment(start, goal, max(self.sample_step, 0.2))
        best_seed = samples[0]
        best_surface = None
        best_dist = float("inf")
        min_corner = np.asarray(obs.min_corner, dtype=float)
        max_corner = np.asarray(obs.max_corner, dtype=float)
        center = 0.5 * (min_corner + max_corner)
        half = 0.5 * (max_corner - min_corner)
        for seed in samples:
            clipped = np.minimum(np.maximum(seed, min_corner), max_corner)
            if np.all(seed >= min_corner) and np.all(seed <= max_corner):
                q = np.abs(seed - center) - half
                axis = int(np.argmax(q))
                clipped = seed.copy()
                clipped[axis] = max_corner[axis] if seed[axis] >= center[axis] else min_corner[axis]
            dist = float(np.linalg.norm(seed - clipped))
            if dist < best_dist:
                best_dist = dist
                best_seed = seed
                best_surface = clipped
        if best_surface is None:
            return best_seed.copy()
        return best_surface

    def _safe_segment_normal(self, start: np.ndarray, goal: np.ndarray) -> np.ndarray:
        direction = np.asarray(goal, dtype=float) - np.asarray(start, dtype=float)
        norm = float(np.linalg.norm(direction))
        if norm < 1e-9:
            return np.array([1.0, 0.0, 0.0], dtype=float)
        direction = direction / norm
        ref = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(direction, ref))) > 0.9:
            ref = np.array([0.0, 1.0, 0.0], dtype=float)
        normal = np.cross(direction, ref)
        normal_norm = float(np.linalg.norm(normal))
        if normal_norm < 1e-9:
            return np.array([1.0, 0.0, 0.0], dtype=float)
        return normal / normal_norm

    def _sample_obstacle_surface(self, obs, center: np.ndarray, radius: float) -> list[np.ndarray]:
        cls_name = type(obs).__name__
        if cls_name == "Sphere":
            return self._sample_sphere(obs, center, radius)
        if cls_name == "Cylinder":
            return self._sample_cylinder(obs, center, radius)
        if cls_name == "AABB":
            return self._sample_aabb(obs, center, radius)
        return []

    def _sample_sphere(self, obs, center: np.ndarray, radius: float) -> list[np.ndarray]:
        if float(np.linalg.norm(obs.center - center)) > radius + obs.radius + self.min_clearance:
            return []
        count = max(24, int(np.ceil(obs.radius / self.obstacle_sample_step) * 12))
        dirs = self._fibonacci_sphere(count)
        return [obs.center + obs.radius * direction for direction in dirs]

    def _sample_cylinder(self, obs, center: np.ndarray, radius: float) -> list[np.ndarray]:
        cyl_center = np.array([obs.center_xy[0], obs.center_xy[1], 0.5 * (obs.z_range[0] + obs.z_range[1])], dtype=float)
        if float(np.linalg.norm(cyl_center - center)) > radius + obs.radius + self.min_clearance:
            return []
        n_theta = max(12, int(np.ceil(2.0 * np.pi * obs.radius / self.obstacle_sample_step)))
        n_z = max(2, int(np.ceil((obs.z_range[1] - obs.z_range[0]) / self.obstacle_sample_step)) + 1)
        samples: list[np.ndarray] = []
        z_values = np.linspace(obs.z_range[0], obs.z_range[1], n_z)
        for theta_idx in range(n_theta):
            theta = 2.0 * np.pi * theta_idx / n_theta
            x = obs.center_xy[0] + obs.radius * np.cos(theta)
            y = obs.center_xy[1] + obs.radius * np.sin(theta)
            for z in z_values:
                samples.append(np.array([x, y, z], dtype=float))
        return samples

    def _sample_aabb(self, obs, center: np.ndarray, radius: float) -> list[np.ndarray]:
        box_center = 0.5 * (obs.min_corner + obs.max_corner)
        half_diag = 0.5 * float(np.linalg.norm(obs.max_corner - obs.min_corner))
        if float(np.linalg.norm(box_center - center)) > radius + half_diag + self.min_clearance:
            return []
        samples: list[np.ndarray] = []
        mins = np.asarray(obs.min_corner, dtype=float)
        maxs = np.asarray(obs.max_corner, dtype=float)
        spans = maxs - mins
        counts = [max(2, int(np.ceil(span / self.obstacle_sample_step)) + 1) for span in spans]
        x_values = np.linspace(mins[0], maxs[0], counts[0])
        y_values = np.linspace(mins[1], maxs[1], counts[1])
        z_values = np.linspace(mins[2], maxs[2], counts[2])

        for x in x_values:
            for y in y_values:
                samples.append(np.array([x, y, mins[2]], dtype=float))
                samples.append(np.array([x, y, maxs[2]], dtype=float))
        for x in x_values:
            for z in z_values:
                samples.append(np.array([x, mins[1], z], dtype=float))
                samples.append(np.array([x, maxs[1], z], dtype=float))
        for y in y_values:
            for z in z_values:
                samples.append(np.array([mins[0], y, z], dtype=float))
                samples.append(np.array([maxs[0], y, z], dtype=float))
        return samples

    def _push_out_if_needed(self, point: np.ndarray) -> np.ndarray:
        point = np.asarray(point, dtype=float)
        if float(self.obstacle_field.signed_distance(point)) >= self.min_clearance:
            return point

        candidate = point.copy()
        eps = 0.05
        for _ in range(self.max_projection_iter * 2):
            sd = float(self.obstacle_field.signed_distance(candidate))
            if sd >= self.min_clearance:
                return candidate
            grad = np.array([
                self.obstacle_field.signed_distance(candidate + [eps, 0.0, 0.0]) - self.obstacle_field.signed_distance(candidate - [eps, 0.0, 0.0]),
                self.obstacle_field.signed_distance(candidate + [0.0, eps, 0.0]) - self.obstacle_field.signed_distance(candidate - [0.0, eps, 0.0]),
                self.obstacle_field.signed_distance(candidate + [0.0, 0.0, eps]) - self.obstacle_field.signed_distance(candidate - [0.0, 0.0, eps]),
            ], dtype=float) / (2.0 * eps)
            norm = float(np.linalg.norm(grad))
            if norm < 1e-12:
                break
            candidate = candidate + grad / norm * max(0.05, 0.2 * self.min_clearance)
        return candidate

    def _push_path_out_if_needed(self, path: np.ndarray) -> np.ndarray:
        path = np.asarray(path, dtype=float).copy()
        for i in range(len(path)):
            path[i] = self._push_out_if_needed(path[i])
        return path

    @staticmethod
    def _closest_point_on_segment(a: np.ndarray, b: np.ndarray, p: np.ndarray) -> np.ndarray:
        ab = b - a
        denom = float(np.dot(ab, ab))
        if denom < 1e-12:
            return a.copy()
        t = float(np.dot(p - a, ab) / denom)
        t = min(1.0, max(0.0, t))
        return a + t * ab

    @staticmethod
    def _sample_segment(start: np.ndarray, goal: np.ndarray, step: float) -> np.ndarray:
        dist = float(np.linalg.norm(goal - start))
        if dist < 1e-12:
            return np.asarray([start.copy()], dtype=float)
        n = max(2, int(np.ceil(dist / step)) + 1)
        ts = np.linspace(0.0, 1.0, n)
        return np.asarray([start + t * (goal - start) for t in ts], dtype=float)

    @staticmethod
    def _fibonacci_sphere(n: int) -> np.ndarray:
        pts = np.zeros((n, 3), dtype=float)
        if n <= 1:
            pts[0] = np.array([1.0, 0.0, 0.0], dtype=float)
            return pts
        golden = np.pi * (3.0 - np.sqrt(5.0))
        for i in range(n):
            y = 1.0 - 2.0 * i / float(n - 1)
            r = np.sqrt(max(0.0, 1.0 - y * y))
            theta = golden * i
            pts[i] = np.array([np.cos(theta) * r, y, np.sin(theta) * r], dtype=float)
        return pts

    @staticmethod
    def _remove_duplicate_points(path: np.ndarray) -> np.ndarray:
        if len(path) <= 1:
            return path
        keep = [np.asarray(path[0], dtype=float)]
        for point in path[1:]:
            point = np.asarray(point, dtype=float)
            if np.linalg.norm(point - keep[-1]) > 1e-9:
                keep.append(point)
        return np.asarray(keep, dtype=float)
