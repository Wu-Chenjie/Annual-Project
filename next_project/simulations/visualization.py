"""仿真可视化模块。

用途
----
在仿真结束后自动生成轨迹图和误差图，用于直观评估编队控制效果。

原理
----
1) 三维轨迹图：显示领航机与从机在空间中的实际飞行路径。
2) 实时误差分量图：使用三个子图显示 e_x/e_y/e_z 随时间的变化。
3) 误差统计图：并列展示每架从机的平均误差和最大误差。

"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from core.obstacles import AABB, Sphere, Cylinder, ObstacleField


class SimulationVisualizer:
    """仿真结果可视化器。"""

    def __init__(self, output_dir: str = "outputs", dpi: int = 140):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.dpi = int(dpi)

    def plot_all(self, result: dict, show: bool = False) -> dict:
        """生成全部图表并保存为 PNG 文件。"""
        saved = {
            "trajectory": self._plot_trajectory(result),
            "error_3d": self._plot_realtime_error_3d(result),
            "error_stats": self._plot_error_stats(result),
        }
        # 障碍物相关图表（仅在存在障碍物数据时生成）
        if result.get("obstacles") is not None and len(result["obstacles"]) > 0:
            saved["trajectory"] = self._plot_trajectory(result)  # 已在上面绘制，含障碍物
        if result.get("replan_events") and len(result["replan_events"]) > 0:
            saved["replan_events"] = self._plot_replan_events(result)
        if result.get("sensor_logs") is not None and len(result["sensor_logs"]) > 0:
            saved["sensor_distance"] = self._plot_sensor_distance(result)
        if show:
            plt.show()
        else:
            plt.close("all")
        return saved

    def _plot_trajectory(self, result: dict) -> str:
        time = result["time"]
        leader = result["leader"]
        followers = result["followers"]
        task_waypoints = result.get("task_waypoints", None)
        replanned_waypoints = result.get("replanned_waypoints", None)
        obstacles = result.get("obstacles", None)
        planned_path = result.get("planned_path", None)
        executed_path = result.get("executed_path", None)

        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection="3d")

        # 优化3D背景和网格线 (Modern clean look)
        ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
        ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
        ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
        ax.grid(color="#ececec", linestyle="--", linewidth=0.5)

        # 先绘制障碍物（半透明在底层）
        if obstacles is not None and len(obstacles) > 0:
            self._draw_obstacles(ax, obstacles)

        # 领航机使用更具质感的颜色和线条
        ax.plot(leader[:, 0], leader[:, 1], leader[:, 2], linewidth=3.0, color="#2563eb", alpha=0.9, label="Leader Path")
        ax.scatter(
            leader[0, 0], leader[0, 1], leader[0, 2],
            marker="o", s=80, color="#22c55e", edgecolors="white", linewidths=1.2,
            label="Leader Start", zorder=7,
        )
        ax.scatter(
            leader[-1, 0], leader[-1, 1], leader[-1, 2],
            marker="*", s=180, color="#ef4444", edgecolors="white", linewidths=1.2,
            label="Leader End", zorder=7,
        )
        
        # 从机使用柔和的现代色系
        follower_colors = ["#10b981", "#f59e0b", "#6366f1", "#ec4899", "#8b5cf6"]
        for i, f in enumerate(followers):
            ax.plot(
                f[:, 0], f[:, 1], f[:, 2],
                linestyle="--", linewidth=1.8, color=follower_colors[i % len(follower_colors)], alpha=0.8,
                label=f"Follower {i + 1} Path",
            )

        # 原始任务航点
        if task_waypoints is not None and len(task_waypoints) > 0:
            ax.scatter(
                task_waypoints[:, 0], task_waypoints[:, 1], task_waypoints[:, 2],
                marker="o", s=45, facecolors="none", edgecolors="#6b7280",
                linewidths=1.2, label="Task Waypoints", zorder=8, alpha=0.7,
            )
            ax.scatter(
                task_waypoints[0, 0], task_waypoints[0, 1], task_waypoints[0, 2],
                marker="s", s=100, color="#10b981", edgecolors="white", linewidths=1.0,
                label="Task Start", zorder=9,
            )
            ax.scatter(
                task_waypoints[-1, 0], task_waypoints[-1, 1], task_waypoints[-1, 2],
                marker="X", s=100, color="#ef4444", edgecolors="white", linewidths=1.0,
                label="Task End", zorder=9,
            )

        # 规划路径与在线重规划点
        if planned_path is not None and len(planned_path) > 0:
            ax.plot(planned_path[:, 0], planned_path[:, 1], planned_path[:, 2],
                    linestyle=":", linewidth=1.5, color="#6b7280", alpha=0.7, label="Offline Planned Path")
            ax.scatter(
                planned_path[0, 0], planned_path[0, 1], planned_path[0, 2],
                marker="o", s=40, color="#4b5563", edgecolors="white", linewidths=0.5,
                label="Plan Start", zorder=9,
            )
            ax.scatter(
                planned_path[-1, 0], planned_path[-1, 1], planned_path[-1, 2],
                marker="*", s=90, color="#4b5563", edgecolors="white", linewidths=0.5,
                label="Plan End", zorder=9,
            )

        if replanned_waypoints is not None and len(replanned_waypoints) > 0:
            ax.plot(
                replanned_waypoints[:, 0], replanned_waypoints[:, 1], replanned_waypoints[:, 2],
                linestyle="-.", linewidth=1.5, color="#8b5cf6", alpha=0.8, label="Replanned Path",
            )
            ax.scatter(
                replanned_waypoints[:, 0], replanned_waypoints[:, 1], replanned_waypoints[:, 2],
                marker=".", s=20, color="#8b5cf6", alpha=0.6, label="Replanned Points", zorder=6,
            )

        # 美化标题和标签
        ax.set_title("UAV Formation 3D Trajectory", pad=20, fontsize=14, fontweight="bold", color="#1f2937")
        ax.set_xlabel("X Position (m)", labelpad=10, color="#374151")
        ax.set_ylabel("Y Position (m)", labelpad=10, color="#374151")
        ax.set_zlabel("Z Position (m)", labelpad=10, color="#374151")
        
        # 调整刻度颜色
        ax.tick_params(axis='x', colors='#4b5563')
        ax.tick_params(axis='y', colors='#4b5563')
        ax.tick_params(axis='z', colors='#4b5563')

        # 图例移至图外以避免遮挡轨迹，调整视角
        ax.legend(loc="center left", bbox_to_anchor=(1.05, 0.5), frameon=True, fancybox=True, shadow=True, facecolor="white")
        ax.view_init(elev=25, azim=-50)  # 改进默认视角


        file_path = self.output_dir / "trajectory_3d.png"
        fig.tight_layout()
        fig.savefig(file_path, dpi=self.dpi)
        return str(file_path)

    def _plot_realtime_error_3d(self, result: dict) -> str:
        """实时误差分量图（3 个子图）。"""
        time = result["time"]
        error_vectors = result["error_vectors"]

        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

        colors = ["#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf"]
        for i, e_vec in enumerate(error_vectors):
            color = colors[i % len(colors)]
            axes[0].plot(time, e_vec[:, 0], color=color, linewidth=1.7, label=f"Follower {i + 1}")
            axes[1].plot(time, e_vec[:, 1], color=color, linewidth=1.7, label=f"Follower {i + 1}")
            axes[2].plot(time, e_vec[:, 2], color=color, linewidth=1.7, label=f"Follower {i + 1}")

        axes[0].set_title("Real-time Error Components vs Time")
        axes[0].set_ylabel("e_x (m)")
        axes[1].set_ylabel("e_y (m)")
        axes[2].set_ylabel("e_z (m)")
        axes[2].set_xlabel("Time (s)")

        for ax in axes:
            ax.grid(True, alpha=0.3)
        axes[0].legend(loc="upper right", ncols=2)

        file_path = self.output_dir / "error_realtime_3d.png"
        fig.tight_layout()
        fig.savefig(file_path, dpi=self.dpi)
        return str(file_path)

    def _plot_error_stats(self, result: dict) -> str:
        means = np.array(result["metrics"]["mean"], dtype=float)
        maxs = np.array(result["metrics"]["max"], dtype=float)

        labels = [f"F{i + 1}" for i in range(len(means))]
        x = np.arange(len(labels))
        width = 0.35

        fig, ax = plt.subplots(figsize=(9, 5))
        ax.bar(x - width / 2, means, width=width, label="Mean Error", color="#4e79a7")
        ax.bar(x + width / 2, maxs, width=width, label="Max Error", color="#e15759")
        ax.axhline(0.3, color="black", linestyle="--", linewidth=1.2, label="0.3m Threshold")

        ax.set_title("Formation Error Statistics")
        ax.set_xlabel("Follower Index")
        ax.set_ylabel("Error (m)")
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.legend(loc="upper right")
        ax.grid(True, axis="y", alpha=0.3)

        file_path = self.output_dir / "error_statistics.png"
        fig.tight_layout()
        fig.savefig(file_path, dpi=self.dpi)
        return str(file_path)

    def _draw_obstacles(self, ax, obstacles: ObstacleField) -> None:
        """在 3D 轴上绘制障碍物。

        原理
        ----
        AABB → 立方体面片（半透明灰色），Sphere → 线框球面，
        Cylinder → 参数化曲面。
        """
        for obs in obstacles:
            if isinstance(obs, AABB):
                mn, mx = obs.min_corner, obs.max_corner
                vertices = [
                    [[mn[0], mn[1], mn[2]], [mx[0], mn[1], mn[2]], [mx[0], mx[1], mn[2]], [mn[0], mx[1], mn[2]]],
                    [[mn[0], mn[1], mx[2]], [mx[0], mn[1], mx[2]], [mx[0], mx[1], mx[2]], [mn[0], mx[1], mx[2]]],
                    [[mn[0], mn[1], mn[2]], [mx[0], mn[1], mn[2]], [mx[0], mn[1], mx[2]], [mn[0], mn[1], mx[2]]],
                    [[mx[0], mn[1], mn[2]], [mx[0], mx[1], mn[2]], [mx[0], mx[1], mx[2]], [mx[0], mn[1], mx[2]]],
                    [[mx[0], mx[1], mn[2]], [mn[0], mx[1], mn[2]], [mn[0], mx[1], mx[2]], [mx[0], mx[1], mx[2]]],
                    [[mn[0], mx[1], mn[2]], [mn[0], mn[1], mn[2]], [mn[0], mn[1], mx[2]], [mn[0], mx[1], mx[2]]],
                ]
                ax.add_collection3d(Poly3DCollection(vertices, alpha=0.25, facecolors="gray", edgecolors="#555555", linewidths=0.3))
            elif isinstance(obs, Sphere):
                u = np.linspace(0, 2 * np.pi, 16)
                v = np.linspace(0, np.pi, 12)
                cx, cy, cz = obs.center
                r = obs.radius
                x = cx + r * np.outer(np.cos(u), np.sin(v))
                y = cy + r * np.outer(np.sin(u), np.sin(v))
                z = cz + r * np.outer(np.ones_like(u), np.cos(v))
                ax.plot_wireframe(x, y, z, color="gray", alpha=0.3, linewidth=0.3)
            elif isinstance(obs, Cylinder):
                theta = np.linspace(0, 2 * np.pi, 24)
                z_vals = np.linspace(obs.z_range[0], obs.z_range[1], 8)
                th, zv = np.meshgrid(theta, z_vals)
                cx, cy = obs.center_xy
                x = cx + obs.radius * np.cos(th)
                y = cy + obs.radius * np.sin(th)
                ax.plot_surface(x, y, zv, alpha=0.25, color="gray", linewidth=0)

    def _plot_replan_events(self, result: dict) -> str:
        """重规划事件时间-高度图。"""
        time = result["time"]
        leader = result["leader"]
        events = result.get("replan_events", [])

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(time, leader[:, 2], color="#1f77b4", linewidth=1.5, label="Leader Altitude")
        if events:
            et = [e["t"] for e in events]
            ez = np.interp(et, time, leader[:, 2])
            ax.scatter(et, ez, marker="x", s=50, color="red", zorder=5, label="Replan Event")
        ax.set_title("Replan Events During Flight")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Altitude (m)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

        file_path = self.output_dir / "replan_events.png"
        fig.tight_layout()
        fig.savefig(file_path, dpi=self.dpi)
        return str(file_path)

    def _plot_sensor_distance(self, result: dict) -> str:
        """六向传感器距离曲线图（仅 online 模式）。"""
        sensor_logs = result["sensor_logs"]
        time = result["time"]
        N = min(len(time), len(sensor_logs))

        fig, ax = plt.subplots(figsize=(10, 5))
        labels = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
        colors = ["#d62728", "#ff9896", "#2ca02c", "#98df8a", "#1f77b4", "#aec7e8"]
        for di in range(6):
            ax.plot(time[:N], sensor_logs[:N, di], color=colors[di], linewidth=1.2, label=labels[di])
        ax.set_title("Sensor Distance Readings")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance (m)")
        ax.legend(loc="upper right", ncols=6)
        ax.grid(True, alpha=0.3)

        file_path = self.output_dir / "sensor_distance.png"
        fig.tight_layout()
        fig.savefig(file_path, dpi=self.dpi)
        return str(file_path)
