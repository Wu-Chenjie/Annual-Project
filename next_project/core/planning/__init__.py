"""路径规划子包。

用途
----
提供统一路径规划接口，支持离线（Dijkstra / RRT* / Informed RRT*）与在线
（Hybrid A* + WindowReplanner 三级分层）两条路线；
新增 GNN 双模式架构（Safe/Danger 调度 + 可见图 + GNN 规划器）。

原理
----
所有规划器继承 Planner 抽象基类，策略模式切换；
RRT* 系列支持 KD-Tree 加速 + 收缩半径 + B 样条平滑；
在线重规划采用 local(滑动窗口) → incremental(D* Lite) → global(Informed RRT*) 分层调度；
GNN 规划器在可见图顶点上运行活动扩散，O(n²) 替代栅格搜索 O(N³)。

"""

from .base import Planner, PlannerError
from .astar import AStar, TurnConstrainedAStar
from .hybrid_astar import HybridAStar
from .dijkstra import Dijkstra
from .rrt_star import RRTStar
from .informed_rrt_star import InformedRRTStar
from .dstar_lite import DStarLite
from .replanner import WindowReplanner, RiskAdaptiveReplanInterval
from .esdf import compute_esdf, CostAwareGrid
from .visibility_graph import VisibilityGraph
from .gnn_planner import GNNPlanner
from .dual_mode import DualModeScheduler, FormationAPF
from .firi import FIRIRefiner, FIRICorridor

__all__ = [
    "Planner",
    "PlannerError",
    "AStar",
    "TurnConstrainedAStar",
    "HybridAStar",
    "Dijkstra",
    "RRTStar",
    "InformedRRTStar",
    "DStarLite",
    "WindowReplanner",
    "RiskAdaptiveReplanInterval",
    "compute_esdf",
    "CostAwareGrid",
    "VisibilityGraph",
    "GNNPlanner",
    "DualModeScheduler",
    "FormationAPF",
    "FIRIRefiner",
    "FIRICorridor",
]
