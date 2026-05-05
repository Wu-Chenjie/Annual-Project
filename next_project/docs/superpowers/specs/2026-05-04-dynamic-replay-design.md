# 动态地图在线重规划验证系统设计

> 编制日期：2026-05-04
> 适用范围：`next_project/cpp/`（已有文件小范围修改）、`next_project/web/`（新增）
> 核心目标：创建动态障碍物场景，验证 D\* Lite 增量重规划在环境变化时的可用性，对比 grid A\* 全局重规划

---

## 1. 验证目标与量化通过阈值

| 指标 | 阈值 | 说明 |
|------|------|------|
| 碰撞次数 | = 0 | 全程无碰 |
| 最终距目标距离 | < wp_radius_final | 到达目标 |
| 路径长度差 (D\* Lite vs A\*) | ≤ 5% | 增量规划不牺牲路径质量 |
| 平均规划耗时比 (A\* / D\* Lite) | ≥ 5x | 增量更新具有数量级优势 |
| 最大规划耗时 P95 比 (A\* / D\* Lite) | ≥ 5x | 最坏情况下仍优于全局重规划 |
| 重复性 | 每场景 N=5 次固定种子 | 排除随机性干扰 |

**核心待验证结论：** D\* Lite 在路径质量等价于 grid A\* 的前提下，平均耗时快 5x 以上，且耗时波动小（增量局部更新 vs 全局搜索）。

---

## 2. 修改范围原则

**原则：尽量新增，必要时小范围修改。** 所有修改点明确列出，不碰核心算法实现。

### 已有文件修改清单

| 文件 | 修改内容 | 原因 |
|------|---------|------|
| `cpp/CMakeLists.txt` | 追加 3 行：新增 `sim_dynamic_replay` executable target | 编译新入口 |
| `cpp/include/obstacles.hpp` | `ObstacleField` 新增 `remove_at(size_t)`、`clear()`、IDs 并行数组 | 支持动态删除障碍物 |
| `cpp/src/obstacle_scenario.cpp` | `setup_online()` 中为 `WindowReplanner` 设置 `DStarLite` 增量规划器 | D\* Lite 当前未接入重规划链路 |
| `cpp/include/replanner.hpp` | `set_incremental_planner` 接口已存在，不修改 | 仅调用方需改 |

### 新增文件清单

| 文件 | 职责 |
|------|------|
| `cpp/include/dynamic_scenario.hpp` | 动态场景编排 + 双路对比引擎 |
| `cpp/src/dynamic_main.cpp` | C++ 入口：读 JSON → 跑仿真 → 写 JSON |
| `cpp/external/nlohmann/json.hpp` | 第三方 header-only JSON 库，MIT 协议 |
| `web/server.py` | FastAPI 后端 |
| `web/dynamic_replay.html` | 单文件 Web 前端 |

---

## 3. 整体架构

```
┌─────────────────────────────────────────────────────────┐
│                Web 前端 (web/dynamic_replay.html)         │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────────┐ │
│  │ 配置面板  │ │ 3D 地图  │ │ 时间轴    │ │ D* Lite vs │ │
│  │ 预设选择  │ │ 障碍物    │ │ 事件标记  │ │ A* 对比    │ │
│  │ 自定义地图│ │ 编辑器    │ │ 回放控制  │ │ 指标面板   │ │
│  └────┬─────┘ └──────────┘ └──────────┘ └────────────┘ │
└───────┼─────────────────────────────────────────────────┘
        │ HTTP API (JSON)
┌───────▼─────────────────────────────────────────────────┐
│              Python FastAPI 后端 (web/server.py)          │
│  GET  /api/presets         → 返回所有预设配置             │
│  POST /api/simulate        → 调 C++ → 返回 replay.json   │
│  GET  /api/maps/<name>     → 返回地图 JSON               │
│  POST /api/maps/validate   → 验证自定义地图 JSON          │
│  GET  /                    → 返回 dynamic_replay.html    │
└───────┬─────────────────────────────────────────────────┘
        │ subprocess (跨平台路径: tempfile + filesystem::path)
┌───────▼─────────────────────────────────────────────────┐
│     C++ sim_dynamic_replay (cpp/src/dynamic_main.cpp)    │
│  使用 nlohmann/json 解析输入 JSON                         │
│  双路: D\* Lite (增量) vs grid A\* (全局重规划)           │
│  DynamicScenarioRunner 编排动态事件 + 逐帧记录             │
│  输出 replay.json                                        │
└─────────────────────────────────────────────────────────┘
```

---

## 4. C++ 端设计

### 4.1 ObstacleField 修改（obstacles.hpp）

当前 `ObstacleField` 只有 `add_*` 和只读 `obstacles()`，缺少删除和 ID 机制。
追加以下能力：

```cpp
class ObstacleField {
public:
    // --- 现有方法保持不变 ---
    void add_aabb(const Vec3& mn, const Vec3& mx);
    void add_sphere(const Vec3& center, double r);
    void add_cylinder(const Vec3& center_xy, double r, double zmin, double zmax);
    // ... signed_distance, is_collision, size, obstacles 不变

    // === 新增 ===

    // 带 ID 的添加（自动生成 dyn_0, dyn_1 ... 或用户指定）
    std::string add_aabb_id(const Vec3& mn, const Vec3& mx, const std::string& id = "");
    std::string add_sphere_id(const Vec3& center, double r, const std::string& id = "");
    std::string add_cylinder_id(const Vec3& cxy, double r, double zmin, double zmax, const std::string& id = "");

    // 按索引删除（供按障碍物下标操作）
    void remove_at(size_t index);

    // 按 ID 删除（供 DynamicEvent REMOVE 引用稳定 ID）
    bool remove_by_id(const std::string& id);

    // 清空所有障碍物
    void clear();

    // 获取 ID 列表
    const std::vector<std::string>& ids() const { return ids_; }

    // 重建后重新生成 ID（供 clear + 批量 add 后调用）
    void regenerate_ids(const std::string& prefix = "obs_");

private:
    std::vector<ObstacleVariant> obstacles_;   // 已有
    std::vector<std::string> ids_;             // 新增：与 obstacles_ 一一对应
    int dyn_counter_ = 0;                      // 新增：动态障碍物自动编号
};
```

**ID 命名规则：**
- 加载静态地图时调用 `regenerate_ids("obs_")` → `obs_0`, `obs_1`, `obs_2` ...
- 动态添加时调用 `add_*_id(...)` → 自动生成 `dyn_0`, `dyn_1` ...
- REMOVE 事件引用稳定字符串 ID，不依赖索引

### 4.2 JSON 序列化方案

当前 `map_loader.cpp` 是手写简易解析器，仅支持固定格式的地图文件。`dynamic_main.cpp` 需要处理结构复杂得多的输入/输出 JSON，引入 `nlohmann/json`（MIT 协议，单头文件约 28KB）：

```
cpp/external/nlohmann/json.hpp   # 从 GitHub release 下载，放 external/ 目录
```

**使用的 JSON 序列化/反序列化：**
- `DynamicSimInput` ↔ JSON（输入）
- `ReplayOutput` ↔ JSON（输出）
- 利用 `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE` 宏自动生成序列化代码

### 4.3 dynamic_scenario.hpp

```cpp
#pragma once
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <chrono>
#include "obstacles.hpp"
#include "config.hpp"
#include "obstacle_scenario.hpp"
#include "replanner.hpp"
#include "astar_planner.hpp"
#include "dstar_lite.hpp"

namespace sim {

// ---- 障碍物描述（JSON 序列化用） ----
struct ObstacleDesc {
    std::string id;       // 稳定字符串 ID
    std::string type;     // "sphere" | "aabb" | "cylinder"
    Vec3 center_or_min;
    Vec3 size_or_max;     // sphere: (r,0,0), aabb: max_corner, cylinder: (r,z_min,z_max)
};

// ---- 动态障碍物事件 ----
struct DynamicEvent {
    double t;
    enum Action { ADD, REMOVE };
    Action action;
    // ADD 时携带
    ObstacleDesc obstacle;
    // REMOVE 时引用已有障碍物 ID（字符串）
    std::string target_id;
};

// ---- 仿真输入 ----
struct DynamicSimInput {
    ObstacleConfig base_config;
    std::string map_file;
    std::vector<DynamicEvent> events;
    std::vector<std::string> compare_planners;  // ["dstar_lite", "astar"]
    int repeat_count = 5;                       // 固定种子重复次数
};

// ---- 规划器结果 ----
struct PlannerResult {
    std::vector<Vec3> path;
    double compute_time_ms;
    std::string phase;          // "local" | "incremental" | "global" (仅 D* Lite 有)
    bool replan_triggered;
    bool success;
};

// ---- 单帧快照 ----
struct FrameSnapshot {
    double t;
    Vec3 leader_pose;
    std::vector<Vec3> follower_poses;
    std::array<double, 6> sensor_readings;
    std::unordered_map<std::string, PlannerResult> planner_results;
    bool collision;
    bool obstacle_changed;
};

// ---- 汇总指标 ----
struct SummaryMetrics {
    int total_replans = 0;
    double avg_compute_ms = 0;
    double max_compute_ms = 0;
    double p95_compute_ms = 0;           // P95 耗时
    int collisions = 0;
    double path_length_m = 0;
    double final_dist_to_goal = 0;       // 终点距离
    std::unordered_map<std::string, double> phase_distribution;
};

// ---- 回放输出 ----
struct ReplayOutput {
    struct Metadata {
        std::string map_file;
        std::string scenario = "dynamic_obstacle";
        int step_count;
        double dt;
        double total_time;
        int repeat_index = 0;            // 第几次重复
        int seed = 42;
    } metadata;

    std::vector<ObstacleDesc> static_obstacles;
    std::array<Vec3, 2> bounds;
    std::vector<DynamicEvent> dynamic_events;
    std::vector<FrameSnapshot> frames;
    std::unordered_map<std::string, SummaryMetrics> summaries;
};

// ---- 动态场景执行器 ----
class DynamicScenarioRunner {
public:
    explicit DynamicScenarioRunner(const DynamicSimInput& input, int seed = 42);

    /// 运行仿真，返回完整回放数据
    ReplayOutput run();

    /// 批量运行 N 次（不同种子），返回汇总
    std::vector<ReplayOutput> run_batch();

private:
    /// 在仿真时间 t 应用所有到期事件
    void apply_events(double t);

    /// 运行单路规划器并计时
    PlannerResult run_one_planner(const std::string& kind,
                                   const Vec3& leader_pose,
                                   const std::array<double,6>& sensor,
                                   const Vec3& goal);

    /// ADD 事件 → 添加障碍物到 ObstacleField + 更新栅格
    void handle_add_obstacle(const DynamicEvent& e);
    /// REMOVE 事件 → 从 ObstacleField 删除 + 清除栅格
    void handle_remove_obstacle(const DynamicEvent& e);
    /// 将 ADD/REMOVE 变更转换为 OccupancyGrid 的体素更新列表
    std::vector<std::pair<std::array<int,3>, bool>> event_to_grid_changes(const DynamicEvent& e);

    DynamicSimInput input_;
    int seed_;

    // 仿真组件
    ObstacleConfig config_;
    std::unique_ptr<ObstacleScenarioSimulation> sim_;
    ObstacleField obstacles_;
    OccupancyGrid grid_;

    // D* Lite 专用（增量规划器独立实例）
    std::unique_ptr<DStarLite> dstar_lite_;
    // Grid A*（对照，每次从头搜索）
    // 使用 astar_plan() 自由函数

    // WindowReplanner 引用（sim_ 内部）
    WindowReplanner* replanner_ = nullptr;

    // 事件索引（避免重复应用）
    size_t event_cursor_ = 0;
};

}  // namespace sim
```

### 4.4 D\* Lite 接入重规划链路

当前 C++ 端 `WindowReplanner` 有 `set_incremental_planner()` 接口，但 `ObstacleScenarioSimulation::setup_online()` 从未调用。修正方案：

**不修改 `setup_online()` 本身**（保持向后兼容），而是在 `DynamicScenarioRunner` 构造中完成 D\* Lite 的创建和接入：

```cpp
DynamicScenarioRunner::DynamicScenarioRunner(const DynamicSimInput& input, int seed)
    : input_(input), seed_(seed), config_(input.base_config)
{
    // ... 初始化仿真组件 ...

    if (config_.planner_mode == "online") {
        // 创建 D* Lite 增量规划器
        dstar_lite_ = std::make_unique<DStarLite>(grid_, start, goal);
        // 通过已有接口注入 WindowReplanner
        replanner_->set_incremental_planner(
            std::make_unique<DStarLite>(grid_, start, goal));
    }
}
```

**ADD/REMOVE 事件 → D\* Lite 增量更新链：**

```
DynamicEvent (ADD, t=5.0)
  → handle_add_obstacle()
    → obstacles_.add_sphere_id(...)
    → event_to_grid_changes() → 计算该障碍占据的体素列表 [(idx, true), ...]
    → dstar_lite_->update_cells(changed_cells)  // 通知 D* Lite 边代价变更
    → grid_.data[idx] = 1                        // 同步 OccupancyGrid

DynamicEvent (REMOVE, t=10.0)
  → handle_remove_obstacle()
    → event_to_grid_changes() → 计算该障碍释放的体素列表 [(idx, false), ...]
    → dstar_lite_->update_cells(changed_cells)
    → grid_.data[idx] = 0
    → obstacles_.remove_by_id(target_id)
```

### 4.5 A\* 基线定义

**对比基线：grid A\*（`astar_plan()` 自由函数），非 Hybrid A\*。**

理由：
- `astar_plan()` 是经典栅格 A\*，每次传入当前位姿 + 目标 + 完整 OccupancyGrid 从头搜索
- 与 D\* Lite 共享相同的栅格表示和六邻域图结构，对比公平
- Hybrid A\* 引入了运动学约束，不在本次验证范围

通过 `compare_planners` 数组控制启用哪些对比规划器：
- `["dstar_lite", "astar"]` — 默认，D\* Lite vs grid A\*
- `["dstar_lite"]` — 仅 D\* Lite（调试用）
- Web 前端对比算法选项中移除 RRT\*（在线模式未实现）

### 4.6 dynamic_main.cpp

```cpp
#include "dynamic_scenario.hpp"
#include "external/nlohmann/json.hpp"
#include <fstream>
#include <filesystem>
#include <iostream>

using json = nlohmann::json;

// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE 宏定义所有 struct 的 JSON 映射
// (ObstacleDesc, DynamicEvent, DynamicSimInput, ReplayOutput, ...)

int main(int argc, char* argv[]) {
    // 用法: sim_dynamic_replay <input.json> [-o <output.json>]
    namespace fs = std::filesystem;

    std::string input_path = argv[1];
    std::string output_path = (argc >= 4 && std::string(argv[2]) == "-o")
        ? argv[3]
        : (fs::path(input_path).parent_path() / "replay_output.json").string();

    // 1. 读取输入 JSON
    std::ifstream ifs(input_path);
    json j; ifs >> j;
    DynamicSimInput input = j.get<DynamicSimInput>();

    // 2. 批量运行（N 次固定种子）
    std::vector<ReplayOutput> results;
    for (int run = 0; run < input.repeat_count; ++run) {
        int seed = 42 + run;
        DynamicScenarioRunner runner(input, seed);
        auto output = runner.run();
        output.metadata.repeat_index = run;
        output.metadata.seed = seed;
        results.push_back(output);
    }

    // 3. 序列化输出
    json out = results;  // vector<ReplayOutput> → JSON 数组
    std::ofstream ofs(output_path);
    ofs << out.dump(2);  // pretty print, indent=2

    std::cout << "Done. " << results.size() << " runs → " << output_path << "\n";
    return 0;
}
```

---

## 5. Python 后端设计

### 5.1 server.py

```python
"""动态重规划验证系统 - FastAPI 后端

启动: uvicorn server:app --host 0.0.0.0 --port 8765
依赖: fastapi, uvicorn (pip install fastapi uvicorn)
"""

import json
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, HTMLResponse

app = FastAPI(title="Dynamic Replay Server")

# 路径配置（跨平台）
PROJECT_ROOT = Path(__file__).resolve().parent.parent
CPP_BUILD_DIR = PROJECT_ROOT / "cpp" / "build"
CPP_EXE = CPP_BUILD_DIR / "Release" / "sim_dynamic_replay.exe" if os.name == "nt" \
     else CPP_BUILD_DIR / "sim_dynamic_replay"
MAPS_DIR = PROJECT_ROOT / "maps"
WEB_DIR = PROJECT_ROOT / "web"

# 预设配置映射（与 config.py / config.hpp 同步）
# 从 config.hpp 中的 get_config() 预设名称提取
PRESETS = {
    "basic":             "基础编队验证",
    "obstacle":          "简单障碍物避障",
    "warehouse":         "工业仓库在线A*",
    "warehouse_a":       "仓库A*+ESDF+Danger",
    "warehouse_online":  "仓库在线简化版",
    "warehouse_danger":  "仓库GNN双模式",
    "fault_tolerance":   "容错测试",
    "school_corridor":   "学校走廊离线",
    "school_corridor_online": "学校走廊在线",
    "company_cubicles":  "公司格子间离线",
    "company_cubicles_online": "公司格子间在线",
    "meeting_room":      "会议室离线",
    "meeting_room_online": "会议室在线",
    "laboratory":        "实验室离线",
    "laboratory_online": "实验室在线",
    "custom":            "自定义",
}


@app.get("/api/presets")
async def get_presets():
    """返回所有预设配置列表，供前端下拉选择"""
    return {"presets": [
        {"id": k, "name": v} for k, v in PRESETS.items()
    ]}


@app.post("/api/simulate")
async def simulate(request: Request):
    """接收前端配置 → 调 C++ → 返回 replay JSON 数组"""
    body = await request.json()

    # 构造输入 JSON
    sim_input = body  # 前端直接发 DynamicSimInput 结构

    with tempfile.TemporaryDirectory() as tmpdir:
        input_path = os.path.join(tmpdir, "input.json")
        output_path = os.path.join(tmpdir, "output.json")

        with open(input_path, "w", encoding="utf-8") as f:
            json.dump(sim_input, f, indent=2, ensure_ascii=False)

        # 调用 C++ 引擎
        result = subprocess.run(
            [str(CPP_EXE), input_path, "-o", output_path],
            capture_output=True, text=True, timeout=300,
        )

        if result.returncode != 0:
            raise HTTPException(500, f"C++ simulation failed: {result.stderr}")

        with open(output_path, "r", encoding="utf-8") as f:
            replay_data = json.load(f)

    return {"results": replay_data}  # array of ReplayOutput


@app.get("/api/maps/{name}")
async def get_map(name: str):
    """返回 maps/ 目录下的 JSON 地图文件内容"""
    map_path = MAPS_DIR / f"{name}.json"
    if not map_path.exists():
        raise HTTPException(404, f"Map not found: {name}")
    with open(map_path, "r", encoding="utf-8") as f:
        return json.load(f)


@app.get("/api/maps")
async def list_maps():
    """列出所有可用地图文件名"""
    maps = [p.stem for p in MAPS_DIR.glob("*.json")]
    return {"maps": maps}


@app.get("/", response_class=HTMLResponse)
async def index():
    """返回前端页面"""
    html_path = WEB_DIR / "dynamic_replay.html"
    with open(html_path, "r", encoding="utf-8") as f:
        return HTMLResponse(f.read())
```

**依赖管理：** 在 `web/requirements.txt` 单独声明 `fastapi`、`uvicorn`，不加入项目根 `requirements.txt`。

---

## 6. Web 前端设计

### 6.1 技术选型

| 组件 | 选型 | 理由 |
|------|------|------|
| 2D Canvas | 原生 Canvas API | 轻量俯视图 |
| 3D 渲染 | Three.js (CDN importmap) | 多角度观察，OrbitControls |
| 图表 | Chart.js (CDN) | 耗时曲线、路径长度对比 |
| 样式 | 内联 CSS | 单文件零依赖 |

### 6.2 配置面板

```
┌─────────────────────────────────────────────────────────┐
│  ⚙ 仿真配置                                              │
│  ┌─────────────────────────────────────────────────────┐│
│  │ 预设场景: [warehouse            ▼] [加载预设]        ││
│  │                                                     ││
│  │ ── 地图 ──────────────────────────────────────       ││
│  │ 地图文件: [sample_warehouse      ▼]                  ││
│  │ 或 [上传自定义地图 JSON...]                           ││
│  │                                                     ││
│  │ ── 算法 ──────────────────────────────────────       ││
│  │ 对比基线: ☑ D* Lite (增量)  ☑ Grid A* (全局)        ││
│  │ 规划模式: ◉ online  ○ offline                       ││
│  │ 重复次数: [5] 次（固定种子：42,43,44,45,46）          ││
│  │                                                     ││
│  │ ── 参数 ──────────────────────────────────────       ││
│  │ 栅格分辨率: [0.4] m    仿真时长: [65.0] s             ││
│  │ 安全裕度: [0.3] m      编队间距: [0.5] m              ││
│  │ 重规划间隔: [0.4] s    规划前瞻: [6.0] m              ││
│  │ ☐ Danger模式  ☐ 自适应间隔  ☐ 故障注入               ││
│  │                                                     ││
│  │ ── 动态障碍物事件 ──────────────────────────────      ││
│  │ ┌──┬────────┬────────┬───────────────────────────┐  ││
│  │ │# │ 时间(s)│ 操作   │ 障碍物描述                  │  ││
│  │ ├──┼────────┼────────┼───────────────────────────┤  ││
│  │ │1 │ 5.0    │ 添加 ▲ │ Sphere c=(15,10,3) r=1.5   │  ││
│  │ │2 │ 10.0   │ 移除 ▼ │ obs_3 (仓库立柱)            │  ││
│  │ │3 │ 15.0   │ 添加 ▲ │ AABB (20,5,0)→(22,8,4)    │  ││
│  │ └──┴────────┴────────┴───────────────────────────┘  ││
│  │ [+ 添加事件]  [从地图选择障碍物填入移除事件]           ││
│  └─────────────────────────────────────────────────────┘│
│                                                         │
│  [▶ 运行仿真]  [重置为预设默认值]                          │
└─────────────────────────────────────────────────────────┘
```

**预设切换行为：**
1. 用户选择预设（如 `warehouse_danger`）→ 点击 [加载预设]
2. 前端请求 `/api/presets` 获取该预设对应的 `ObstacleConfig` 默认参数
3. 自动填充所有参数字段，地图文件、规划模式、Danger 模式等
4. 用户可在此基础上微调任意参数，或切换到 `custom` 完全自定义

**自定义地图：**
- 可从 `/api/maps` 下拉选择已有地图
- 可上传自定义 JSON 文件（前端验证格式后传给后端 `/api/maps/validate`）
- 编辑器模式可直接在地图上增删障碍物，导出为 JSON 复用

### 6.3 3D 地图视图

```
┌──────────────────────────────────────────────────┐
│  🗺 地图视图                    [2D俯视] [3D ▾]   │
│  ┌──────────────────────────────────────────────┐│
│  │              Three.js 渲染区                   ││
│  │  · AABB 灰色半透明  · Sphere 蓝色半透明        ││
│  │  · Cylinder 黄色半透明  (统一 opacity=0.3)     ││
│  │  · 绿色实线 = D* Lite 当前路径                 ││
│  │  · 橙色虚线 = Grid A* 当前路径                 ││
│  │  · 灰色细线 = 历史路径（淡出）                  ││
│  │  · 蓝色球 = 领航机      · 红色球 = 目标点       ││
│  │  · 青色小球群 = 从机编队                        ││
│  │                                              ││
│  │  OrbitControls: 鼠标拖拽旋转 | 滚轮缩放         ││
│  │                 右键平移 | 中键复位视角          ││
│  └──────────────────────────────────────────────┘│
│  ┌──────────────────────────────────────────────┐│
│  │ 编辑模式: [开启/关闭]                           ││
│  │ 类型: [立方体 ▾]  尺寸/位置: [...]              ││
│  │ [+ 添加]  [选中障碍物: obs_0]  [删除选中]       ││
│  │ [导出当前地图为 JSON]                          ││
│  └──────────────────────────────────────────────┘│
└──────────────────────────────────────────────────┘
```

### 6.4 场景编辑器

- **添加障碍物**：打开编辑模式 → 选择类型 → 在地图上点击放置 → 调整尺寸 → 确认后加入"动态事件表"（t=0 添加，或指定触发时间）
- **选中障碍物**：点击 3D 场景中的障碍物 → 高亮边框 → 属性面板显示 ID/类型/参数
- **删除障碍物**：选中后点击 [删除选中] → 自动生成 REMOVE 事件（引用障碍物 ID），加入事件表
- **导出**：将所有当前障碍物导出为项目 `maps/*.json` 格式，可保存复用

### 6.5 时间轴

- 红色竖线标记障碍物变更时刻
- 拖拽时间轴手柄任意跳转
- 播放速度：0.5x / 1x / 2x / 4x
- 不闪烁

### 6.6 对比视图 + 指标面板

```
┌────────────────────┬─────────────────────────────┐
│  D* Lite (增量)     │  Grid A* (全局重规划)        │
│  ───────────────── │  ──────────────────────     │
│  路径长度:   28.3m  │  路径长度:   28.3m          │
│  重规划次数: 12     │  重规划次数: 12              │
│  平均耗时:   3.1ms  │  平均耗时:   42.7ms         │
│  最大耗时:   8.5ms  │  最大耗时:   78.2ms         │
│  P95耗时:    7.2ms  │  P95耗时:    72.1ms         │
│  碰撞次数:   0   ✅ │  碰撞次数:   0   ✅         │
│  终点距离:  0.3m ✅ │  终点距离:  0.3m ✅         │
│  耗时比(A*/D*):     │  路径差: +0.0%              │
│    平均: 13.8x ✅   │                             │
│    P95:  10.0x ✅   │                             │
│  调度层分布:        │  规划失败: 0                 │
│    local:     40%   │                             │
│    incremental:45%  │                             │
│    global:    15%   │                             │
└────────────────────┴─────────────────────────────┘

⏱ 规划耗时曲线 (ms) — Chart.js
📈 路径长度对比 (m) — Chart.js
📊 调度层分布饼图 — Chart.js
```

---

## 7. 回放 JSON 格式（修订版）

```json
{
    "metadata": {
        "map_file": "maps/sample_warehouse.json",
        "scenario": "dynamic_obstacle",
        "step_count": 1300,
        "dt": 0.05,
        "total_time": 65.0,
        "repeat_index": 0,
        "seed": 42
    },
    "static_map": {
        "bounds": [[-3,-3,0], [45,28,10]],
        "obstacles": [
            {"id": "obs_0",  "type": "sphere",   "center": [7,6,3.5],  "size_or_max": [1.2,0,0]},
            {"id": "obs_1",  "type": "aabb",     "center_or_min": [5,3,0], "size_or_max": [9,9,0.4]},
            {"id": "obs_2",  "type": "cylinder", "center_or_min": [10.5,2,0], "size_or_max": [0.15,0,6]}
        ]
    },
    "dynamic_events": [
        {"t": 5.0,  "action": "add",    "obstacle": {"type": "sphere", "center_or_min": [15,10,3], "size_or_max": [1.5,0,0]}},
        {"t": 10.0, "action": "remove", "target_id": "obs_3"},
        {"t": 15.0, "action": "add",    "obstacle": {"type": "aabb", "center_or_min": [20,5,0], "size_or_max": [22,8,4]}}
    ],
    "frames": [
        {
            "t": 0.0,
            "leader_pose": [4.2, 13.2, 5.7],
            "followers": [[3.8,12.5,5.7], [4.6,13.8,5.7], [3.8,13.8,5.7]],
            "sensor_readings": [8.0, 4.2, 8.0, 2.8, 8.0, 8.0],
            "dstar_lite": {
                "path": [[4.2,13.2,5.7], [5.0,13.0,5.5]],
                "phase": "local",
                "replan_triggered": false,
                "compute_time_ms": 2.3,
                "success": true
            },
            "astar": {
                "path": [[4.2,13.2,5.7], [5.0,13.0,5.5]],
                "compute_time_ms": 45.1,
                "success": true
            },
            "collision": false,
            "obstacle_changed": false
        }
    ],
    "summary": {
        "dstar_lite": {
            "total_replans": 12,
            "avg_compute_ms": 3.1,
            "max_compute_ms": 8.5,
            "p95_compute_ms": 7.2,
            "collisions": 0,
            "path_length_m": 28.3,
            "final_dist_to_goal": 0.3,
            "phase_distribution": {"local": 0.40, "incremental": 0.45, "global": 0.15}
        },
        "astar": {
            "total_replans": 12,
            "avg_compute_ms": 42.7,
            "max_compute_ms": 78.2,
            "p95_compute_ms": 72.1,
            "collisions": 0,
            "path_length_m": 28.3,
            "final_dist_to_goal": 0.3
        }
    }
}
```

---

## 8. 非目标

- 动态障碍物预测
- SLAM 自建图
- 实物部署
- GPU 加速
- Web 端实时仿真（离线计算 + 在线回放）

---

## 9. 风险与对策

| 风险 | 影响 | 对策 |
|------|------|------|
| ObstacleField 新增 remove 接口破坏兼容性 | 已有代码编译报错 | 仅追加方法，不修改已有接口签名，确保编译通过 |
| nlohmann/json 引入增加编译时间 | header-only 约 28KB，影响小 | 仅 dynamic_main.cpp 引用，不影响 sim_core 库 |
| C++ 双路仿真耗时 | 用户体验差 | C++ O3 优化已有 A\* <5ms 基准；验证阶段限制仿真步数 |
| Web 端加载大回放 JSON 卡顿 | 回放不流畅 | 前端帧抽样渲染，默认每 5 帧取 1 帧 |
| 跨平台路径差异 | Windows 运行失败 | Python 端用 tempfile；C++ 端用 std::filesystem::path |
| Three.js CDN 加载失败 | 3D 视图不可用 | 降级到 2D Canvas 模式 |
