# DroneParams Profile 说明

> 关联代码：`core/drone_params.py`、`core/drone.py`、`core/rotor.py`、`simulations/formation_simulation.py`  
> 用途：说明当前无人机动力学参数 profile 的来源、适用范围和边界，避免把仿真 profile 误读为实测标定结果。

---

## 一、定位

`DroneParams` 将质量、惯量、旋翼推力系数、力矩系数、电机时间常数、最大转速和旋翼模型选择从历史硬编码中抽出，形成可切换 profile。当前 profile 用于**仿真尺度对照和回归守门**，不是实机标定结论。

当前已接入链路：

```text
SimulationConfig.drone_profile
  -> get_drone_params()
  -> Drone / QuaternionDrone
  -> Rotor / BEMRotor
  -> Controller mass and inertia
```

每个 profile 会在 `DroneParams.__post_init__()` 中检查：

- `mass > 0`
- 惯量三个对角项为正
- `kf`、`km` 为正
- `hover_omega < omega_max`
- `rotor_model` 只能为 `simple` 或 `bem`

---

## 二、Profile 清单

| Profile | 用途 | 旋翼模型 | 适用范围 | 边界 |
|---|---|---|---|---|
| `default_1kg` | 历史回归基线 | `simple` | 默认仿真、短期验收、跨版本对比 | 保持既有参数，不代表某一真实机型 |
| `indoor_micro` | 室内微型机尺度对照 | `simple` | Crazyflie 量级的质量/尺度敏感性实验 | 未接入真实电机曲线和飞控延迟标定 |
| `light_uav_regulatory` | 低空安全/标准化演示 | `bem` | 低空 profile、风险报告尺度说明 | 地图尺度与室内仿真不同，不能直接与室内安全间隔混用 |

---

## 三、参数摘要

| Profile | mass (kg) | inertia diag (kg m^2) | arm length (m) | kf | km | omega max (rad/s) | hover omega (rad/s) |
|---|---:|---|---:|---:|---:|---:|---:|
| `default_1kg` | 1.000 | (0.01, 0.01, 0.02) | 0.200 | 1.0e-5 | 2.0e-7 | 1000 | 495.2 |
| `indoor_micro` | 0.027 | (1.4e-5, 1.4e-5, 2.2e-5) | 0.046 | 1.7e-8 | 2.4e-10 | 2200 | 1973.5 |
| `light_uav_regulatory` | 1.800 | (0.021, 0.021, 0.038) | 0.280 | 1.5e-5 | 3.0e-7 | 1100 | 542.5 |

数值由代码中的 profile 计算得到，后续如引入实测数据，应同步更新本表和对应测试。

---

## 四、使用建议

默认仍使用：

```python
drone_profile = "default_1kg"
```

切换 profile 时必须同时关注：

- 控制器增益是否仍适合当前质量/惯量。
- `leader_max_vel`、`leader_max_acc`、`follower_max_vel`、`follower_max_acc` 是否超过当前 profile 可跟踪范围。
- 低空标准 profile 不应用来证明室内厘米级避障结论。
- C++ 子集若未同步对应字段，应在 `docs/capability-matrix.md` 中标注为“子集/待同步”。

---

## 五、后续标定入口

中期计划中的标定工作应新增在：

```text
experiments/calibration/
```

当前第一版已提供：

```powershell
python -m experiments.calibration.drone_params_calibration `
  outputs/calibration_smoke.csv `
  --name smoke_candidate `
  --output outputs/calibration/smoke_candidate.json
```

输入 CSV 支持以下已预处理物理参数列：

```text
mass, ixx, iyy, izz, arm_length, k_drag, j_rotor,
kf, km, tau_motor, omega_max,
bem_radius, bem_chord, bem_num_blades, bem_theta_tip, bem_theta_root,
rotor_model
```

脚本会对数值列取中位数，缺失字段回退到 `default_1kg` 或指定 `--base-profile`，并输出候选 `DroneParams` JSON。

没有真实飞行日志前，不应将任何 profile 描述为“实测标定”。
