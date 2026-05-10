# 四旋翼无人机模型及控制系统

## 简介
本项目实现了基于简化旋翼模型的四旋翼动力学仿真、多无人机编队仿真及位置-姿态串级PID控制，同时支持二阶滑模控制模块扩展。

## 模块说明
- Rotor.py：单旋翼简化推力模型。
- ControlAllocator.py：四旋翼推力分配和转速计算。
- WindField.py：稳态+湍流风场模型。
- Packaged_Drone_Dynamics_Model.py：飞行器动力学模型及状态更新。
- Controller.py：位置与姿态串级PID控制器。
- SecondOrderSMC.py：滑模控制逻辑模块。
- HybridAttitudeController.py：PID与滑模混合控制示例。
- Formation_Flight.py：多无人机编队飞行仿真示例。

## 运行方法
1. 安装依赖：`numpy, matplotlib, plotly`
2. 执行 `python Formation_Flight.py` 运行编队仿真。

## 规划
后续将支持：
- BEMT旋翼模型替换
- 复杂风场模型集成
- 滑模控制参数在线自适应调节
