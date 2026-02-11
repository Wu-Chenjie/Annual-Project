import numpy as np
from Packaged_Drone_Dynamics_Model import Drone
from Controller import Controller

def run_simulation(kp, ki, kd, show_progress=False):
    """
    运行一次仿真，返回总误差（Cost）。
    Cost = 位置误差的平方和 + 惩罚项（如超调或控制量过大）
    """
    # 限制参数为非负
    if kp < 0 or ki < 0 or kd < 0:
        return float('inf')

    dt = 0.01 # 加快仿真步长以节省时间
    drone = Drone(dt=dt)
    drone.set_initial_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], dt=dt, max_acc=10.0)
    
    # 实例化控制器
    controller = Controller(kp_pos=kp, ki_pos=ki, kd_vel=kd, max_acc=10.0, max_integral=5.0)
    
    # 测试任务：飞向一个点并悬停 (阶跃响应)
    target_pos = np.array([10.0, 10.0, 5.0])
    
    total_error = 0.0
    sim_time = 15.0 # 15秒仿真
    steps = int(sim_time / dt)
    
    for _ in range(steps):
        pos, vel = drone.get_state()
        
        # 计算控制量
        acc = controller.compute_acceleration(pos, vel, target_pos, dt=dt)
        drone.update_state(acc)
        
        # 累积误差 (均方误差 MSE)
        dist_sq = np.sum((target_pos - pos)**2)
        total_error += dist_sq
        
        # 额外惩罚：如果发散了（距离过远），直接返回无穷大
        if dist_sq > 1000.0:
            return float('inf')

    return total_error / steps

def twiddle(tol=0.2):
    """
    Twiddle 算法 (坐标上升法) 自动寻找最优 PID 参数
    """
    # 初始猜测 [Kp, Ki, Kd]
    p = [1.0, 0.0, 1.0]
    # 初始步长 (搜索范围)
    dp = [0.5, 0.05, 0.5]
    
    best_err = run_simulation(p[0], p[1], p[2])
    print(f"初始状态: Kp={p[0]:.2f}, Ki={p[1]:.2f}, Kd={p[2]:.2f} -> 误差={best_err:.2f}")
    
    it = 0
    # 当步长之和大于容差时继续搜索
    while sum(dp) > tol:
        it += 1
        print(f"迭代 {it}: 调整量 sum(dp)={sum(dp):.4f}")
        
        for i in range(len(p)):
            # 尝试增加参数
            p[i] += dp[i]
            err = run_simulation(p[0], p[1], p[2])
            
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
                print(f"  > 参数[{i}]增加优化成功: p={p}, err={err:.2f}")
            else:
                # 增加无效，尝试减小参数
                p[i] -= 2 * dp[i] # 回到原点再减去 dp
                err = run_simulation(p[0], p[1], p[2])
                
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                    print(f"  > 参数[{i}]减小优化成功: p={p}, err={err:.2f}")
                else:
                    # 都不行，还原并减小步长
                    p[i] += dp[i]
                    dp[i] *= 0.9
                    
    return p, best_err

if __name__ == "__main__":
    print("开始 PID 参数自动整定 (Twiddle 算法)...")
    best_params, min_err = twiddle()
    print("\n" + "="*40)
    print("整定完成！推荐参数：")
    print(f"Kp (比例): {best_params[0]:.4f}")
    print(f"Ki (积分): {best_params[1]:.4f}")
    print(f"Kd (微分): {best_params[2]:.4f}")
    print(f"最小误差: {min_err:.4f}")
    print("="*40)
