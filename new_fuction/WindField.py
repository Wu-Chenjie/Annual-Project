import numpy as np
# 风场模型：稳态风 + 一阶马尔科夫湍流
'''
稳态风：固定的风速向量，模拟持续的风环境
湍流：使用一阶马尔科夫过程生成随机风速扰动，模拟风的随机变化。湍流的强度由 turbulence_std 控制，tau 控制扰动的时间相关性。
每次调用 sample(dt) 时，都会更新湍流状态，并返回当前的总风速（稳态风 + 湍流）。
通过调整参数，可以模拟不同的风环境，从平静到强风，适用于无人机飞行控制器的鲁棒性测试。
使用了ornstein-uhlenbeck过程进行湍流模拟，保证了湍流的时间相关性和连续性，避免了纯随机噪声的非物理特性。
'''
class WindField:
    def __init__(self, steady=(0.0, 0.0, 0.0), turbulence_std=0.1, tau=0.5, seed=None):
        '''
        :param steady: 稳态风速 (wx, wy, wz) [m/s]
        :param turbulence_std: 湍流强度（标准差） [m/s]
        :param tau: 湍流时间常数 [s]
        :param seed: 随机种子，确保可重复性
        '''
        self.steady = np.array(steady)
        self.turbulence_std = turbulence_std
        self.tau = tau  # 一阶马尔科夫过程时间常数
        self.state = np.zeros(3)
        self.rng = np.random.default_rng(seed)

    def sample(self, dt):
        """
        生成风场扰动（稳态风+湍流）
        一阶马尔科夫过程（湍流随时间连续变化，不是纯随机）
        总风场 = 稳态风 + 湍流状态
        dw = -w/tau·dt + 湍流项
        湍流项√(2/tau)·√dt·随机数是一阶马尔科夫过程的标准离散化形式，保证湍流的时间相关性；
        """
        w = self.state
        #计算湍流扰动的增量dw：均值回复项 + 随机激励项
        dw = -w / self.tau * dt + self.turbulence_std * np.sqrt(2 / self.tau) * np.sqrt(dt) * self.rng.standard_normal(3)
        #更新马尔科夫状态
        self.state = w + dw
        return self.steady + self.state
