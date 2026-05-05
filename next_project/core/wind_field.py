"""风场模型。

用途
----
模拟室内/近地环境中的稳态风与湍流扰动，为控制器抗扰测试提供统一输入。

原理
----
将总风速写为：

    v_w(t) = v_steady + w(t)

其中 w(t) 使用 Ornstein-Uhlenbeck（OU）过程建模：

    dw = -(1/tau) * w * dt + sigma * sqrt(2/tau) * dW_t

离散化后（Euler-Maruyama）：

    w_{k+1} = w_k - (w_k/tau) * dt + sigma * sqrt(2/tau) * sqrt(dt) * N(0, I)

这种模型相比独立白噪声更符合真实气流的时间相关性。

"""

from __future__ import annotations

import numpy as np


class WindField:
    """稳态风 + 湍流风场。"""

    def __init__(self, steady=(0.0, 0.0, 0.0), turbulence_std: float = 0.1,
                 tau: float = 0.5, seed: int | None = None):
        self.steady = np.array(steady, dtype=float)
        self.turbulence_std = float(turbulence_std)
        self.tau = float(tau)
        self.state = np.zeros(3, dtype=float)
        self.rng = np.random.default_rng(seed)

    def sample(self, dt: float) -> np.ndarray:
        dw = -self.state / self.tau * dt + self.turbulence_std * np.sqrt(2.0 / self.tau) * np.sqrt(dt) * self.rng.standard_normal(3)
        self.state = self.state + dw
        return self.steady + self.state
