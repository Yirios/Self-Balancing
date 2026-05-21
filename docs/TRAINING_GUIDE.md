# 自平衡机器人强化学习训练指南

## 概述

本项目的目标：为 WHEELTEC B585 二阶自平衡机器人训练一个强化学习控制器，替代原有的 LQR 控制器，并部署到 STM32F103RCT6 硬件。

**最终成果**：一个 10→32→32→2 的 MLP 神经网络，1400/1400 步完美平衡，同时跟踪速度指令（直行、转弯），Flash 占用 5.9KB，推理耗时约 1ms（5ms 中断周期内安全运行）。

---

## 第一步：环境建模（构建 Gymnasium 仿真）

### 1.1 物理模型

从 MATLAB 仿真提取物理参数和状态空间模型：

| 参数 | 值 | 含义 |
|------|-----|------|
| m₁ | 0.9 kg | 车身质量 |
| m₂ | 0.1 kg | 摆杆质量 |
| R | 0.0335 m | 轮子半径 |
| L₁ | 0.126 m | 车身长度 |
| L₂ | 0.390 m | 摆杆长度 |
| g | 9.8 m/s² | 重力加速度 |

### 1.2 状态空间推导

通过拉格朗日方程推导连续时间线性化动力学：

```
p(4×4) · q̈ = q(4×10) · [x; u]
```

其中 `p` 包含惯性和耦合项，`q` 包含重力和输入耦合项。求解得到：

```
ẋ = A·x + B·u     (连续时间，8 状态，2 动作)
x_{k+1} = G·x_k + H·u_k     (离散时间，ZOH，Ts=5ms)
```

**关键文件**：[balancing_robot/dynamics.py](balancing_robot/dynamics.py)

### 1.3 状态与动作

**8 个物理状态**：
```
x = [θ_L, θ_R, θ₁, θ₂, θ̇_L, θ̇_R, θ̇₁, θ̇₂]
     左轮   右轮   车身  摆杆   左轮速  右轮速  车身角速  摆杆角速
    转角   转角   倾角  倾角
```

**2 个动作**：`u = [u_L, u_R]`（左右轮角加速度，rad/s²，范围 ±5000）

**2 个速度指令**（加入观测使 RL 学会运动控制）：
```
obs = [θ_L, θ_R, θ₁, θ₂, θ̇_L, θ̇_R, θ̇₁, θ̇₂, v_cmd, ω_cmd]
                                                └────┬────┘
                                          前进速度(m/s) 转弯速度(rad/s)
```

### 1.4 奖励函数

```
reward = -(state_cost + control_cost + velocity_tracking_cost)

state_cost    = xᵀ·diag([0.01,0.01,10,50,0.001,0.001,5,5])·x
control_cost  = 1e-5 × (u_L² + u_R²)
velocity_cost = 5.0 × [(v_actual - v_cmd)² + (ω_actual - ω_cmd)²]
```

权重设计：车身倾角（10）和摆杆倾角（50）权重大 → 优先保持平衡 → 再学习速度跟踪。

### 1.5 终止条件

- 车身倾角 |θ₁| > 45°
- 摆杆倾角 |θ₂| > 60°
- 最大步数 1400（= 7 秒）

---

## 第二步：行为克隆预训练（BC）

### 2.1 为什么需要 BC

直接从头训练 PPO 无法学会平衡。原因：

- 8 维不稳定系统中，随机动作几乎必然在 50 步内倒下
- PPO 永远看不到"成功"轨迹，无法学习

**解决方案**：用 LQR 控制器作为"老师"，收集示范数据，用监督学习训练一个初始化策略。

### 2.2 数据收集

使用 LQR + Ornstein-Uhlenbeck（OU）噪声收集 200 条轨迹：

```python
# LQR 速度跟踪：u = -K·(x - x_ref)
x_ref = [0, 0, 0, 0, target_L_dot, target_R_dot, 0, 0]

# OU 噪声：时间相关，比白噪声更接近真实探索
ou += -0.3·ou·dt + 0.1·ε·√dt

# 噪声逐渐衰减
noise_scale = 20 × (1 - episode/200)
u = u_lqr + ou × noise_scale
```

每步记录 `(obs_10d, action_2d)`，共约 28 万组数据。

### 2.3 网络训练

```
BC Model: Linear(10→32) → ReLU → Linear(32→32) → ReLU → Linear(32→2)
参数: 1474, Loss: MSE(action_pred, action_lqr)
Epochs: 50, Batch: 256, LR: 1e-3
```

训练后 BC 策略评估：**1400/1400 步完美平衡 + 速度跟踪**。

---

## 第三步：KL 正则化 PPO 微调

### 3.1 为什么不直接用 BC

BC 只是模仿 LQR，不会超越它。理论上 RL 可以找到比 LQR 更优的策略（更高效的控制，更少的能量消耗）。

### 3.2 PPO 微调的问题

直接对 BC 初始化做 PPO 微调会导致**灾难性遗忘**：

- 原因：PPO 梯度更新冲掉 BC 学到的权重，agent 快速偏离 → 摔倒 → 收集不到好数据 → 恶性循环
- 表现：Episode 从 1300 步急剧降到 100 步

**解决方案**：KL 正则化 — 在 PPO loss 中加入对 BC 策略偏离的惩罚。

### 3.3 奖励函数设计

PPO 的训练目标是最大化**累计折扣奖励** $R = \sum_{t=0}^{T} \gamma^t r_t$，其中每步奖励 $r_t$ 由四项组成：

#### 3.3.1 完整奖励函数

```
r = -( r_state + r_control + r_velocity + r_bc )
     └──┬──┘  └───┬───┘  └───┬────┘  └─┬─┘
    平衡成本   控制成本   速度跟踪    BC约束
```

#### 3.3.2 各项详解

**(1) 状态成本 r_state（保持平衡）**：

$$
r_{state} = \sum_{i=1}^{8} w_i \cdot x_i^2
$$

| 权重 | 值 | 对应状态 | 作用 |
|------|-----|----------|------|
| w₁, w₂ | 0.01 | θ_L, θ_R（轮子转角） | 不惩罚——允许自由移动 |
| w₃ | 10 | θ₁（车身倾角） | **主要平衡目标** |
| w₄ | 50 | θ₂（摆杆倾角） | **最优先平衡目标** |
| w₅, w₆ | 0.001 | θ̇_L, θ̇_R（轮子角速度） | 轻度惩罚——允许速度跟踪 |
| w₇, w₈ | 5 | θ̇₁, θ̇₂（车身/摆杆角速度） | 阻尼振荡 |

**(2) 控制成本 r_control（节能）**：

$$r_{control} = 10^{-5} \times (u_L^2 + u_R^2)$$

系数极小（1e-5）因为动作量级为 100-5000 rad/s²。典型值：u=1000 时，cost=0.02，远小于状态成本。

**(3) 速度跟踪成本 r_velocity（运动控制）**：

$$r_{velocity} = 5.0 \times \left[(v_{actual} - v_{cmd})^2 + (\omega_{actual} - \omega_{cmd})^2\right]$$

其中 $v_{actual} = \frac{R}{2}(\dot{\theta}_L + \dot{\theta}_R)$，$\omega_{actual} = \frac{R}{D}(\dot{\theta}_R - \dot{\theta}_L)$。

权重 5.0 的选取：平衡优先（state cost 对倾斜敏感度 > 速度跟踪需求）。在车身倾角 θ₁=0.1 rad 时，state_cost ≈ 10×0.01=0.1；对应速度误差 0.14 m/s 时，velocity_cost ≈ 5.0×0.02=0.1。两者在同一量级，策略需要同时优化。

**(4) BC 正则化 r_bc（可选，仅训练时）**：

$$r_{bc} = \beta \times \|u - u_{LQR}\|^2$$

当 `bc_beta > 0` 时，在环境 reward 中加入对 LQR 动作偏差的惩罚。这是可选的辅助项，与 KL-PPO 的 bc_coef（在 loss 中）是两个独立的正则化路径。

#### 3.3.3 典型数值示例

| 场景 | θ₁ | v_err | u | r_state | r_control | r_velocity | r_total |
|------|:--:|:--:|:--:|:---:|:---:|:---:|:---:|
| 完美平衡+速度跟踪 | 0.01° | 0.05 m/s | 50 | ~0 | ~0 | 0.01 | **-0.01** |
| 轻微倾斜 | 5° | 0.1 m/s | 200 | 0.76 | 0.08 | 0.05 | **-0.89** |
| 接近摔倒 | 40° | — | 2000 | 48.8 | 8.0 | — | **-56.8** |

#### 3.3.4 实际训练中的 Reward Scale

训练时使用 `VecNormalize(norm_reward=True)` 对奖励做 running mean/std 归一化：

$$r_{norm} = (r - \mu_{running}) / \sigma_{running}$$

归一化后 PPO 看到的奖励大致在 [-3, 3] 范围，value network 学习稳定（explained_variance > 0.9）。

### 3.4 KL 正则化 PPO 的 Loss 设计

```
loss_total = loss_PPO_clipped     ← 标准 PPO：最大化 advantage × ratio（clip）
           + ent_coef × H(π)      ← 熵奖励：鼓励探索（ent_coef=0.01）
           + vf_coef × loss_value ← 价值函数：MSE(returns, V(s))（vf_coef=0.5）
           + bc_coef × loss_bc    ← 新增：惩罚策略均值偏离 BC
```

其中：

$$loss_{bc} = \mathbb{E}_{s \sim rollout}\left[\|\mu_\pi(s) - \mu_{BC}(s)\|^2\right]$$

- $\mu_\pi(s)$：PPO 策略对状态 s 输出的 Gaussian 均值（确定性动作）
- $\mu_{BC}(s)$：冻结的 BC 网络对相同状态 s 的输出
- $bc\_coef = 0.01$：正则化强度

核心原理：`||μ_ppo - μ_bc||²` 等价于 KL(N(μ_ppo,σ²) || N(μ_bc,σ²)) 乘以常数。它禁止 PPO 策略均值偏离 BC 太远，保护了平衡能力，同时允许在 BC 附近小范围探索改进。

**关键文件**：[kl_ppo.py](kl_ppo.py)（子类化 PPO，覆盖 `train()` 方法）

### 3.5 训练超参数

| 参数 | 值 | 说明 |
|------|-----|------|
| learning_rate | 1e-4 | 低 lr 避免冲掉 BC |
| clip_range | 0.1 | 小裁剪范围 |
| ent_coef | 0.01 | 鼓励探索 |
| bc_coef | 0.01 | BC 正则化强度 |
| w_vel | 5.0 | 速度跟踪权重（在 reward 中） |
| R_weight | 1e-5 | 控制成本权重（在 reward 中） |
| n_steps | 1024 | 每轮收集步数 |
| n_epochs | 5 | 每轮更新次数 |
| total_timesteps | 1.5M | 约 180 秒 |

---

## 第四步：网络缩减与 STM32 适配

### 4.1 初始 [64,64] 网络

- 参数：4866 float × 4 = 19.5KB Flash
- MAC：4736 次乘加
- STM32F103 @ 72MHz 无 FPU：估算 ~4ms

**太接近 5ms 上限，不安全。**

### 4.2 缩减到 [32,32]

```
[10→64→64→2]: 4866 参数, 4736 MAC, ~19KB Flash, ~4ms
[10→32→32→2]: 1474 参数, 1408 MAC, ~5.9KB Flash, ~1ms  ✅
```

性能几乎无退化（ep_len 1400 持平，v_err 0.12→0.14），换来 4 倍安全裕量。

---

## 第五步：C 导出与 STM32 集成

### 5.1 权重导出

[export_to_c.py](export_to_c.py) 提取 PyTorch 权重，生成 C 头文件：

```bash
uv run python export_to_c.py -m bc_model.pt -o balance_nn.h
```

生成的 `balance_nn.h` 包含：
- `nn_w0[320]`, `nn_b0[32]` — 第一层权重
- `nn_w1[1024]`, `nn_b1[32]` — 第二层权重
- `nn_w2[64]`, `nn_b2[2]` — 输出层权重
- `nn_predict(input, output)` — 前向推理函数

数值验证：**Python 和 C 输出完全一致**（atol=1e-5）。

### 5.2 STM32 集成

在 [control.c](control.c) 中三处修改：

**1. 添加头文件**（第 20 行）：
```c
#include "balance_nn.h"
```

**2. 控制模式选择**（第 117-119 行）：
```c
if (Control_mode == 0) {
    // 原始 LQR 控制器
} else {
    RL_Controller();  // NN 推理
}
```

**3. RL 控制器实现**（第 173-186 行）：
```c
void RL_Controller() {
    float v_cmd = (Target_theta_L_dot + Target_theta_R_dot) * 0.01675f;
    float omega_cmd = (Target_theta_R_dot - Target_theta_L_dot) * 0.209375f;
    float state[10] = {
        theta_L, theta_R, theta_1, theta_2,
        theta_L_dot, theta_R_dot, theta_dot_1, theta_dot_2,
        v_cmd, omega_cmd
    };
    float action[2];
    nn_predict(state, action);
    u_L = action[0];
    u_R = action[1];
}
```

下游 PI 速度环（`Incremental_L/R`）和电机驱动（`Set_Pwm`）完全不变。

---

## 实验过程中遇到的问题与解决方案

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| **PPO 从零训练永远不收敛** | 8D 不稳定系统，随机动作必倒 | 先用 LQR 收集示范 → BC 预训练 |
| **PPO 微调后 episode 从 1300 降到 100** | 无约束下 PPO 冲掉 BC 权重 | 加入 KL 正则化（BC action-MSE 惩罚） |
| **Reward 爆炸到 -1e7** | Q_diag 权重太大（80/300），value loss 无法学习 | 缩小 reward 权重 100x + VecNormalize |
| **Action space ±500 导致 LQR 被截断** | LQR 动作 ~1000 rad/s²，被截断到 500 后无法平衡 | 改回 ±5000，BC 预训练解决探索问题 |
| **VecNormalize 观测归一化导致策略失效** | BC 在原始观测上训练，归一化后输入分布变了 | norm_obs=False，只 norm_reward=True |
| **[64,64] 网络太高耗时** | 4ms 推理太接近 5ms 中断周期 | 缩减到 [32,32]，1ms 推理，性能不变 |
| **2D 渲染无法展示差速转向** | 侧视图中左右轮重叠 | 实现 3D matplotlib 渲染 + 路径轨迹 |

---

## 最终性能

| 指标 | BC [32,32] | PPO [32,32] |
|------|-----------|------------|
| Episode 长度 | 1400/1400 | 1400/1400 |
| 速度跟踪误差 | 0.14 m/s | 0.13 m/s |
| 转弯跟踪误差 | 0.51 rad/s | 0.61 rad/s |
| Flash 占用 | 5.9 KB | 5.9 KB |
| RAM 暂存 | 256 B | 256 B |
| 推理 MAC | 1408 | 1408 |
| STM32 估算 | ~1 ms | ~1 ms |

---

## 文件索引

| 文件 | 功能 |
|------|------|
| `balancing_robot/dynamics.py` | 物理模型：p, q → A, B → G, H |
| `balancing_robot/env.py` | Gymnasium 环境（10D 观测，速度跟踪 reward） |
| `pretrain_bc.py` | LQR 示范收集 + BC 预训练 |
| `kl_ppo.py` | KL 正则化 PPO（子类化） |
| `train_ppo_reg.py` | BC 正则化 PPO 训练脚本 |
| `visualize.py` | 3D 动画生成 |
| `export_to_c.py` | 模型 → C 头文件导出 |
| `test_env.py` | 环境验证测试 |
| `control.c` | STM32 控制代码（已集成 RL） |
| `backup_balance_only/` | 纯平衡版本备份 |
