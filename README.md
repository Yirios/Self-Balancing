# Self-Balancing Robot RL

二阶自平衡机器人（双轮倒立摆）的强化学习控制器。从 MATLAB Simulink LQR 仿真迁移到 Python Gymnasium + PyTorch，最终导出为 C 头文件部署到 STM32F103RCT6。

## 系统

**8 物理状态 + 2 速度指令 = 10 维观测**：

```
obs = [θ_L, θ_R, θ₁, θ₂, θ̇_L, θ̇_R, θ̇₁, θ̇₂, v_cmd, ω_cmd]
```

**2 连续动作**：`[u_L, u_R]`（轮子角加速度 rad/s²，±5000）

**5ms 离散时间步长**（ZOH，匹配 MPU6050 中断周期）

物理参数来源于 WHEELTEC B585 二阶平衡机器人的 MATLAB 仿真（`ref/` 目录）。

## 项目结构

```
.
├── balancing_robot/
│   ├── dynamics.py          # 物理模型：p, q → A, B → G, H
│   ├── env.py               # BalancingRobotEnv（10D 观测，速度跟踪）
│   └── __init__.py          # 注册 "BalancingRobot-v0"
├── pretrain_bc.py           # LQR 示范收集 → BC 预训练
├── kl_ppo.py                # KL 正则化 PPO（子类化）
├── train_ppo_reg.py         # BC 正则化 PPO 训练（推荐）
├── train_ppo.py             # 普通 PPO 训练
├── visualize.py             # 3D 仿真动画 GIF
├── export_to_c.py           # 模型权重 → C 头文件
├── control.c                # STM32 控制代码（已集成 RL）
├── test_env.py              # 环境验证测试
├── pyproject.toml           # 依赖配置
├── README.md
├── TRAINING_GUIDE.md        # 详细训练指南
├── REPORT.md                # 课程实验报告
└── assets/                  # 输出动画
```

## 方法

1. **BC 预训练**：LQR 控制器 + OU 噪声收集 200 条轨迹 → 训练 MLP(10→32→32→2) 模仿 LQR
2. **KL 正则化 PPO**：`loss = PPO_loss + bc_coef × ||μ_ppo − μ_bc||²`，保护 BC 知识不被冲掉
3. **1.5M steps 微调**：在 BC 附近搜索更优控制策略，保持 100% 平衡成功率

## 结果（50 次随机速度指令测试）

| 模型 | Ep Len | 成功率 | 速度跟踪 MAE | 转弯跟踪 MAE |
|------|:---:|:---:|:---:|:---:|
| LQR（基准） | 1400 | 100% | 0.148 m/s | 0.433 rad/s |
| BC [32,32] | 1400 | 100% | 0.140 m/s | 0.449 rad/s |
| **KL-PPO [32,32]** | **1400** | **100%** | **0.113 m/s** | 0.654 rad/s |

KL-PPO 在直线速度跟踪上超越 LQR 24%，转弯跟踪待数据均衡改进。

## 硬件部署

```
网络: 10→32→32→2, 1474 参数
Flash: 5.9 KB, RAM: 256 B, MAC: 1408
STM32F103 @72MHz: ~1ms 推理（5ms 中断安全）
```

```bash
uv run python export_to_c.py -m best_model_reg/best_model.zip -o balance_nn.h
```

STM32 集成：`control.c` 中 `RL_Controller()` 调用 `nn_predict(state, action)`，下游 PI 速度环不动。

## 使用

```bash
uv sync                                          # 安装依赖
uv run python test_env.py                        # 环境测试
uv run python pretrain_bc.py                     # BC 预训练（~30s）
uv run python train_ppo_reg.py                   # PPO 训练（~180s）
uv run python visualize.py -m ppo --slalom       # 3D 蛇形动画
uv run python export_to_c.py -o balance_nn.h     # 导出 C 头文件
```

## 环境 API

```python
import gymnasium as gym
import balancing_robot

env = gym.make("BalancingRobot-v0")
obs, _ = env.reset()            # obs[0:8]=状态, obs[8]=v_cmd, obs[9]=ω_cmd
obs, reward, terminated, truncated, info = env.step(action)
```
