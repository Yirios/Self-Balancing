# 自平衡机器人 RL

WHEELTEC B585 双轮倒立摆机器人的强化学习神经网络控制器。全部训练在仿真中完成——仿真使用从真实硬件遥测数据拟合的数据驱动植物模型。最终导出部署到 STM32F103RCT6。

**当前状态：真机平衡成功。**

## 系统

**8 维观测**（位置误差 + 物理状态）：

```
obs = [θ_L-target_L, θ_R-target_R, θ₁, θ₂, θ̇_L, θ̇_R, θ̇₁, θ̇₂]
```

**2 维连续动作**：`[u_L, u_R]` — 轮子角加速度（rad/s²，±5000），经过固件 PI 速度环驱动电机。

**10ms 离散步长**，匹配 STM32 LQR 控制周期。物理参数来自 `ref/` 中的 MATLAB 仿真。

## 方法

### 1. 数据驱动植物模型

```
x[k+1] = A_plant · x[k] + B_plant · u[k] + noise
```

从 6 个 CSV 文件的真车遥测数据（36025 个样本），用正则化最小二乘拟合（`fit_plant.py`）。A_plant, B_plant 隐含编码了 PI 速度环、电机响应、摩擦、传感器动态。拟合得到的 B_plant 约为理想 H 矩阵的 1/7。

### 2. BC 预训练（梯度训练，非 OLS）

直接从真车数据做标准化监督学习，不用仿真采样：

```
标准化 → Linear(8,16) → ReLU → Linear(16,2) → 反标准化
```

Adam + ReduceLROnPlateau，200 epochs。标准化消除特征尺度失衡。val_mae ≈ 52 rad/s²（动作范围 ±5000 的 1%），92% 存活率。

### 3. 残差瓶颈 PPO 微调

BC 冻结，叠加低秩可学习瓶颈（16→4→16），残差连接：

```
obs → W1(8→16)→ReLU→h1 ──────────────┐
                  ↓                    │
            W_down(16→4)→LeakyReLU→z   │
                  ↓                    │
            W_up(4→16)→LeakyReLU→h2 ───⊕→h→W2(16→2)→u
```

W_up 零初始化 → 初始等价 BC。LeakyReLU(0.01) 保证梯度流通。KL 正则化防止遗忘。

| 架构 | MAC | 参数 |
|------|:---:|:---:|
| 旧 PPO (8→32→32→2) | 1379 | 1410 |
| **残差瓶颈 (8→16→4→16→2)** | **288** | **386** |
| 仅 BC (8→16→2) | 160 | 290 |

## 结果

| 控制器 | 平均 ep_len | 存活率 |
|--------|:----------:|:------:|
| LQR（真车 K 矩阵）| 1881 | 94% |
| BC（梯度训练）| 2761 | 92% |
| PPO（残差瓶颈）| 1841 | 92% |

三者均能稳定平衡。瓶颈权重学到了非零值，但在当前线性植物上 LQR 已是最优——架构的真正价值在于部署到真机后学习非线性效应。

## 项目结构

```
.
├── balancing_robot/
│   ├── dynamics.py         # 拉格朗日力学 → G, H
│   ├── env.py              # Gymnasium 环境（ideal / pi_motor / data_driven）
│   └── __init__.py
├── fit_plant.py            # 系统辨识：A_plant, B_plant 岭回归
├── pretrain_bc.py           # BC：真车数据标准化 + Adam 梯度训练
├── train_residual_ppo.py    # 残差瓶颈 KL-PPO 训练入口
├── train_ppo_reg.py         # 旧版 KL-PPO（标准 MLP，用于对比）
├── kl_ppo.py                # KL 正则化 PPO 子类
├── visualize.py             # 3D matplotlib 仿真动画
├── export_to_c.py           # PyTorch → STM32 C 头文件
├── read_bin.py / read_bt.py # 数据采集（USB / WiFi）
├── test_env.py              # 环境验证测试
├── data/                    # 真车遥测 + 拟合模型
│   ├── realcar/*.csv
│   ├── logs/*.csv
│   └── real_data_model.npz
├── models/                  # 检查点
│   ├── bc_model.pt
│   ├── ppo_residual.zip
│   └── best_model_residual/
├── outputs/                 # gifs/ + plots/
├── docs/                    # TECHNICAL_REPORT.md, DEVLOG.md 等
└── WHEELTEC_HAL/            # STM32 Keil 固件
```

## 硬件部署

```
架构: 8→16→4→16→2 残差瓶颈, 386 参数
MAC: 288, Flash: ~4 KB, RAM: ~128 B
STM32F103 @72MHz: <1ms 推理（10ms 控制周期内安全）
```

```bash
uv run python export_to_c.py -m models/ppo_residual.zip -o balance_nn.h
# balance_nn.h → WHEELTEC_HAL/MiniBalance/Inc/
```

标准化参数已在导出时烘焙进权重，C 代码无需预处理。

## 快速开始

```bash
uv sync
uv run python fit_plant.py                          # 拟合植物模型
uv run python pretrain_bc.py                        # BC 梯度预训练 (~1min)
uv run python train_residual_ppo.py                  # 残差瓶颈 PPO (~8min)
uv run python visualize.py -m ppo --data-driven \
    --model-path models/ppo_residual.zip             # 3D 动画
uv run python export_to_c.py \
    -m models/ppo_residual.zip -o balance_nn.h       # 导出 STM32

# 数据采集
uv run python read_bin.py -s /dev/ttyACM0 -t 30
uv run python read_bt.py --tcp 192.168.4.1 -t 30
```
