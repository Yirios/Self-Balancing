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

## 项目结构

```
.
├── balancing_robot/
│   ├── dynamics.py         # 拉格朗日力学 → G, H（ZOH 离散化）
│   ├── env.py              # Gymnasium 环境（ideal / pi_motor / data_driven 三种模式）
│   └── __init__.py
├── fit_plant.py            # 系统辨识：从真车数据拟合 A_plant, B_plant
├── pretrain_bc.py           # LQR 示范收集 → BC（K 初始化 MLP）
├── kl_ppo.py                # KL 正则化 PPO 子类
├── train_ppo_reg.py         # KL-PPO 训练入口
├── visualize.py             # 3D matplotlib 仿真动画
├── export_to_c.py           # PyTorch 权重 → STM32 C 头文件
├── read_bin.py              # USB 二进制遥测接收（100 Hz）
├── read_bt.py               # WiFi/蓝牙 ASCII 接收
├── test_env.py              # 环境验证测试
├── data/                    # 真车遥测 + 拟合模型
│   ├── realcar/*.csv        # 6 次采集，每次 36K 样本
│   ├── logs/*.csv           # 二进制日志
│   └── real_data_model.npz  # A_plant, B_plant, noise_cov
├── models/                  # 训练检查点
│   ├── bc_model.pt          # K 初始化 MLP（≈ LQR）
│   ├── ppo_balance_bot_kl.zip
│   └── best_model_reg/
├── outputs/
│   ├── gifs/                # 3D 动画
│   └── plots/               # 诊断图
├── WHEELTEC_HAL/            # STM32 Keil MDK 固件
├── docs/                    # 技术文档（README.en.md, TECHNICAL_REPORT.md, DEVLOG.md 等）
└── pyproject.toml
```

## 方法

### 1. 数据驱动植物模型

从 6 个 CSV 文件的真车遥测数据（36025 个样本）拟合开放受控植物：

```
x[k+1] = A_plant · x[k] + B_plant · u[k] + noise
```

正则化最小二乘（`fit_plant.py`）。A_plant, B_plant 隐含编码了 PI 速度环、电机响应、摩擦、传感器动态。拟合得到的 `B_plant` 约为理想 `H` 矩阵的 1/7——PI 电机链显著衰减了控制力。

### 2. BC 预训练

在数据驱动植物上，LQR 收集示范 (~52 万步)。BC 模型（MLP 8→16→2）**直接从 LQR 增益矩阵 K 初始化**，跳过了梯度训练：

```
W1 = [I₈; -I₈]     # 正/负通道拆分
W2 = [-K; K]        # 通过 ReLU 重构 -K·obs
```

BC 输出与 `u = -K·obs` 逐位一致。

### 3. KL 正则化 PPO 微调

BC 权重膨胀到更大的 PPO 网络（8→32→32→2）。KL 惩罚项 `bc_coef · ||μ_ppo - μ_bc||²` 防止灾难性遗忘。保守超参数（lr=5e-5, clip_range=0.05, n_epochs=3）保护 LQR 基线。

## 结果

| 控制器 | 平均 ep_len | 存活/100 |
|--------|:----------:|:--------:|
| LQR | 1881 | 94 |
| BC (K-init) | 1881 | 94 |
| PPO | 1881 | 94 |

三者表现一致。PPO 策略已部署到真车，**平衡成功**。

## 硬件部署

```
网络: 8→32→32→2, 1410 参数
Flash: ~20 KB, RAM: ~256 B, MAC: 1379
STM32F103 @72MHz: ~1-2ms 推理（10ms 控制周期内安全）
```

```bash
uv run python export_to_c.py -m models/ppo_balance_bot_kl.zip -o balance_nn.h
# 复制 balance_nn.h 到 WHEELTEC_HAL/MiniBalance/Inc/
```

固件 `RL_Controller()` 调用 `nn_predict(state, action)`，下游 PI 速度环与 LQR 共用。

## 快速开始

```bash
uv sync
uv run python test_env.py                           # 验证环境
uv run python fit_plant.py                          # 从真车数据拟合植物模型
uv run python pretrain_bc.py                        # K-init BC (~30s)
uv run python train_ppo_reg.py                      # KL-PPO 微调 (~5min)
uv run python visualize.py -m ppo --data-driven     # 3D 动画
uv run python export_to_c.py -m models/ppo_balance_bot_kl.zip -o balance_nn.h

# 数据采集
uv run python read_bin.py -s /dev/ttyACM0 -t 30
uv run python read_bt.py --tcp 192.168.4.1 -t 30
```
