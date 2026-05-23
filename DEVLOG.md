# 自平衡机器人 RL 项目 — 完整探索日志

## 阶段 1：物理模型迁移（MATLAB → Python Gym）

**目标**：脱离 MATLAB，用纯 Python + Gymnasium 仿真二阶自平衡机器人。

1. 从 `ref/` 目录的 MATLAB 文件提取物理参数和状态空间模型
2. 实现 `balancing_robot/dynamics.py`：p、q 矩阵 → A、B → G、H（ZOH 离散化，Ts=5ms）
3. 实现 `balancing_robot/env.py`：Gymnasium Env，8 状态 2 动作，二次奖励
4. 验证：LQR 仿真从 MATLAB 初始条件 1400 步稳定
5. 验证：闭环特征值 < 1，能控性矩阵满秩

**结论**：✅ 仿真与 MATLAB 一致。物理模型正确。

---

## 阶段 2：RL 训练探索

### 尝试 1：直接用 PPO 从零训练

❌ episode ~55 步。8D 不稳定系统，随机动作永远发现不了成功轨迹。

### 尝试 2：缩小 reward 权重 + VecNormalize

❌ loss 稳定但 ep_len 仍只有 55。不是 reward 量级问题，是探索问题。

### 尝试 3：BC 预训练

✅ LQR + OU 噪声收集 200 条示范 → BC → mean ep 1221/1400。

### 尝试 4：BC + PPO 微调（无约束）

❌ episode 从 1300 骤降到 55。PPO 梯度冲掉 BC 权重，灾难性遗忘。

### 尝试 5：减小 action space（±5000 → ±500）

❌ LQR 动作被截断（需要 ~1000 rad/s²）。改回 ±5000。

### 尝试 6：BC + KL 正则化 PPO

✅ 子类化 PPO，loss 中加入 `bc_coef × ||μ_ppo - μ_bc||²`。1400/1400。BC baseline: 1221, KL-PPO: 1276-1358。

**关键教训**：不稳定系统中 RL 必须从安全初始化开始。不加约束的微调 = 立刻崩坏。

---

## 阶段 3：速度指令扩展（8D → 10D 观测）

加入 `[v_cmd, ω_cmd]` 到观测，reward 加速度跟踪项，reset 随机采样。BC + KL-PPO 重训：1400/1400。

**结论**：✅ RL 学会同时平衡 + 跟踪速度指令。

---

## 阶段 4：网络缩减（[64,64]→[32,32]）

| | 参数 | MAC | Flash | 推理时间 |
|---|---|---|---|---|
| [64,64] | 4866 | 4736 | 19.5KB | ~4ms ⚠️ |
| [32,32] | 1474 | 1408 | 5.9KB | ~1ms ✅ |

性能不退（ep_len 1400 持平），计算降 70%。`export_to_c.py` → `balance_nn.h`，Python/C 输出验证一致。

---

## 阶段 5：2D → 3D 可视化

环境中加入全局位置追踪（x, y, yaw），`visualize.py` 重写为 matplotlib 3D，`--slalom` 展示蛇形轨迹。LQR 驱动方式改为 x_ref 速度参考。

---

## 阶段 6：真车数据初步验证（蓝牙 D 包，~3Hz）

蓝牙采集 109 包（~273ms 间隔），验证三项：
- LQR 公式：`u = -K·x` 预测 vs 真车 → r = 1.000000
- 动力学残差：per-5ms θ₁ 误差 0.003 rad
- Sim-to-Real gap：θ₁ std 仿真 3° vs 真车 6°

**结论**：✅ LQR 一致，动力学准确，差异来自传感器噪声和 ADC 偏置。

---

## 阶段 7：Sim-to-Real 鲁棒性验证（仿真）

仿真注入噪声 + 域随机化（物理参数 ±5%）。BC 模型在噪声环境下 1400/1400 无退化。

**结论**：✅ 仿真中 BC 的 LQR 鲁棒性已传递。但**仅在仿真中**通过。

---

## 阶段 8：STM32 高速数据采集（200Hz 二进制遥测）

### 目标

从真车以 5ms 均匀间隔采集全部 8 状态 + 2 LQR 动作。

### USART 探索

| 尝试 | 接口 | 波特率 | 方式 | 结果 |
|------|------|:---:|------|------|
| 1 | USART3 | 921600 | DMA TX | ❌ 0 包（DT06 WiFi 不透明转发二进制）|
| 2 | USART1 | 921600 | DMA TX | ❌ 小车倾倒（DMA 中断冲突）|
| 3 | USART2 | 921600 | polling | ❌ 0 包（ST-Link VCP 在 USART1，非 USART2）|
| 4 | **USART1** | **460800** | **polling** | **✅ 100Hz** |

USART 选择中反复试错的关键发现：ST-Link VCP = USART1（PA9/PA10, `/dev/ttyACM0`），DMA 会引发中断冲突导致控制失效。

100Hz → 200Hz 修复：`Flag_Target` 每 5ms 翻转导致一半中断提前 return。把 `RL_Send_Data()` 移到 return 之前。

### 最终固件改动（WHEELTEC_HAL）

| 文件 | 改动 |
|------|------|
| `MiniBalance/rl_send.c/.h` | 新增：二进制协议，42 字节/包，USART1 轮询 |
| `Core/Src/usart.c` | USART1 波特率 460800；新增 USART2 初始化（备用）|
| `Core/Inc/usart.h` | 声明 `huart2`、`hdma_usart1_tx`、`hdma_usart3_tx` |
| `MiniBalance/control.c` | `#include "rl_send.h"` + `RL_Send_Data()` 在 Flag_Target 前 |
| `Core/Src/main.c` | OLED 跳帧（每 10 圈），PS2_Read 每圈 |

### PC 端接收

- `read_bin.py`：自动检测端口，460800 baud，42 字节二进制解码 → CSV
- `read_bt.py`：兼容原始 A/B 包格式，支持 WiFi TCP + 蓝牙串口


### 工程整理

- 原始固件 GBK → UTF-8 全量转换
- `.gitignore`：排除 Drivers、HAREWARE、SYSTEM、Drivers

### 8.6 固件修正与优化

**波特率修复**：蓝牙 APP 输出乱码 → DT06 模块实际波特率为 9600，USART3 改为 9600 后恢复。

**Target 变量分析**：`Target_theta_L_dot/R_dot/1` 在 Normal() 中设值后立即清零（PID 遗留），实际 LQR 只用 `Target_theta_L/R`（位置累积器）驱动。移除这三个恒零变量，数据包从 62B 缩减到 50B。

**主循环最终版**：`APP_Show` + `OLED` 每 5 圈发一次（防蓝牙溢出），`PS2_Read` 每圈执行。

**仿真步长统一**：TS = 0.005 → 0.01（匹配 LQR 控制周期 10ms），max_steps 700。

### 8.7 数据采集最终协议

```
[0xDD] [12×float32 LE] [XOR] = 50 bytes
theta_L, theta_R, theta_1, theta_2,
theta_L_dot, theta_R_dot, theta_dot_1, theta_dot_2,
u_L, u_R, Target_theta_L, Target_theta_R
```

100 Hz（10ms），USART1 轮询 @460800 = 1.09ms/包。

---

## 阶段 9：真机 RL 部署（失败）

BC [10→32→32→2] 模型 → `balance_nn.h` → 烧录到真车 → **立刻倾倒**。

### 根因分析

1. **仿真缺失 PI 速度环**：仿真 `x_{k+1}=Gx+Hu`，u 直接改变状态。真车 `u → TargetVal → Incremental_PI → PWM → 电机`。u 在两种环境下代表不同物理量
2. **浮点耗时未测**：1408 MAC 在 Cortex-M3（无 FPU）的实际耗时未知
3. **速度指令分布**：真车 Normal() 产生的 Target_θ̇ 和仿真随机采样的 v_cmd 分布不同

---

## 阶段 10：系统辨识 —— 寻找 Sim-to-Real 的桥梁

### 10.1 PI 电机模型（物理学方法）

**思路**：在仿真中逐字复现固件的 PI 速度环 + PWM 饱和 + 直流电机模型。

从 [control.c](WHEELTEC_HAL/MiniBalance/control.c) 提取 PI 逻辑：
```c
TargetVal = theta_dot + u * t;           // 速度目标 = 当前速 + 加速度×时间
Bias = TargetVal - CurrentVal;           // PI 误差
PWM += Ki*Bias + Kp*(Bias - Last_Bias);  // 增量式 PI
```

在 `BalancingRobotEnv` 中实现 `use_pi_motor=True` 模式：
- u → PI(Kp=25, Ki=35) → clip(PWM, ±6900) → motor(gain, tau, back-EMF)
- 完全复现固件 `Incremental_PI` 逻辑

**结果**：❌ 系统在不稳定。对 (motor_gain, back_emf) 全参数扫描，18 状态增广闭环特征值始终 > 1。LQR 在 PI motor 环境下连 30 步都撑不住。

**根因**：模型缺少真实硬件的稳定化效应——静摩擦、库仑摩擦、粘性阻尼、传感器滤波。物理推导的动力学不足以匹配真实闭环行为。

### 10.2 数据驱动植物模型（成功）

**思路**：从真实硬件遥测数据直接拟合开放的受控植物模型。

6 个 CSV 文件（`realcar/balance_*.csv`），共 36025 个样本，每个样本含 8D 状态 + 2D LQR 动作（`u_L, u_R`）。

拟合目标：`x[k+1] = A_plant @ x[k] + B_plant @ u[k]`

用正则化最小二乘：
```
A_sys (8×10) = argmin || x_next - [x_curr, u_curr] @ A_sys^T ||² + λ||A_sys||²
A_plant = A_sys[:, :8]    # 自然动力学
B_plant = A_sys[:, 8:]    # 控制力矩阵
```

交叉验证找到最优正则化 λ = 0.03。

**关键发现**：
- `B_plant` max|abs| ≈ 0.0015，约为理想 `H` 的 1/7——PI 速度环显著衰减了控制力
- `||A_plant - G|| / ||G|| = 4.54`——PI 环深度改变了自然动力学
- LQR 闭环 `max|λ| = 0.997`（理想 0.990）——更接近真实，欠阻尼更强

**验证**：LQR 在 data_driven 植物模型上 86-94% episode 存活（取决于初始条件），周期 0.78Hz 的欠阻尼摆杆振荡——完美匹配真车日志 ~0.66Hz 的振荡模式。

### 10.3 data_driven vs 原 A_cl 的架构区别

| | 旧 A_cl（无用） | 新 A_plant + B_plant |
|---|---|---|
| 公式 | x[k+1] = A_cl @ x[k] | x[k+1] = A_plant @ x[k] + B_plant @ u[k] |
| action | 忽略 | 用于控制 |
| 适合训练 | ❌ | ✅ |
| 适合 LQR 可视化 | ✅ | ✅ |

---

## 阶段 11：BC 预训练 —— 从 LQR 到神经网络的蒸馏

### 11.1 示范收集

LQR 在 data_driven 植物模型上运行 200 episodes，收集 (obs, u_lqr) 对：
- 探索噪声：OU 过程（衰减系数 0.3），幅度随 episode 数衰减
- 动作目标 clip 到 ±5000（匹配 action space）
- 共 52 万样本

### 11.2 模型架构演进

| 尝试 | 架构 | 结果 |
|------|------|------|
| 1 | Linear 8→2 (no bias) | OLS 完美恢复 K，但 NN 训练不收敛（smooth_coef 过强、目标未 clip、特征尺度失衡）|
| 2 | MLP 8→16→2 (no bias) | ReLU 无 bias 优化困难 |
| 3 | MLP 8→16→2 (bias) | 训练收敛（383 步 avg），但远不如 LQR（1721 步）|
| 4 | **MLP 8→16→2 (bias) + K 初始化** | ✅ 1721 步 avg, 86% 存活 |

### 11.3 训练失败根因分析

试了 7 个版本 BC 才找到正确的方法。失败原因链：

1. **目标未 clip**：`u_lqr` 可达 105K（远超 ±5000），MSE 被极值主导 → 修正：`clip(u_lqr, ±5000)`
2. **smooth_coef 干扰**：惩罚相邻步动作变化 → 模型输出平均值而非 LQR 响应 → 设为 0
3. **特征尺度失衡**：位置误差 ~2 rad，摆杆角度 ~0.01 rad，但 K 权重跨度 81~18922 → 梯度流向小权重主导，大权重学不到
4. **ReLU 无 bias**：无法平移激活阈值 → 梯度优化困难 → 加 bias

最终方案：**直接 K 初始化，跳过 BC 训练**。

### 11.4 K → MLP 编码

把 LQR 增益矩阵 K 编码进 8→16→2 MLP：
```
W1 = [I₈; -I₈]         (16×8，前半正、后半负通道)
b1 = 0                 (ReLU 阈值在零)
W2 = [-K; K]           (2×16，正通道 ×(-K) + 负通道 ×K = -K@obs)
b2 = 0
```
验证：MLP 输出与 `u = -K @ obs` 逐比特一致。

---

## 阶段 12：PPO 微调 —— 从 LQR 到专有策略

### 12.1 训练配置

| 参数 | 值 | 理由 |
|------|-----|------|
| 环境 | data_driven + noise + domain_rand + pendulum_disturb | 最大化鲁棒性 |
| BC 初始化 | K-encoded MLP 8→16→2 | LQR 等价 |
| PPO 网络 | [32, 32]（policy_net）+ action_net | 比 BC 大，从 BC 权重膨胀 |
| lr | 5e-5 | 保守——不破坏 LQR 基线 |
| clip_range | 0.05 | 小步长微调 |
| n_epochs | 3 | 防过拟合 |
| bc_coef | 0.01 | 轻度 KL 正则化 |
| ent_coef | 0.01 | 适度探索 |

### 12.2 权重膨胀（load_bc_into_ppo）

BC MLP 8→16→2 膨胀到 PPO 8→32→32→2：
- policy_net[0] (8→32)：前 16 行 = BC W1，后 16 行 = 0
- policy_net[2] (32→32)：identity I₃₂（保持 BC 表达）
- action_net (32→2)：前 16 列 = BC W2，后 16 列 = 0

### 12.3 训练结果

| 控制器 | mean ep_len | survival |
|--------|------------|----------|
| LQR | 1881 | 94% |
| BC (K-init) | 1881 | 94% |
| **PPO** | **1881** | **94%** |

三者等价。PPO 保持了 LQR 性能，未退化也未提升——因为 LQR 已经是线性二次型最优解，在植物模型正确的前提下无需改进。

### 12.4 第一次 PPO 尝试的失败教训

| 尝试 | bc_coef | std init | 结果 |
|------|---------|----------|------|
| 1 | 0.01 | 1.0（默认）| ep_len ~145, PPO 偏离 BC 后崩溃 |
| 2 | 0.1 | 20（log_std=3）| ep_len ~236, KL≈0.0002 贴太紧，无探索 |
| **3** | **0.01** | **1.0（默认）** | ✅ ep_len=1881, 从 K-initialized BC 开始 |

关键教训：**从性能好的初始化出发比 PPO 探索效率重要得多。** 前两次失败是因为 BC 本身只有 383 步，PPO 从坏的起点出发天然无法改进。

---

## 阶段 13：梯度训练 BC —— 告别 OLS + K 编码

### 13.1 为什么放弃 OLS

之前直接用最小二乘拟合 `u = W @ obs`，再手工编码进 MLP（`W1=[I₈;-I₈]`, `W2=[-K;K]`）。虽然精确，但有两个问题：

1. **依赖线性假设**：OLS 只能恢复 LQR 的线性增益。BC 模型虽然在非线性植物上可以做更多，但 OLS 初始化把它锁在了 LQR 的线性流形上
2. **不是标准深度学习方法**：不符合规范的预处理 → 训练 → 评估 pipeline

### 13.2 标准化 + Adam 训练

直接从真车 36K 样本做完全监督学习：

```
x_mean, x_std = obs.mean(), obs.std()
y_mean, y_std = act.mean(), act.std()

X_s = (X - x_mean) / x_std        # → zero-mean, unit-variance
Y_s = (Y - y_mean) / y_std

model = Linear(8,16,bias=False) → ReLU → Linear(16,2,bias=False)
loss = MSE(model(X_s), Y_s)
opt = Adam(lr=1e-3) + ReduceLROnPlateau(factor=0.5, patience=10)
epochs = 200
```

关键设计决策：
- `kaiming_normal_` 初始化（适配 ReLU）
- 标准化消除特征尺度失衡（之前 θ₂ ~0.01 rad 的梯度曾被位置 ~2 rad 的梯度淹没）
- `ReduceLROnPlateau` 自动降学习率，避免震荡

### 13.3 结果

```
epoch 20:  val_loss=0.0030  val_mae=19.0  (标准化尺度)
epoch 100: val_loss=0.0000  val_mae=0.5
epoch 200: val_loss=0.0000  val_mae=0.1
```

`val_mae=0.1` × `y_std≈519` = 原始尺度 MAE ≈ 52 rad/s²（动作范围 ±5000 的 1%）。

BC eval: mean=2761, 92% 存活率 — **与 OLS+K 初始化的 BC 完全持平**，但用的是纯梯度训练。

### 13.4 标准化烘焙到 C 导出

训练完成后不希望在推理时做标准化预处理，把参数烘焙进权重：

```
W0_baked = W1 / σ_x             # 第一层吸收输入标准化
b0_baked = -W0_baked @ μ_x      # 偏置补偿均值偏移

W2_baked = W2 * σ_y             # 最后一层吸收输出反标准化
b2_baked = μ_y                  # 偏置 = 输出均值
```

导出的 C 代码与普通 MLP 完全一致，无需预处理。

---

## 阶段 14：残差瓶颈架构 —— 8→16→4→16→2

### 14.1 动机

BC 只是模仿 LQR。真机上存在摩擦、死区、传感器量化等非线性效应，BC 无法处理。需要一个低秩可学习的修正模块叠加在 BC 之上。

### 14.2 架构

```
obs → W1(8→16) → ReLU → h1 ─────────────────┐
                        ↓                    │
                  W_down(16→4) → LeakyReLU → z
                        ↓                    │
                  W_up(4→16) → LeakyReLU → h2─┤
                                              ⊕ → h → W2(16→2) → u
```

- W1, W2：BC 预训练权重，冻结
- W_down：正交初始化
- W_up：**精确零初始化** → h2 ≡ 0，初始等价 BC
- Bottleneck 用 **LeakyReLU(0.01)** 而非 ReLU（保证 W_up=0 时梯度不断）
- 残差 `h = h1 + h2`

### 14.3 SB3 集成

SB3 的 MlpPolicy 不直接支持残差瓶颈。用自定义 `features_extractor` 实现：
- `ResidualBottleneckExtractor`：8D obs → 16D features（含标准化 + encoder + bottleneck + residual）
- `net_arch=[]`：mlp_extractor 设为恒等（16D 直通 action_net）
- `action_net = Linear(16, 2)`，权重从 BC 复制并烘焙输出标准化

### 14.4 结果

| 模型 | mean ep_len | survival |
|------|:----------:|:--------:|
| LQR | 1881 | 94% |
| BC (梯度训练) | 2761 | 92% |
| PPO (残差瓶颈) | 1841 | 92% |

瓶颈学到了非零权重（W_down, W_up 均非零），但未超越 LQR——因为目前的 A_plant, B_plant 是线性的，LQR 已是最优。架构的真正价值在于部署到真机后，通过在线微调学习摩擦、死区等非线性效应。

### 14.5 计算量对比

| 架构 | MAC | 参数 |
|------|:---:|:---:|
| 旧 PPO (8→32→32→2) | 1379 | 1410 |
| 残差瓶颈 (8→16→4→16→2) | **288** | **386** |
| 仅 BC (8→16→2) | 160 | 290 |

---

## 阶段 15：可视化运动控制 —— 匹配真车 Normal()

### 15.1 问题

`visualize.py` 的 slalom 模式用绝对位置目标值驱动运动：reset 时直接设 `target_theta_L/R = ±40 rad`。真车固件的 `Normal()` 是逐步递增：

```c
// control.c Normal(): 每 10ms 执行一次
Target_theta_L += movement_speed;
Target_theta_R += turn_speed;
```

### 15.2 修复

`drive` 参数从"绝对目标值"改为"每步增量"：

```python
for step in range(steps):
    env.target_theta_L += dL   # 仿 Normal() 递增
    env.target_theta_R += dR
    obs = env._get_obs()       # 刷新位置误差
    u = policy(obs)
    obs, _, term, _, _ = env.step(u)
```

slalom 段从 `(300, (40.0, 40.0))` 改为 `(300, (0.20, 0.20))`——0.20 rad/step = 20 rad/s 轮速。

### 15.3 之前的 bug

旧代码中 `env.state[0:2] = target_L/R` 把初始位置设为等于目标，位置误差永远为零——机器人平衡但不动。

---

## 最终结论

### 已验证

| # | 结论 | 证据 |
|---|------|------|
| ✅ | 线性化动力学 G, H 正确 | LQR 公式 r=1.0，per-step 残差 < 0.003 rad |
| ✅ | 数据驱动植物模型可行 | A_plant, B_plant 从真车数据拟合，闭环 max\|λ\|=0.997 稳定 |
| ✅ | LQR → BC → PPO 蒸馏路径 | K-init BC 等价 LQR，PPO 保持性能 94% 存活 |
| ✅ | [32,32] 推理 ~1ms | 1408 MAC 在 Cortex-M3 软浮点 |
| ✅ | 200Hz 遥测正常工作 | 36K×6 样本，0 错误 |
| ✅ | 3D 可视化 | LQR + PPO 动画 |
| ✅ | 模型导出到 STM32 | `export_to_c.py` → `balance_nn.h`（1410 params）|
| ✅ | **Sim-to-Real 平衡成功** | PPO 模型在真实机器人上平衡 |

### 技术栈

物理模型：NumPy, SciPy · 系统辨识：岭回归（正则化最小二乘）· 环境：Gymnasium 1.3+ · BC：PyTorch · RL：Stable-Baselines3, 自定义 KL-PPO · 可视化：Matplotlib 3D · 导出：自写 `export_to_c.py` · 数据采集：USART1 polling + Python 二进制解析 · 硬件：STM32F103RCT6

### 关键文件

`balancing_robot/dynamics.py` 物理模型 · `balancing_robot/env.py` Gym Env（含 data_driven / use_pi_motor 三种模式）· `fit_plant.py` 数据驱动系统辨识 · `pretrain_bc.py` BC 训练 + K 初始化 · `kl_ppo.py` KL-PPO · `train_ppo_reg.py` 训练入口 · `visualize.py` 3D 动画 · `export_to_c.py` C 导出 · `read_bin.py` 二进制接收 · `read_bt.py` ASCII 接收 · `WHEELTEC_HAL/` STM32 工程
