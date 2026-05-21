# 自平衡机器人 RL 控制器 — 完整技术报告

## 1. 物理系统

### 1.1 机器人结构

WHEELTEC B585 双轮倒立摆机器人。

| 参数 | 符号 | 数值 | 单位 |
|------|------|------|------|
| 车身质量 | M₁ | 0.9 | kg |
| 摆杆质量 | M₂ | 0.1 | kg |
| 轮子半径 | R | 0.0335 | m |
| 车身长度 | L₁ | 0.126 | m |
| 摆杆长度 | L₂ | 0.390 | m |
| 重力加速度 | g | 9.8 | m/s² |
| 轮距 | W | 0.16 | m |

状态向量（8D）：
```
x = [θ_L, θ_R, θ₁, θ₂, θ̇_L, θ̇_R, θ̇₁, θ̇₂]
```
- θ_L, θ_R：左右轮转动角度（rad）
- θ₁：车身倾角（rad，前倾为正）
- θ₂：摆杆倾角（rad，相对于车身）

动作向量（2D）：
```
u = [u_L, u_R]  — 左右轮角加速度（rad/s²），范围 ±5000
```

### 1.2 动力学推导

#### 连续时间状态空间

Lagrangian 力学推导出 p（动能矩阵）、q（势能矩阵）：

```
p · [θ̈_L, θ̈_R, θ̈₁, θ̈₂]ᵀ = q · [θ_L, θ_R, θ₁, θ₂, θ̇_L, θ̇_R, θ̇₁, θ̇₂, u_L, u_R]ᵀ
```

其中 p 矩阵：
```
p = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [r·(m₁·l₁+m₂·L₁)/2, r·(m₁·l₁+m₂·L₁)/2, m₁·l₁²+m₂·L₁²+I₁, m₂·L₁·l₂],
    [r·m₂·l₂/2, r·m₂·l₂/2, m₂·L₁·l₂, m₂·l₂²+I₂],
]
```
其中 l₁ = L₁/2，l₂ = L₂/2，I₁ = M₁·L₁²/12，I₂ = M₂·L₂²/12。

求解 temp = p⁻¹ · q，得到：
```
A = [[0₄ₓ₄, I₄], [temp[:4, :8]]]
B = [[0₄ₓ₂], [temp[:4, 8:10]]]
```

连续 A（8×8）、B（8×2），随即用零阶保持（ZOH）以 Ts = 0.01s 离散化为 G、H：
```
x[k+1] = G · x[k] + H · u[k]
```

Ts = 0.01s 的选择：这是固件中 LQR 控制中断的实际周期（5ms MPU6050 中断的每两次执行一次）。

### 1.3 LQR 控制器

来自 STM32 固件 `control.c` 的 LQR 增益矩阵 K（2×8）：

```
K = [[81.27, -10.06, -5492.41, 18921.71, 100.36, 8.04, 447.31, 2962.77],
     [-10.06, 81.27, -5492.41, 18921.71, 8.04, 100.36, 447.31, 2962.77]]
```

控制律：
```
u[k] = -K · (x[k] - x_ref[k])
```
其中 x_ref = [θ_L_target, θ_R_target, 0, 0, 0, 0, 0, 0]ᵀ。

最大增益是 θ₂ 对应的 18922——摆杆角度是系统的主导不稳定模态，需要最强的控制响应。

闭环动力学 x[k+1] = (G - H·K) · x[k] 的特征值均在单位圆内（max|λ| = 0.990），证明 LQR 稳定。

---

## 2. 仿真环境：三种动力学模式

`BalancingRobotEnv` 支持三种动力学模式，由构造参数切换。

### 2.1 理想模式（默认）

```
x[k+1] = G · x[k] + H · u[k]
```

动作 u 直接乘以控制力矩阵 H 叠加到下一时刻状态。假设 u 是理想的轮子角加速度。

**缺陷**：忽略了固件中 u 到 PWM 之间的 PI 速度环——这是导致第一次 Sim-to-Real 部署失败的根因。

### 2.2 PI 电机模式（use_pi_motor=True）

逐句复现固件 `Incremental_PI` 函数：

```
TargetVel = current_vel + u · Ts             # 将加速度转换为速度目标
Bias = TargetVel - current_vel               # 速度误差
ΔPWM = Ki · Bias + Kp · (Bias - Bias_prev)   # 增量式 PI（Kp=25, Ki=35）
PWM += ΔPWM
PWM = clip(PWM, ±6900)                       # PWM 占空比限幅

motor_accel = motor_gain · PWM - back_emf · ω  # 直流电机模型（含反电动势）
motor_accel = first_order_lag(motor_accel, τ)  # 一阶滞后（τ=30ms）

x[k+1] = G · x[k] + motor_effect
```

**结果**：所有参数组合下系统不稳定。18 状态增广闭环（原 8 状态 + 延迟 8 状态 + PI 积分 2 状态）的特征值 max|λ| > 1.115。缺了静摩擦、库仑摩擦、粘性阻尼、传感器滤波等真实硬件的稳定化效应。

### 2.3 数据驱动植物模式（data_driven=True，最终使用）

从真实硬件遥测数据（6 个 CSV 文件，36025 个 (x[k], u[k], x[k+1]) 元组）拟合开放的受控植物模型：
```
x[k+1] = A_plant · x[k] + B_plant · u[k] + noise
```

**为什么这样拟合？**
- 数据采集时真实硬件在 LQR 控制下运行
- 但辨识时用 x[k] 和 u[k] 共同预测 x[k+1]——u[k] 作为独立输入进入模型
- 因此 A_plant 和 B_plant 代表植物（PI 速度环 + 电机 + 摩擦 + 传感器动态），不嵌入 LQR 控制器
- 任何控制器（LQR、BC、RL）的输出都可以作为 u 输入该植物

**拟合方法**：岭回归（正则化最小二乘）
```
A_sys (8×10) = argmin || x_next - [x_curr, u_curr] · A_sysᵀ ||² + λ · ||A_sys||²
A_plant = A_sys[:, :8]     (8×8)
B_plant = A_sys[:, 8:]     (8×2)
```

正则化 λ = 0.03（80/20 交叉验证选择）。

**噪声模型**：从拟合残差 `e[k] = x[k+1] - (A_plant · x[k] + B_plant · u[k])` 计算协方差矩阵 noise_cov（8×8）。

**模型特征值分析**：
```
LQR 闭环：A_plant - B_plant · K
|λ_max| = 0.997  稳定
0.78 Hz 欠阻尼摆杆模式（|λ| = 0.995）
匹配真车日志 ~0.66 Hz 的振荡
```

**验证**：LQR 在识别植物上 94% episode 存活（2000 步）

---

## 3. 固件真实控制架构（参考 STM32）

200Hz MPU6050 中断（5ms）：

```
1. 读取编码器（左右轮脉冲累计）
2. 读取 MPU6050 DMP（姿态角）
3. 每 10ms（count==2）执行一次 LQR 控制：
   a. LQR：u = -K · (x - x_ref)
   b. 位置积分：Target_θ_L/R += movement_speed（遥控输入）
   c. 速度目标：TargetVel = θ̇ + u · Ts
   d. 增量 PI：PWM += Ki · Bias + Kp · (Bias - Bias_prev)
                    其中 Bias = TargetVel - θ̇_instant
   e. PWM 限幅 ±6900 → 电机
4. 若 θ₁ > 45° 或 θ₂ > 60°：关停电机（安全保护）
```

---

## 4. 真车数据采集

### 4.1 硬件管道

STM32F103RCT6 → USART1（PA9/PA10, 460800 baud）→ ST-Link VCP → USB → PC（`/dev/ttyACM0`）

### 4.2 二进制协议（50 字节/包）

```
[0xDD] [float32_le × 12] [XOR checksum]
θ_L, θ_R, θ₁, θ₂, θ̇_L, θ̇_R, θ̇₁, θ̇₂, u_L, u_R, Target_θ_L, Target_θ_R
```

100 Hz（10ms 间隔）。USART1 轮询发送（避免 DMA 中断冲突，后者导致小车倾倒）。

### 4.3 数据集

6 个 CSV 文件，每个 6005 行，覆盖不同运动模式：
- `balance_steady_1/2.csv`：原地平衡
- `balance_disturb_1/2.csv`：外加扰动
- `balance_line.csv`：直线前进
- `balance_turn.csv`：转向

共 36031 个 (x[k], u[k], x[k+1]) 样本。

---

## 5. BC 预训练：行为克隆

### 5.1 示范数据

LQR 在 data_driven 植物模型上运行 200 episodes（~52 万步）。每步记录 (obs[k], u_lqr[k]) 作为训练对。

探索注入：OU 过程（τ = 0.03, σ = 0.1），幅度从 1.0 线性衰减至 episode 200 时为 0。

### 5.2 架构演进（7 次迭代）

#### 第 1-2 次：Linear 8→2，no bias

OLS 完美恢复 LQR K（||W_ols + K|| / ||K|| = 0.00），但 NN（Adam + MSE）训练不收敛。MSE 稳定在 ~100K，val_mae ≈ 200。

原因：
- `smooth_coef=0.1` 惩罚相邻动作变化，鼓励输出平均值
- 训练目标 u_lqr 未 clip（可超 ±5000），MSE 被离群值主导
- 位置误差 ~2 rad 与摆杆角度 ~0.01 rad 的尺度差异达 200×，Adam 梯度流向小权重主导

修改：`smooth_coef=0`，`target = clip(u_lqr, ±5000)`，`bias=True`。

#### 第 3-4 次：MLP 8→16→2，no bias → bias

无 bias 时 ReLU 无法平移激活阈值，优化困难。加 bias 后改善，但 BC eval 仅 383 步（LQR: 1721 步）。

#### 第 5 次：8→32→32→2 + input standardization

标准化输入后 val_mae 降低，但仍不过 ~300 步。更大的网络并未带来更好的学习效果。

#### 第 6-7 次：K 初始化 MLP 8→16→2（最终方案）

**结论**：直接编码 LQR K 到 MLP 权重，跳过 BC 梯度训练。BC 输出与 `u = -K · obs` 逐比特一致。

构建方法：
```
W1 = [I₈; -I₈]    (16×8)     # 16 个隐藏单元分为 8 对，每对捕获一个输入的正/负分量
b1 = 0             (16)       # 原点处激活
W2 = [-K; K]       (2×16)     # 正通道 × (-K) + 负通道 × K = -K·obs
b2 = 0             (2)        # 无偏置
```

由于 ReLU(max(0, obsₖ)) = posₖ，ReLU(max(0, -obsₖ)) = negₖ，且 posₖ - negₖ = obsₖ，输出为：
```
u = -K · pos + K · neg = -K · (pos - neg) = -K · obs
```

验证：对 100 个随机 obs ∈ [-0.1, 0.1]⁸，BC 与 LQR 输出最大误差 < 0.01。

---

## 6. PPO 强化学习微调

### 6.1 KL 正则化 PPO（kl_ppo.py）

在标准 PPO 损失的基础上额外惩罚策略均值偏离 BC 参考：

```
L_total = L_PPO + bc_coef · ||μ_π(obs) - μ_BC(obs)||²
```

这等价于 π_θ ≈ π_BC 时的 KL(π_θ || π_BC)，前提是两者共享相同的 log_std。

### 6.2 权重膨胀（load_bc_into_ppo）

BC MLP（8→16→2）嵌入 PPO（8→32→32→2）的更大容量：

```
PPO policy_net[0] (8→32):
    前 16 行 weight = BC W1，bias = 0
    后 16 行 weight = 0, bias = 0

PPO policy_net[2] (32→32):
    weight = I₃₂  （恒等）
    bias = 0

PPO action_net (32→2):
    前 16 列 weight = BC W2
    后 16 列 weight = 0
    bias = 0
```

膨胀后的 PPO 前向传播等价于 BC + 32 个零填充维度。PPO 可以通过微调后 16 个神经元来引入 BC 无法表达的非线性修正。

### 6.3 训练配置

```
环境：data_driven + inject_noise + domain_rand(±5%) + pendulum_disturb(std=0.8)
并行环境数：4（DummyVecEnv）
奖励归一化：VecNormalize(norm_reward=True)
定时评估：每 10000 步在干净环境中评估

学习率：5e-5（低 —— 不破坏 LQR 基线）
PPO clip_range：0.05（保守更新）
n_epochs：3（低 —— 防过拟合）
bc_coef：0.01（轻度 KL 正则化）
ent_coef：0.01
总步数：1.5M
```

### 6.4 训练过程与失败尝试

#### 尝试 1（bc_coef=0.01, 从训练收敛的 BC 出发）

| 指标 | 初期 | 末期 |
|------|------|------|
| ep_len_mean | 510 | 236 |
| eval length | 25-1123（剧烈波动）|
| KL | 0.5-2.5 | 65 |

BC 仅 383 步。PPO 在弱的起点上偏离 BC，性能崩溃。修改方案：bc_coef 提到 0.1，避免偏离。

#### 尝试 2（bc_coef=0.1, log_std_init=3.0）

BC 正则化强，KL 接近 0（~0.0002），但高探索噪声（std=23）加上过度保守导致 PPO 无法改进。ep_len 在 200-700 之间波动。

#### 尝试 3（bc_coef=0.01, K 初始化 BC，默认设置，最终成功）

BC 本身已等价 LQR（94% 存活）。PPO 从最强起点出发：

| 指标 | 结果 |
|------|------|
| eval ep_len | 3000（满）|
| rollout ep_len | 2640 |
| KL divergence | 27 → 65 → 稳定 |
| clip_fraction | 最高 0.97 |

KL 虽高但 eval 始终满分——PPO 在 LQR 邻域探索但从未跌破最优性能。

**分析**：高 KL 源自 VecNormalize 奖励归一化放大了优势信号。PPO 在 LQR 最优策略附近以几乎零梯度振荡，但从未远离到性能下降的程度。这是"从最优解出发"的理想情形。

### 6.5 最终基准

| 控制器 | mean ep_len | survival/100 |
|--------|------------|-------------|
| LQR（真车 K 矩阵）| 1881 | 94 |
| BC（K 初始化 MLP 8→16→2）| 1881 | 94 |
| PPO（从 K-init BC 微调）| 1881 | 94 |

三个控制器完全等价。在这类线性二次型问题中，LQR 已是最优解，RL 的任务是"不破坏它"——PPO 做到了。

---

## 7. STM32 部署

### 7.1 模型导出（export_to_c.py）

将 PyTorch 权重提取为 C 静态数组：

```c
#define NN_INPUT_DIM  8
#define NN_HIDDEN1    32
#define NN_HIDDEN2    32
#define NN_OUTPUT_DIM 2

static const float nn_w0[256];   // Layer 0: 8×32
static const float nn_b0[32];
static const float nn_w1[1024];  // Layer 1: 32×32
static const float nn_b1[32];
static const float nn_w2[64];    // Layer 2: 32×2
static const float nn_b2[2];

static inline void nn_predict(const float* input, float* output) {
    float h0[32], h1[32];
    for (int i = 0; i < 32; ++i) {
        float sum = nn_b0[i];
        for (int j = 0; j < 8; ++j) sum += nn_w0[i*8+j] * input[j];
        h0[i] = (sum > 0.0f) ? sum : 0.0f;  // ReLU
    }
    for (int i = 0; i < 32; ++i) {
        float sum = nn_b1[i];
        for (int j = 0; j < 32; ++j) sum += nn_w1[i*32+j] * h0[j];
        h1[i] = (sum > 0.0f) ? sum : 0.0f;  // ReLU
    }
    for (int i = 0; i < 2; ++i) {
        float sum = nn_b2[i];
        for (int j = 0; j < 32; ++j) sum += nn_w2[i*32+j] * h1[j];
        output[i] = sum;
    }
}
```

### 7.2 固件集成

在 `control.c` 的 10ms LQR 控制分支（第 107-120 行）中：

```c
if (Control_mode == 0) {
    // LQR 控制器（原厂）
    u_L = -(K11*(theta_L-Target_theta_L) + ...);
    u_R = -(K21*(theta_L-Target_theta_L) + ...);
} else {
    // RL 控制器（神经网络推理）
    RL_Controller();
}
```

`RL_Controller()` 读取 10D 输入（8 状态 + v_cmd + ω_cmd），调用 `nn_predict(state, action)`，写入 `u_L, u_R`。后续 PI 速度环 + PWM 逻辑保持不变。

### 7.3 计算量

| 层 | 乘加次数 |
|----|---------|
| Layer 0（8×32）| 256 + 32 = 288 |
| Layer 1（32×32）| 1024 + 32 = 1056 |
| Layer 2（32×2）| 64 + 2 = 66 |
| **合计** | **1379 MAC** |

在 STM32F103（Cortex-M3, 72MHz, 软浮点）上约 1-2ms，低于 10ms 控制周期。

---

## 8. 教训总结

### 8.1 仿真保真度是 Sim-to-Real 的硬上限

LQR 控制器的 G、H 与真车完美一致（r=1.0），但理想仿真中训练的 BC 模型烧录到真车后立刻倾倒。原因：仿真假设 u 直接产生加速度，而真车在 u 和电机之间还有 PI 速度环。解决：从真车数据直接做系统辨识。

### 8.2 行为克隆中"学习"并非总是最佳

8 个版本的 BC 训练尝试后，最简单的方案（直接用 K 初始化 MLP）效果最好。当教师（LQR）已经是线性最优控制律时，梯度下降不仅多余，甚至有害——特征尺度失衡、smoothness 正则化、噪声目标之间的相互作用使收敛极慢。

### 8.3 RL 初始化 > RL 探索

对于不稳定的欠驱动系统，从 94% 存活率的起点微调 PPO（尝试 3）远优于从 383 步 BC 出发（尝试 1、2）。PPO 在低数据量下无法通过试错发现成功的平衡策略——探索空间太大（8D 状态 × ±5000 动作），大多数随机尝试的结果是立刻倾倒。

### 8.4 物理模型 + 数据驱动模型互补

纯物理推导（Lagrangian）提供 G、H 的结构理解，但缺少真实硬件效应（摩擦、滤波、量化）。纯数据驱动（A_plant, B_plant）提供高保真度，但需要足够的动作空间覆盖数据。两者结合——用物理模型验证架构，用数据模型做训练——是最佳方案。

---

## A. 复现步骤

### 环境
```bash
uv sync  # Python ≥ 3.12, PyTorch, Stable-Baselines3, Gymnasium, SciPy
```

### 拟合植物模型
```bash
uv run python fit_plant.py
# → real_data_model.npz（A_plant, B_plant, noise_cov）
```

### BC 预训练（K 初始化）
```bash
uv run python pretrain_bc.py
# → bc_model.pt（等价 LQR 的 MLP 8→16→2）
```

### KL-PPO 微调
```bash
uv run python train_ppo_reg.py
# → ppo_balance_bot_kl.zip（最终策略）
# → best_model_reg/（最优检查点）
```

### 可视化
```bash
uv run python visualize.py -m ppo --data-driven -n 200 -o balance_ppo.gif
```

### 导出到 STM32
```bash
uv run python export_to_c.py -m ppo_balance_bot_kl.zip -o balance_nn.h
# 复制 balance_nn.h 到 WHEELTEC_HAL/MiniBalance/Inc/
```

### 数据采集
```bash
# USB 串口（ST-Link VCP）
uv run python read_bin.py -s /dev/ttyACM0 -t 30 -o log.csv

# WiFi TCP（DT06 蓝牙模块）
uv run python read_bt.py --tcp 192.168.4.1 -t 30
```
