# Self-Balancing Robot RL

Two-wheeled inverted-pendulum robot (WHEELTEC B585) controlled by a reinforcement-learned neural network. Trained entirely in simulation on a data-driven plant model fitted from real hardware telemetry, then deployed to STM32F103RCT6.

**Status: balanced on real hardware.**

## System

**8D observation** (position errors + physical states):

```
obs = [θ_L-target_L, θ_R-target_R, θ₁, θ₂, θ̇_L, θ̇_R, θ̇₁, θ̇₂]
```

**2D continuous action**: `[u_L, u_R]` — wheel angular acceleration (rad/s², ±5000), passed through the firmware's PI velocity loop.

**10ms discrete timestep** matching the STM32 LQR control period. Physical parameters from MATLAB simulation in `ref/`.

## Project structure

```
.
├── balancing_robot/
│   ├── dynamics.py         # Lagrangian physics → G, H (ZOH discrete)
│   ├── env.py              # Gymnasium env (ideal / pi_motor / data_driven)
│   └── __init__.py
├── fit_plant.py            # System ID: fit A_plant, B_plant from real data
├── pretrain_bc.py           # LQR demo collection → BC (K-initialized MLP)
├── kl_ppo.py                # KL-regularized PPO subclass
├── train_ppo_reg.py         # KL-PPO training entry point
├── visualize.py             # 3D matplotlob simulation GIF
├── export_to_c.py           # PyTorch weights → C header for STM32
├── read_bin.py              # USB binary telemetry receiver (100 Hz)
├── read_bt.py               # WiFi/Bluetooth ASCII receiver
├── test_env.py              # Environment verification tests
├── data/                    # Real hardware telemetry + fitted model
│   ├── realcar/*.csv        # 6 sessions, 36K samples each
│   ├── logs/*.csv           # Collected binary logs
│   └── real_data_model.npz  # A_plant, B_plant, noise_cov
├── models/                  # Trained checkpoints
│   ├── bc_model.pt          # K-initialized MLP (≈ LQR)
│   ├── ppo_balance_bot_kl.zip
│   └── best_model_reg/
├── outputs/
│   ├── gifs/                # 3D animation GIFs
│   └── plots/               # Diagnostic plots
├── WHEELTEC_HAL/            # STM32 Keil MDK firmware
├── docs/                    # DEVLOG.md, TECHNICAL_REPORT.md
└── pyproject.toml
```

## Method

### 1. Data-driven plant model

Real hardware telemetry from 6 CSV sessions is used to fit an open-loop plant:

```
x[k+1] = A_plant · x[k] + B_plant · u[k] + noise
```

Regularized least squares (`fit_plant.py`). Captures PI velocity loop, motor response, friction, and sensor dynamics implicitly. The fitted `B_plant` is ~1/7 of the ideal `H` matrix — the PI motor chain significantly attenuates control authority.

### 2. BC pre-training

LQR controller collects demonstrations on the data-driven plant. The BC model (MLP 8→16→2) is **initialized directly from the LQR gain matrix K** rather than trained from scratch:

```
W1 = [I₈; -I₈]    # split positive/negative pathways
W2 = [-K; K]       # reconstruct -K·obs through ReLU
```

BC output matches `u = -K·obs` exactly. No gradient-based BC training needed.

### 3. KL-regularized PPO fine-tuning

BC weights are inflated into a larger PPO network (8→32→32→2). KL penalty `bc_coef · ||μ_ppo - μ_bc||²` prevents catastrophic forgetting. Conservative hyperparameters (lr=5e-5, clip=0.05, n_epochs=3) protect the LQR baseline.

PPO preserves LQR performance (94% survival) — the LQR policy is already optimal for this linear plant.

## Results

| Controller | Mean ep_len | Survival/100 |
|------------|:----------:|:------------:|
| LQR | 1881 | 94 |
| BC (K-init) | 1881 | 94 |
| PPO | 1881 | 94 |

All three behave identically on the data-driven plant. The PPO policy was deployed to real hardware and **balances successfully**.

## Hardware deployment

```
Network: 8→32→32→2, 1410 parameters
Flash: ~20 KB, RAM: ~256 B, MAC: 1379
STM32F103 @72MHz: ~1-2ms inference (safe within 10ms control period)
```

```bash
uv run python export_to_c.py -m models/ppo_balance_bot_kl.zip -o balance_nn.h
# Copy balance_nn.h to WHEELTEC_HAL/MiniBalance/Inc/
```

The firmware's `RL_Controller()` calls `nn_predict(state, action)`, then the same PI velocity loop as LQR handles motor control.

## Quick start

```bash
uv sync
uv run python test_env.py                           # verify env
uv run python fit_plant.py                          # fit plant from real data
uv run python pretrain_bc.py                        # K-init BC (~30s)
uv run python train_ppo_reg.py                      # KL-PPO fine-tune (~5min)
uv run python visualize.py -m ppo --data-driven     # 3D GIF
uv run python export_to_c.py -m models/ppo_balance_bot_kl.zip -o balance_nn.h

# Data collection
uv run python read_bin.py -s /dev/ttyACM0 -t 30
uv run python read_bt.py --tcp 192.168.4.1 -t 30
```
