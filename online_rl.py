"""
Online RL: blend MLP with LQR, trained on real car via serial.

Control:  u_total = (1-α)·u_LQR + α·u_MLP
PC sends delta = α·(u_MLP - u_LQR) to STM32, which adds it on top of LQR.

Usage:
  uv run python online_rl.py -s /dev/ttyACM0
  uv run python online_rl.py -s /dev/ttyACM0 --alpha-start 0.03 --alpha-max 0.3 --alpha-step 0.003
"""
import argparse
import struct
import time
from collections import deque

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

from balancing_robot.dynamics import get_lqr_gains

# ─── Config ───
SYNC = 0xDD
PACKET_SIZE = 66
EXT_SYNC = 0xCC
EXT_PACKET_SIZE = 9

STATE_KEYS = [
    "theta_L", "theta_R", "theta_1", "theta_2",
    "theta_L_dot", "theta_R_dot", "theta_dot_1", "theta_dot_2",
    "u_L", "u_R",
    "Target_theta_L", "Target_theta_R",
    "TargetVal_L", "TargetVal_R", "PWM_L", "PWM_R",
]

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
HIDDEN = 16
LR = 3e-4
UPDATE_INTERVAL = 1.0
BUF_CAPACITY = 2000

K = get_lqr_gains()  # LQR gain matrix, same as STM32 firmware
print(f"Device: {DEVICE}")


# ─── Residual MLP ───

class ResidualMLP(nn.Module):
    """Small MLP that learns residual correction to LQR."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(8, HIDDEN), nn.ReLU(),
            nn.Linear(HIDDEN, 2),
        )

    def forward(self, x):
        return self.net(x)


# ─── Serial I/O ───

class SerialIO:
    """Bidirectional serial communication with WHEELTEC robot."""

    def __init__(self, port, baud=460800):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.rx_buf = b""
        self.running = True
        self.pkt_count = 0

    def read_packets(self) -> list[dict]:
        """Read and parse all available binary packets."""
        pkts = []
        n = self.ser.in_waiting or 1
        self.rx_buf += self.ser.read(n)

        while len(self.rx_buf) >= PACKET_SIZE:
            idx = self.rx_buf.find(bytes([SYNC]))
            if idx < 0:
                self.rx_buf = self.rx_buf[-PACKET_SIZE:]
                break
            if len(self.rx_buf) - idx < PACKET_SIZE:
                break

            raw = self.rx_buf[idx:idx + PACKET_SIZE]
            ck = 0
            for b in raw[:-1]:
                ck ^= b
            if ck != raw[-1]:
                self.rx_buf = self.rx_buf[idx + 1:]
                continue

            floats = struct.unpack("<16f", raw[1:65])
            pkts.append(dict(zip(STATE_KEYS, floats)))
            self.pkt_count += 1
            self.rx_buf = self.rx_buf[idx + PACKET_SIZE:]

        return pkts

    def send_action(self, u_L: float, u_R: float):
        """Send u_L, u_R to STM32 (PC -> robot external control)."""
        buf = bytearray(EXT_PACKET_SIZE)
        buf[0] = EXT_SYNC
        struct.pack_into("<f", buf, 1, u_L)
        struct.pack_into("<f", buf, 5, u_R)
        self.ser.write(buf)

    def close(self):
        self.running = False
        self.ser.close()


# ─── Helpers ───

def build_obs(data: dict) -> np.ndarray:
    """8D observation from telemetry packet."""
    return np.array([
        data["theta_L"] - data["Target_theta_L"],
        data["theta_R"] - data["Target_theta_R"],
        data["theta_1"], data["theta_2"],
        data["theta_L_dot"], data["theta_R_dot"],
        data["theta_dot_1"], data["theta_dot_2"],
    ], dtype=np.float32)


def build_state(data: dict) -> np.ndarray:
    """Full 8D state vector for LQR computation."""
    return np.array([
        data["theta_L"], data["theta_R"],
        data["theta_1"], data["theta_2"],
        data["theta_L_dot"], data["theta_R_dot"],
        data["theta_dot_1"], data["theta_dot_2"],
    ])


def compute_reward(data: dict, alpha: float) -> float:
    """Quadratic cost penalizing tilt and control effort."""
    cost = (10.0 * data["theta_1"] ** 2 +
            50.0 * data["theta_2"] ** 2 +
            1e-5 * (data["u_L"] ** 2 + data["u_R"] ** 2))
    return -cost


# ─── Episode ───

def run_episode(io: SerialIO, model: ResidualMLP, alpha: float,
                max_steps: int = 3000, verbose: bool = True,
                perturb_steps: int = 0) -> list:
    """Run one episode with (1-α)*LQR + α*MLP blend.

    perturb_steps: if > 0, send asymmetric pulse for first N steps
                   to disturb the car into oscillation before RL takes over.
    """
    buffer = []
    last_obs = None
    last_action = None

    for step in range(max_steps):
        pkts = io.read_packets()
        if not pkts:
            time.sleep(0.001)
            continue

        data = pkts[-1]

        obs = build_obs(data)
        x = build_state(data)
        x_ref = np.array([data["Target_theta_L"], data["Target_theta_R"],
                          0, 0, 0, 0, 0, 0])

        # ── Control: (1-α)·LQR + α·MLP ──
        u_lqr = -K @ (x - x_ref)
        with torch.no_grad():
            inp = torch.from_numpy(obs).float().to(DEVICE)
            u_mlp = model(inp).cpu().numpy()
        u_blend = (1 - alpha) * u_lqr + alpha * u_mlp

        # PC sends delta = α·(u_mlp - u_lqr), STM32 adds to its own LQR
        delta = alpha * (u_mlp - u_lqr)

        # Perturbation: asymmetric pulse to excite oscillation
        if step < perturb_steps:
            delta += np.array([800.0, -800.0])  # strong turn disturbance

        io.send_action(delta[0], delta[1])

        if verbose and step < 10:
            th1 = np.rad2deg(data["theta_1"])
            print(f"  step {step}: th1={th1:+.2f}deg "
                  f"STM32_u=({data['u_L']:.0f},{data['u_R']:.0f}) "
                  f"PC_delta=({delta[0]:.0f},{delta[1]:.0f}) "
                  f"u_blend=({u_blend[0]:.0f},{u_blend[1]:.0f})")

        reward = compute_reward(data, alpha)
        terminated = bool(abs(data["theta_1"]) > 0.7854 or
                          abs(data["theta_2"]) > 0.7854)

        if last_obs is not None:
            buffer.append((last_obs, last_action, reward, obs, terminated))

        last_obs = obs
        last_action = u_mlp - u_lqr  # store the residual, not the delta

        if terminated:
            print(f"  Episode terminated at step {step} "
                  f"(th1={np.rad2deg(data['theta_1']):.1f}deg)")
            break

    return buffer


# ─── Main ───

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Online RL on real WHEELTEC robot")
    parser.add_argument("-s", "--serial", type=str, default=None)
    parser.add_argument("-b", "--baud", type=int, default=460800)
    parser.add_argument("--alpha-start", type=float, default=0.03,
                        help="Initial alpha for LQR/MLP blend")
    parser.add_argument("--alpha-max", type=float, default=0.3,
                        help="Maximum alpha")
    parser.add_argument("--alpha-step", type=float, default=0.003,
                        help="Alpha increment per update")
    parser.add_argument("--batch-size", type=int, default=128,
                        help="Samples per training update")
    parser.add_argument("--lr", type=float, default=3e-4,
                        help="Learning rate")
    parser.add_argument("--perturb", type=int, default=8,
                        help="Perturbation steps at start of each episode (0=off)")
    parser.add_argument("--episode-delay", type=float, default=2.0,
                        help="Delay between episodes (seconds)")
    args = parser.parse_args()

    port = args.serial
    if port is None:
        import glob
        candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        if not candidates:
            print("No serial port found. Use -s /dev/ttyACM0")
            exit(1)
        port = candidates[0]
        print(f"Auto-detected: {port}")

    alpha = args.alpha_start

    print(f"Connecting to {port} @ {args.baud}...")
    io = SerialIO(port, args.baud)
    time.sleep(0.5)

    model = ResidualMLP().to(DEVICE)
    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    replay = deque(maxlen=BUF_CAPACITY)

    print(f"Online RL started.")
    print(f"  Control:  u = (1-α)·LQR + α·MLP")
    print(f"  Alpha:    {args.alpha_start} → {args.alpha_max} "
          f"(step={args.alpha_step})")
    print(f"  Batch:    {args.batch_size}")
    print(f"  Perturb:  {args.perturb} steps")
    print(f"  Model:    8→{HIDDEN}→2")
    print()

    t_start = time.perf_counter()
    last_update = t_start
    episode = 0

    try:
        while True:
            episode += 1
            print(f"Episode {episode}  α={alpha:.3f}  "
                  f"buffer={len(replay)}/{BUF_CAPACITY}")

            ep_buffer = run_episode(io, model, alpha,
                                    perturb_steps=args.perturb)
            replay.extend(ep_buffer)
            print(f"  Collected {len(ep_buffer)} steps")

            # Training update
            if len(replay) >= args.batch_size:
                now = time.perf_counter()
                if now - last_update > UPDATE_INTERVAL:
                    idx = np.random.choice(
                        len(replay), min(args.batch_size, len(replay)))
                    batch_obs = torch.tensor([replay[i][0] for i in idx],
                                             dtype=torch.float32, device=DEVICE)
                    batch_act = torch.tensor([replay[i][1] for i in idx],
                                             dtype=torch.float32, device=DEVICE)
                    batch_rew = torch.tensor([replay[i][2] for i in idx],
                                             dtype=torch.float32, device=DEVICE)

                    # Weighted regression: higher-reward transitions matter more
                    weights = torch.softmax(batch_rew / 1e4, dim=0)
                    pred = model(batch_obs)
                    weighted_loss = (weights.unsqueeze(-1) *
                                     (pred - batch_act) ** 2).mean()
                    l2_penalty = (pred ** 2).mean()
                    loss = weighted_loss + 0.01 * l2_penalty

                    optimizer.zero_grad()
                    loss.backward()
                    optimizer.step()

                    # Advance alpha toward max
                    alpha = min(alpha + args.alpha_step, args.alpha_max)

                    print(f"  Update: loss={loss.item():.3f}, "
                          f"alpha→{alpha:.3f}, lr={args.lr}")
                    last_update = now

            time.sleep(args.episode_delay)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        io.close()
        torch.save(model.state_dict(), "online_rl_model.pt")
        print(f"Saved online_rl_model.pt ({episode} episodes)")
