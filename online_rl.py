"""
Online RL: residual MLP correction to LQR, trained on real car via serial.

Protocol:
  PC reads 100Hz telemetry -> computes LQR+MLP action -> sends u back to STM32
  STM32 uses external u (with 500ms timeout fallback to internal controller)

Usage:
  uv run python online_rl.py -s /dev/ttyACM0

Safety:
  - Residual starts near zero (LQR baseline)
  - alpha scales residual magnitude, starts small
  - Falls back to LQR if PC crashes or disconnects
"""
import argparse
import struct
import time
import threading
from collections import deque

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

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
GAMMA = 0.99
UPDATE_INTERVAL = 1.0  # seconds between PPO-style updates
BUF_CAPACITY = 2000     # ~20s of data
ALPHA_START = 0     # residual scaling starts small

print(f"Device: {DEVICE}")

# ─── Residual MLP ───

class ResidualMLP(nn.Module):
    """Small MLP that outputs correction to LQR."""

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
        self.latest = None
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
        """Send u_L, u_R to STM32 (PC -> robot control command)."""
        buf = bytearray(EXT_PACKET_SIZE)
        buf[0] = EXT_SYNC
        struct.pack_into("<f", buf, 1, u_L)
        struct.pack_into("<f", buf, 5, u_R)
        self.ser.write(buf)

    def close(self):
        self.running = False
        self.ser.close()


# ─── RL training ───

def compute_reward(data: dict, alpha: float) -> float:
    """Quadratic cost (same as simulation)."""
    th1 = data["theta_1"]
    th2 = data["theta_2"]
    u_L = data["u_L"]
    u_R = data["u_R"]
    # Heavily penalize tilt
    cost = 10.0 * th1 ** 2 + 50.0 * th2 ** 2 + 1e-5 * (u_L ** 2 + u_R ** 2)
    return -cost


def run_episode(io: SerialIO, model: ResidualMLP, alpha: float,
                max_steps: int = 3000) -> list:
    """
    Run one self-balancing episode with RL policy.
    Returns list of (obs, action, reward, next_obs, done).
    """
    buffer = []
    last_obs = None
    last_action = None

    for step in range(max_steps):
        pkts = io.read_packets()
        if not pkts:
            time.sleep(0.001)
            continue

        data = pkts[-1]  # latest packet

        # Debug: first 10 steps, print received state and computed action
        if step < 10:
            th1 = np.rad2deg(data["theta_1"])
            thL = data["theta_L"]; thR = data["theta_R"]
            tL = data["Target_theta_L"]; tR = data["Target_theta_R"]
            uL_stm = data["u_L"]; uR_stm = data["u_R"]
            print(f"  step {step}: th1={th1:+.2f}deg thL={thL:.1f} tL={tL:.1f} "
                  f"STM32_u=({uL_stm:.0f},{uR_stm:.0f})")

        # Build 8D obs
        obs = np.array([
            data["theta_L"] - data["Target_theta_L"],
            data["theta_R"] - data["Target_theta_R"],
            data["theta_1"], data["theta_2"],
            data["theta_L_dot"], data["theta_R_dot"],
            data["theta_dot_1"], data["theta_dot_2"],
        ], dtype=np.float32)

        # Residual MLP
        with torch.no_grad():
            inp = torch.from_numpy(obs).float().to(DEVICE)
            u_res = model(inp).cpu().numpy()

        # PC sends only the residual delta (STM32 computes LQR internally)
        delta = alpha * u_res
        io.send_action(delta[0], delta[1])
        if step < 10:
            stm_u = (data["u_L"], data["u_R"])
            print(f"         PC delta=({delta[0]:.1f},{delta[1]:.1f})  STM32_u=({stm_u[0]:.0f},{stm_u[1]:.0f})")

        # Store transition
        reward = compute_reward(data, alpha)
        terminated = bool(abs(data["theta_1"]) > 0.7854 or
                          abs(data["theta_2"]) > 0.7854)

        if last_obs is not None:
            buffer.append((last_obs, last_action, reward, obs, terminated))

        last_obs = obs
        last_action = u_total  # store the ACTUAL action sent

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
    parser.add_argument("--alpha", type=float, default=0.01,
                        help="Initial residual scaling")
    parser.add_argument("--lr", type=float, default=3e-4,
                        help="Learning rate")
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

    alpha = args.alpha

    # Connect
    print(f"Connecting to {port} @ {args.baud}...")
    io = SerialIO(port, args.baud)
    time.sleep(0.5)

    # Model
    model = ResidualMLP().to(DEVICE)
    optimizer = optim.Adam(model.parameters(), lr=args.lr)

    # Replay buffer
    replay = deque(maxlen=BUF_CAPACITY)

    print(f"Online RL started. alpha={alpha}, lr={args.lr}")
    print(f"  Model: 8->{HIDDEN}->2 residual")
    print(f"  Protocol: PC sends u={EXT_SYNC:02x}+8bytes, STM32 timeout=500ms")
    print(f"  Safety: alpha starts small, falls back to LQR on disconnect")
    print()

    t_start = time.perf_counter()
    last_update = t_start
    episode = 0

    try:
        while True:
            # Wait for user to position the car (manual reset)
            input(f"Press Enter to start episode {episode + 1} (Ctrl+C to stop)...")
            print(f"  Episode {episode + 1} starting ...")

            # Run episode
            ep_buffer = run_episode(io, model, alpha)
            replay.extend(ep_buffer)
            print(f"  Collected {len(ep_buffer)} steps, buffer size={len(replay)}")

            # PPO-style update (simplified: policy gradient on collected data)
            if len(replay) >= 256:
                now = time.perf_counter()
                if now - last_update > UPDATE_INTERVAL:
                    # Sample batch
                    idx = np.random.choice(len(replay), min(256, len(replay)))
                    batch_obs = torch.tensor([replay[i][0] for i in idx],
                                             dtype=torch.float32, device=DEVICE)
                    batch_act = torch.tensor([replay[i][1] for i in idx],
                                             dtype=torch.float32, device=DEVICE)
                    batch_rew = torch.tensor([replay[i][2] for i in idx],
                                             dtype=torch.float32, device=DEVICE)

                    # Residual should be small; only non-zero if it improves reward
                    # Weight: higher reward = more influence
                    weights = torch.softmax(batch_rew / 1e4, dim=0)
                    # Regress toward the stored residual action
                    pred = model(batch_obs)
                    weighted_loss = (weights.unsqueeze(-1) *
                                     (pred - batch_act) ** 2).mean()
                    # L2 penalty: encourage residual to stay small
                    l2_penalty = (pred ** 2).mean()
                    weighted_loss = weighted_loss + 0.01 * l2_penalty

                    optimizer.zero_grad()
                    weighted_loss.backward()
                    optimizer.step()

                    # Slowly increase alpha
                    alpha = min(alpha * 1.02, 0.5)

                    print(f"  Update: loss={weighted_loss.item():.3f}, "
                          f"alpha={alpha:.3f}, lr={args.lr}")
                    last_update = now

            episode += 1

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        io.close()
        torch.save(model.state_dict(), "online_rl_model.pt")
        print(f"Saved online_rl_model.pt ({episode} episodes)")
