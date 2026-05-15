"""
Online MLP training from real car serial data — observe LQR, learn to imitate.

Reads 100Hz binary telemetry via serial, buffers (obs, u_lqr) pairs,
continuously trains an 8->16->2 MLP to predict LQR actions from states.

Usage:
    uv run python online_train.py                          # auto-detect port
    uv run python online_train.py -s /dev/ttyACM0          # specify port
    uv run python online_train.py -s /dev/ttyACM0 -t 120   # 2 minute session

Does NOT send any commands to the robot — safe to run during normal LQR operation.
"""
import argparse
import csv
import struct
import time
import threading
import queue
from collections import deque

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

# ─── Config ───
SYNC = 0xDD
PACKET_SIZE = 66  # 16-float protocol
STATE_KEYS = [
    "theta_L", "theta_R", "theta_1", "theta_2",
    "theta_L_dot", "theta_R_dot", "theta_dot_1", "theta_dot_2",
    "u_L", "u_R",
    "Target_theta_L", "Target_theta_R",
    "TargetVal_L", "TargetVal_R", "PWM_L", "PWM_R",
]
OBS_DIM = 8  # [pos_err_L, pos_err_R, th1, th2, thdotL, thdotR, thdot1, thdot2]
ACT_DIM = 2  # [u_res_L, u_res_R]

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
BUFFER_CAPACITY = 50000  # ~8 minutes at 100Hz
TRAIN_INTERVAL = 5.0  # seconds between training steps
BATCH_SIZE = 256
LEARNING_RATE = 1e-3
HIDDEN = 16

print(f"Device: {DEVICE}")


# ─── Model ───

class OnlineMLP(nn.Module):
    """MLP 8→32→2, trained to output residual correction to LQR baseline."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(8, HIDDEN), nn.ReLU(),
            nn.Linear(HIDDEN, 2),
        )

    def forward(self, x):
        return self.net(x)


# ─── Serial reader (thread) ───

def serial_reader(port: str, baud: int, buffer: deque, stop_event: threading.Event):
    """Read binary packets, compute LQR actions, push (obs, u_lqr) pairs to buffer."""
    try:
        import serial
    except ImportError:
        print("pyserial not installed. Run: pip install pyserial")
        return

    from balancing_robot.dynamics import get_lqr_gains
    K = get_lqr_gains()

    ser = serial.Serial(port, baud, timeout=1)
    buf = b""
    count = 0

    while not stop_event.is_set():
        n = ser.in_waiting or 1
        buf += ser.read(n)

        while len(buf) >= PACKET_SIZE:
            sync_idx = buf.find(bytes([SYNC]))
            if sync_idx < 0:
                buf = buf[-PACKET_SIZE:]
                break
            if len(buf) - sync_idx < PACKET_SIZE:
                break

            raw = buf[sync_idx:sync_idx + PACKET_SIZE]
            ck = 0
            for b in raw[:-1]:
                ck ^= b
            if ck != raw[-1]:
                buf = buf[sync_idx + 1:]
                continue

            floats = struct.unpack("<16f", raw[1:65])
            data = dict(zip(STATE_KEYS, floats))

            # Build 8D obs: [pos_err_L, pos_err_R, th1, th2, thdotL, thdotR, thdot1, thdot2]
            obs = np.array([
                data["theta_L"] - data["Target_theta_L"],
                data["theta_R"] - data["Target_theta_R"],
                data["theta_1"], data["theta_2"],
                data["theta_L_dot"], data["theta_R_dot"],
                data["theta_dot_1"], data["theta_dot_2"],
            ], dtype=np.float32)

            # LQR target action
            x = np.array([data["theta_L"], data["theta_R"],
                          data["theta_1"], data["theta_2"],
                          data["theta_L_dot"], data["theta_R_dot"],
                          data["theta_dot_1"], data["theta_dot_2"]])
            x_ref = np.array([data["Target_theta_L"], data["Target_theta_R"],
                              0, 0, 0, 0, 0, 0])
            u_lqr = -K @ (x - x_ref)
            target = np.clip(u_lqr, -5000, 5000).astype(np.float32)

            buffer.append((obs, target))
            count += 1
            buf = buf[sync_idx + PACKET_SIZE:]

    ser.close()
    print(f"Serial reader stopped. {count} packets received.")


# ─── Training loop ───

def train_step(model, optimizer, buffer, batch_size):
    """Train one step on random samples from buffer."""
    if len(buffer) < batch_size:
        return None, None

    indices = np.random.choice(len(buffer), batch_size, replace=False)
    batch_obs = np.array([buffer[i][0] for i in indices])
    batch_act = np.array([buffer[i][1] for i in indices])

    X = torch.from_numpy(batch_obs).to(DEVICE)
    Y = torch.from_numpy(batch_act).to(DEVICE)

    pred = model(X)
    loss = nn.MSELoss()(pred, Y)

    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

    # Also compute MAE and effective gain
    with torch.no_grad():
        mae = (pred - Y).abs().mean().item()
        # Check th2 gain: d_u/d_th2 from model
        base = torch.zeros(8, device=DEVICE)
        base[3] = 0.001
        u_th2 = model(base)
        base[3] = 0.0
        u0 = model(base)
        gain_th2 = (u_th2[0] - u0[0]).item() / 0.001

    return loss.item(), mae, gain_th2


# ─── Main ───

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Online MLP training from real car serial")
    parser.add_argument("-s", "--serial", type=str, default=None)
    parser.add_argument("-b", "--baud", type=int, default=460800)
    parser.add_argument("-t", "--time", type=float, default=120.0, help="Duration (s)")
    parser.add_argument("--hidden", type=int, default=16, help="Hidden layer size")
    parser.add_argument("--lr", type=float, default=1e-3, help="Learning rate")
    args = parser.parse_args()

    HIDDEN = args.hidden
    LEARNING_RATE = args.lr

    # Auto-detect port
    port = args.serial
    if port is None:
        import glob
        candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        if not candidates:
            candidates = [f"COM{i}" for i in range(1, 20)]
            print("Cannot auto-detect serial port. Use -s COMx")
            exit(1)
        port = candidates[0]
        print(f"Auto-detected: {port}")

    # Shared buffer (thread-safe deque)
    buffer = deque(maxlen=BUFFER_CAPACITY)

    # Start serial reader thread
    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=serial_reader, args=(port, args.baud, buffer, stop_event),
        daemon=True
    )
    reader_thread.start()
    print(f"Reading from {port} @ {args.baud} baud...")
    time.sleep(1.0)  # let serial fill up

    # Model and optimizer
    model = OnlineMLP().to(DEVICE)
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

    # Training history
    losses, maes, gains = [], [], []
    t_start = time.perf_counter()
    last_train = t_start
    last_print = t_start

    print(f"Online training started. Will run for {args.time}s.")
    print(f"  Buffer: {BUFFER_CAPACITY} samples")
    print(f"  Train interval: {TRAIN_INTERVAL}s")
    print(f"  Model: 8→{HIDDEN}→2, lr={LEARNING_RATE}")
    print()

    try:
        while time.perf_counter() - t_start < args.time:
            time.sleep(0.1)

            now = time.perf_counter()
            # Print status every 3s
            if now - last_print > 3.0:
                buf_size = len(buffer)
                loss_str = f"loss={losses[-1]:.1f}" if losses else "waiting for data"
                gain_str = f"th2_gain={gains[-1]:.0f}" if gains else ""
                mae_str = f"mae={maes[-1]:.0f}" if maes else ""
                print(f"  t={now - t_start:.0f}s  buffer={buf_size}  {loss_str}  {mae_str}  {gain_str}")
                last_print = now

            # Periodic training
            if now - last_train > TRAIN_INTERVAL and len(buffer) >= BATCH_SIZE:
                loss, mae, gain = train_step(model, optimizer, buffer, BATCH_SIZE)
                if loss is not None:
                    losses.append(loss)
                    maes.append(mae)
                    gains.append(gain)
                    last_train = now

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        stop_event.set()
        reader_thread.join(timeout=2.0)
        elapsed = time.perf_counter() - t_start

    # ─── Final report ───
    print(f"\n=== Session summary ({elapsed:.0f}s) ===")
    print(f"Total samples: {len(buffer)}")
    if losses:
        print(f"Training steps: {len(losses)}")
        print(f"Final loss: {losses[-1]:.1f}")
        print(f"Final MAE: {maes[-1]:.0f}")
        print(f"Final th2 gain: {gains[-1]:.0f}  (LQR: -18922)")

    if len(buffer) > 0:
        # Fit OLS on full buffer for comparison
        all_obs = np.array([b[0] for b in buffer])
        all_act = np.array([b[1] for b in buffer])
        W_ols = np.linalg.lstsq(all_obs, all_act, rcond=None)[0]
        print(f"\nOLS fit on all {len(buffer)} samples:")
        print(f"  W_ols vs LQR -K max|diff|: {np.max(np.abs(W_ols.T - (-get_lqr_gains()))):.2f}")

    # Save model
    torch.save(model.state_dict(), "online_model.pt")
    print("Saved online_model.pt")

    # Compute final model's OLS-like fit
    model.eval()
    with torch.no_grad():
        W_model = model.net[0].weight.data.cpu().numpy()
        if len(model.net) > 2 and hasattr(model.net[2], 'weight'):
            W_out = model.net[2].weight.data.cpu().numpy()  # 2 -> 32
            W_effective = W_out @ W_model  # 2 × 8
            print(f"  Effective W (MLP) max|abs|: {np.max(np.abs(W_effective)):.0f}")
