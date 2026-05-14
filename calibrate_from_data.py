"""
Calibrate simulation sensor noise from real car 100Hz data.

Usage:
    uv run python calibrate_from_data.py
"""
import csv
import numpy as np

TS = 0.01
STATE_NAMES = [
    "theta_L", "theta_R", "theta_1", "theta_2",
    "theta_L_dot", "theta_R_dot", "theta_dot_1", "theta_dot_2",
]
FILES = [
    "realcar/balance_steady_1.csv",
    "realcar/balance_steady_2.csv",
    "realcar/balance_disturb_1.csv",
    "realcar/balance_disturb_2.csv",
    "realcar/balance_line.csv",
    "realcar/balance_turn.csv",
]


def load_all():
    data = []
    for fn in FILES:
        with open(fn) as f:
            rows = list(csv.DictReader(f))
        arr = {col: np.array([float(r[col]) for r in rows]) for col in rows[0].keys()}
        data.append(arr)
        print(f"  {fn}: {len(rows)} samples, {float(rows[-1]['time_s']):.1f}s")
    return data


def calibrate_noise(data):
    """Per-state sensor noise via high-pass residual (MA window=5)."""
    noise_stds = []
    for name in STATE_NAMES:
        residuals = []
        for d in data:
            x = d[name]
            kernel = np.ones(5) / 5.0
            smooth = np.convolve(x, kernel, mode="same")
            res = x - smooth
            residuals.append(res[2:-2])
        std = float(np.std(np.concatenate(residuals)))
        noise_stds.append(std)

    noise_stds = np.array(noise_stds)
    print("\nSensor noise std (high-pass residual, per 10ms step):")
    for i, name in enumerate(STATE_NAMES):
        print(f"  {name}: {noise_stds[i]:.6f}")
    return noise_stds


def summarize(data):
    """Key statistics for simulation validation."""
    all_th1 = np.concatenate([d["theta_1"] for d in data])
    all_thdot = np.concatenate([d["theta_L_dot"] for d in data])
    all_u = np.concatenate([d["u_L"] for d in data])
    all_pwm = np.concatenate([d["PWM_L"] for d in data])
    all_tv = np.concatenate([d["TargetVal_L"] for d in data])

    print(f"\nReal data statistics (all {len(all_th1)} samples):")
    print(f"  theta_1:     std={np.rad2deg(np.std(all_th1)):.1f}deg  "
          f"[{np.rad2deg(np.min(all_th1)):.1f}, {np.rad2deg(np.max(all_th1)):.1f}]")
    print(f"  theta_dot_L: std={np.std(all_thdot):.3f} rad/s  "
          f"[{np.min(all_thdot):.1f}, {np.max(all_thdot):.1f}]")
    print(f"  u_L:         std={np.std(all_u):.0f} rad/s²  "
          f"[{np.min(all_u):.0f}, {np.max(all_u):.0f}]")
    print(f"  TargetVal_L: std={np.std(all_tv):.3f} rad/s")
    print(f"  PWM_L:       std={np.std(all_pwm):.0f}  "
          f"[{np.min(all_pwm):.0f}, {np.max(all_pwm):.0f}]")
    print(f"  PWM sat (>6800): {np.mean(np.abs(all_pwm) > 6800)*100:.1f}%")


if __name__ == "__main__":
    print("Loading real car data...")
    data = load_all()

    print("\n=== Sensor noise calibration ===")
    noise = calibrate_noise(data)

    print("\n=== Summary statistics ===")
    summarize(data)

    print(f"\n--- Update env.py NOISE_STD with: ---")
    print(f"NOISE_STD = np.{repr(noise)}")
