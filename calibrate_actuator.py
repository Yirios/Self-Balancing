"""Fit data-driven closed-loop model from real car data.

Approach: system identification -> A_cl where x[k+1] = A_cl @ x[k]
plus estimated noise covariance from residuals.

Also sweeps various mathematical blocks (h_scale, lowpass, delay, dead zone)
to find the combination whose steady-state statistics best match
real hardware LQR oscillation (th1_std=5.4deg, th2_std=1.4deg, thdot_std=3.0).

Usage:
    uv run python calibrate_actuator.py          # sweep all blocks
    uv run python calibrate_actuator.py --save   # fit & save data-driven model
"""
import numpy as np
import csv
from collections import deque

from balancing_robot.dynamics import compute_state_space, get_lqr_gains, TS

_, _, G, H = compute_state_space()
K = get_lqr_gains()

# ─── Load real data ───
with open("realcar/balance_disturb_2.csv") as f:
    rows = list(csv.DictReader(f))
real = np.array([[float(r[k]) for k in [
    "theta_1", "theta_2", "theta_L_dot", "u_L"
]] for r in rows])
real_s = {
    'th1_std': np.rad2deg(np.std(real[100:, 0])),
    'th2_std': np.rad2deg(np.std(real[100:, 1])),
    'thdot_std': np.std(real[100:, 2]),
    'u_std': np.std(real[100:, 3]),
}
print(f"Real target: th1_std={real_s['th1_std']:.2f}°, "
      f"th2_std={real_s['th2_std']:.2f}°, thdot_std={real_s['thdot_std']:.2f}")


def simulate_block(n_steps, block_type, params, disturb_at=150):
    """Run LQR episode with specified actuator block.

    Block types:
      'h_scale':        u_eff = h_scale * u
      'lowpass':        u_eff[k] = α*u_eff[k-1] + (1-α)*u[k]
      'delay':          u_eff = u[k - delay_steps]
      'delay_lowpass':  delay + lowpass combined
      'saturation':     u_eff = clip(u, -max_accel, +max_accel)
      'rate_limit':     u_eff changes at most rate_limit per step
      'lowpass_sat':    lowpass + saturation
    """
    x = np.zeros(8)
    x[2] = 0.02
    tL, tR = 0.0, 0.0

    # Block state
    u_eff_hist_L = deque([0.0] * max(params.get('delay', 0), 1),
                         maxlen=max(params.get('delay', 0), 1))
    u_eff_hist_R = deque([0.0] * max(params.get('delay', 0), 1),
                         maxlen=max(params.get('delay', 0), 1))
    lp_state = np.zeros(2)  # lowpass state per wheel
    rate_state = np.zeros(2)  # rate limiter state

    history = []
    for step in range(n_steps):
        x_ref = np.array([tL, tR, 0, 0, 0, 0, 0, 0])
        u = -K @ (x - x_ref)

        # ── Apply block ──
        u_eff = u.copy()

        if block_type in ('lowpass', 'delay_lowpass', 'lowpass_sat'):
            tau = params.get('tau', 0.03)
            alpha = np.exp(-TS / tau) if tau > 0 else 0.0
            lp_state = alpha * lp_state + (1 - alpha) * u
            u_eff = lp_state.copy()

        if block_type in ('delay', 'delay_lowpass'):
            d = params.get('delay', 7)
            u_eff_hist_L.append(u_eff[0])
            u_eff_hist_R.append(u_eff[1])
            u_eff[0] = u_eff_hist_L[0]
            u_eff[1] = u_eff_hist_R[0]

        if block_type in ('h_scale',):
            u_eff = params.get('h_scale', 1.0) * u

        if block_type in ('saturation', 'lowpass_sat'):
            u_eff = np.clip(u_eff, -params.get('sat', 5000), params.get('sat', 5000))

        if block_type == 'rate_limit':
            rl = params.get('rate', 100)  # max change per step (rad/s² per 10ms)
            delta = np.clip(u_eff - rate_state, -rl, rl)
            rate_state += delta
            u_eff = rate_state.copy()

        # ── State update (always through G,H) ──
        x = G @ x + H @ u_eff

        if disturb_at and step == disturb_at:
            x[7] += 1.5

        history.append(np.concatenate([x, u_eff]))
    return np.array(history)


def score(data, trim=300):
    """Score how close simulation steady-state is to real data."""
    d = data[trim:]
    if len(d) < 50:
        return np.inf
    th1_std = np.rad2deg(np.std(d[:, 2]))
    th2_std = np.rad2deg(np.std(d[:, 3]))
    thdot_std = np.std(d[:, 4])

    if np.isnan(th1_std) or th1_std > 100:
        return np.inf

    return (
        ((th1_std - real_s['th1_std']) / real_s['th1_std']) ** 2 +
        ((th2_std - real_s['th2_std']) / real_s['th2_std']) ** 2 +
        ((thdot_std - real_s['thdot_std']) / real_s['thdot_std']) ** 2
    )


def simulate_combined(n_steps, hs, tau, disturb_at=150):
    """Combined: u_eff = hs * lowpass(u), h_scale applied to filtered u."""
    x = np.zeros(8)
    x[2] = 0.02
    tL, tR = 0.0, 0.0

    alpha = np.exp(-TS / tau) if tau > 0 else 0.0
    lp_state = np.zeros(2)

    history = []
    for step in range(n_steps):
        x_ref = np.array([tL, tR, 0, 0, 0, 0, 0, 0])
        u = -K @ (x - x_ref)

        lp_state = alpha * lp_state + (1 - alpha) * u
        u_eff = hs * lp_state

        x = G @ x + H @ u_eff

        if disturb_at and step == disturb_at:
            x[7] += 1.5

        history.append(np.concatenate([x, u_eff]))
    return np.array(history)


# ─── Sweep all block types ───
print("\n" + "=" * 70)
print("Block type sweeps → matching real hardware oscillation")
print("=" * 70)

results = []

# 1. h_scale only
for hs in [0.01, 0.02, 0.05, 0.08, 0.10, 0.12, 0.15, 0.18, 0.20, 0.25, 0.30, 0.4, 0.5]:
    data = simulate_block(1000, 'h_scale', {'h_scale': hs}, disturb_at=200)
    s = score(data)
    if s < np.inf:
        results.append((s, 'h_scale', f'hs={hs:.3f}', data))

# 2. lowpass only
for tau in [0.01, 0.02, 0.03, 0.05, 0.08, 0.10, 0.15, 0.20, 0.30, 0.50]:
    data = simulate_block(1000, 'lowpass', {'tau': tau}, disturb_at=200)
    s = score(data)
    if s < np.inf:
        results.append((s, 'lowpass', f'tau={tau*1000:.0f}ms', data))

# 3. delay + lowpass
for d in [4, 5, 6, 7, 8, 10, 12]:
    for tau in [0.01, 0.02, 0.03, 0.05, 0.08, 0.10]:
        data = simulate_block(1000, 'delay_lowpass', {'delay': d, 'tau': tau},
                              disturb_at=200)
        s = score(data)
        if s < np.inf:
            results.append((s, 'delay+LPF', f'd={d*10}ms,tau={tau*1000:.0f}ms', data))

# 4. velocity damping + lowpass
for damp in [0.5, 1.0, 2.0, 3.0, 5.0]:
    for tau in [0.01, 0.02, 0.03, 0.05, 0.10]:
        data = simulate_block(1000, 'lowpass', {'tau': tau}, disturb_at=200)
        s = score(data)
        if s < np.inf:
            results.append((s, 'LPF', f'tau={tau*1000:.0f}ms', data))

# 5. Saturation only
for sat in [10, 20, 50, 100, 200, 500, 1000]:
    data = simulate_block(1000, 'saturation', {'sat': sat}, disturb_at=200)
    s = score(data)
    if s < np.inf:
        results.append((s, 'sat', f'sat={sat}', data))

# 6. Rate limit
for rate in [5, 10, 20, 50, 100, 200, 500]:
    data = simulate_block(1000, 'rate_limit', {'rate': rate}, disturb_at=200)
    s = score(data)
    if s < np.inf:
        results.append((s, 'rate_limit', f'rate={rate}', data))

# 7. lowpass + saturation
for tau in [0.01, 0.02, 0.03, 0.05, 0.08, 0.10]:
    for sat in [20, 50, 100, 200, 500, 1000]:
        data = simulate_block(1000, 'lowpass_sat', {'tau': tau, 'sat': sat},
                              disturb_at=200)
        s = score(data)
        if s < np.inf:
            results.append((s, 'LPF+sat', f'tau={tau*1000:.0f}ms,sat={sat}', data))

# 8. COMBINED: h_scale + lowpass (best of both worlds)
for hs in [0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50]:
    for tau in [0.01, 0.015, 0.02, 0.025, 0.03, 0.04, 0.05, 0.08]:
        data = simulate_block(1000, 'delay_lowpass',
                              {'delay': 0, 'tau': tau}, disturb_at=200)
        # Apply h_scale manually after
        s = score(data)
        if s < np.inf:
            # Re-simulate with h_scale factored in
            # Actually, need a new block for this
            pass

# 9. Custom: h_scale * lowpass(u)
for hs in [0.10, 0.12, 0.15, 0.18, 0.20, 0.25, 0.30, 0.40]:
    for tau in [0.01, 0.015, 0.02, 0.025, 0.03, 0.04, 0.05, 0.08, 0.10]:
        data = simulate_combined(1000, hs=hs, tau=tau, disturb_at=200)
        s = score(data)
        if s < np.inf:
            results.append((s, 'hs+LPF', f'hs={hs:.2f},tau={tau*1000:.0f}ms', data))

# 10. Dead-zone + lowpass + h_scale (motor friction model)
def simulate_deadzone(n_steps, hs, tau, dead, disturb_at=150):
    """Dead zone on u: |u| < dead → 0, else u. Then lowpass, then scale."""
    x = np.zeros(8)
    x[2] = 0.02
    tL, tR = 0.0, 0.0
    alpha = np.exp(-TS / tau) if tau > 0 else 0.0
    lp_state = np.zeros(2)

    history = []
    for step in range(n_steps):
        x_ref = np.array([tL, tR, 0, 0, 0, 0, 0, 0])
        u = -K @ (x - x_ref)

        # Dead zone: zero out small u (mimics static friction)
        u_dz = np.where(np.abs(u) < dead, 0.0, u - np.sign(u) * dead)

        lp_state = alpha * lp_state + (1 - alpha) * u_dz
        u_eff = hs * lp_state

        x = G @ x + H @ u_eff

        if disturb_at and step == disturb_at:
            x[7] += 1.5

        history.append(np.concatenate([x, u_eff]))
    return np.array(history)

for hs in [0.15, 0.20, 0.25, 0.30, 0.40]:
    for tau in [0.010, 0.015, 0.020, 0.025, 0.030, 0.040]:
        for dead in [5, 10, 20, 50, 100, 200, 500]:
            data = simulate_deadzone(1000, hs=hs, tau=tau, dead=dead,
                                     disturb_at=200)
            s = score(data)
            if s < np.inf:
                results.append((s, 'dz+hs+LPF',
                                f'hs={hs:.2f},tau={tau*1000:.0f}ms,dz={dead}',
                                data))

# 11. h_scale + lowpass + persistent disturbance noise
# Real robot has continuous vibration/noise excitation from floor, sensors, etc.
def simulate_noisy(n_steps, hs, tau, noise_std, disturb_at=150):
    """h_scale * lowpass(u) + continuous noise injection on theta_dot_2."""
    x = np.zeros(8)
    x[2] = 0.02
    tL, tR = 0.0, 0.0
    alpha = np.exp(-TS / tau) if tau > 0 else 0.0
    lp_state = np.zeros(2)
    rng = np.random.RandomState(42)

    history = []
    for step in range(n_steps):
        x_ref = np.array([tL, tR, 0, 0, 0, 0, 0, 0])
        u = -K @ (x - x_ref)

        lp_state = alpha * lp_state + (1 - alpha) * u
        u_eff = hs * lp_state

        x = G @ x + H @ u_eff

        # Continuous small disturbance to theta_dot_2 (simulates real vibration)
        x[7] += rng.normal(0, noise_std)

        if disturb_at and step == disturb_at:
            x[7] += 1.5

        history.append(np.concatenate([x, u_eff]))
    return np.array(history)

for hs in [0.15, 0.18, 0.20, 0.22, 0.25, 0.28, 0.30, 0.35]:
    for tau in [0.010, 0.015, 0.020, 0.025, 0.030, 0.040]:
        for noise in [0.02, 0.05, 0.08, 0.10, 0.15, 0.20, 0.30]:
            data = simulate_noisy(1000, hs=hs, tau=tau, noise_std=noise,
                                  disturb_at=200)
            s = score(data)
            if s < np.inf:
                results.append((s, 'hs+LPF+noise',
                                f'hs={hs:.2f},tau={tau*1000:.0f}ms,n={noise:.2f}',
                                data))

# 12. Second-order resonant lowpass filter (fitted to 38° phase lag @ 0.83Hz)
# H(s) = ω_n² / (s² + 2ζω_n s + ω_n²), discretized via bilinear
from scipy import signal as sig

def simulate_resonant(n_steps, hs, wn, zeta, disturb_at=150):
    """h_scale * resonant_2nd_order_lowpass(u)"""
    x = np.zeros(8); x[2] = 0.02
    tL, tR = 0.0, 0.0

    # Design continuous 2nd-order lowpass, discretize at 100Hz
    # Tune ω_n (rad/s) and ζ to match measured phase lag
    b, a = sig.butter(2, wn / (2 * np.pi * 50))  # Wn in [0,1] for butter
    # Actually use bilinear transform explicitly
    b_cont = [wn**2]
    a_cont = [1, 2*zeta*wn, wn**2]
    sys_d = sig.cont2discrete((b_cont, a_cont), TS, method='bilinear')
    b_d = sys_d[0].flatten()
    a_d = sys_d[1].flatten()

    zi_L = np.zeros(len(b_d) - 1)
    zi_R = np.zeros(len(b_d) - 1)

    history = []
    for step in range(n_steps):
        x_ref = np.array([tL, tR, 0, 0, 0, 0, 0, 0])
        u = -K @ (x - x_ref)

        fL, zi_L = sig.lfilter(b_d, a_d, [u[0]], zi=zi_L)
        fR, zi_R = sig.lfilter(b_d, a_d, [u[1]], zi=zi_R)
        u_eff = hs * np.array([fL[0], fR[0]])

        x = G @ x + H @ u_eff

        if disturb_at and step == disturb_at:
            x[7] += 1.5

        history.append(np.concatenate([x, u_eff]))
    return np.array(history)

# Sweep ω_n, ζ around the target phase lag
for wn in [8.0, 9.0, 10.0, 11.0, 12.0, 14.0, 16.0, 20.0]:
    for zeta in [0.3, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8, 1.0]:
        # Compute phase at 0.83 Hz for reference
        w83 = 2 * np.pi * 0.83
        phase_theory = -np.degrees(np.arctan2(
            2 * zeta * w83 / wn, 1 - (w83 / wn)**2))
        # Only test filters that give 20-50° phase (close to measured 38°)
        if 15 < abs(phase_theory) < 55:
            for hs in [0.15, 0.20, 0.25, 0.30, 0.40, 0.50]:
                data = simulate_resonant(1000, hs, wn, zeta, disturb_at=200)
                s = score(data)
                if s < np.inf:
                    d = data[300:]
                    label = f'hs={hs:.2f},wn={wn:.0f},z={zeta:.2f}'
                    results.append((s, 'resonant', label, data))

# 13. Finer sweep of best h_scale + lowpass region
for hs in np.arange(0.10, 0.45, 0.02):
    for tau in np.arange(0.010, 0.040, 0.002):
        hs = round(hs, 3)
        tau = round(tau, 4)
        data = simulate_combined(1000, hs=hs, tau=tau, disturb_at=200)
        s = score(data)
        if s < np.inf:
            results.append((s, 'hs+LPF(fine)',
                            f'hs={hs:.2f},tau={tau*1000:.0f}ms', data))

# Sort and display
results.sort(key=lambda r: r[0])
print(f"\n{'rank':>4s} {'block':>12s} {'params':>25s} {'score':>8s} "
      f"{'th1_std':>8s} {'th2_std':>8s} {'thdot_std':>9s}")
print("-" * 80)
for i, (s, block, params, data) in enumerate(results[:30]):
    d = data[300:]
    th1s = np.rad2deg(np.std(d[:, 2]))
    th2s = np.rad2deg(np.std(d[:, 3]))
    thds = np.std(d[:, 4])
    print(f"{i+1:>4d} {block:>12s} {params:>25s} {s:>8.4f} "
          f"{th1s:>7.2f}° {th2s:>7.2f}° {thds:>8.2f}")

if results:
    best_s, best_block, best_params, best_data = results[0]
    print(f"\nBest: {best_block} {best_params}, score={best_s:.4f}")
    d = best_data[300:]
    print(f"  th1_std={np.rad2deg(np.std(d[:,2])):.2f}° "
          f"(target={real_s['th1_std']:.2f}°)")
    print(f"  th2_std={np.rad2deg(np.std(d[:,3])):.2f}° "
          f"(target={real_s['th2_std']:.2f}°)")
    print(f"  thdot_std={np.std(d[:,4]):.2f} "
          f"(target={real_s['thdot_std']:.2f})")


# ─── Data-driven model fitting (run with --save) ───
if __name__ == "__main__" and "--save" in __import__("sys").argv:
    import glob
    print("\n" + "=" * 70)
    print("Fitting data-driven closed-loop model from all real data")
    print("=" * 70)

    STATE_KEYS = ['theta_L', 'theta_R', 'theta_1', 'theta_2',
                  'theta_L_dot', 'theta_R_dot', 'theta_dot_1', 'theta_dot_2']

    files = sorted(glob.glob("realcar/balance_*.csv"))
    print(f"Files: {files}")

    X_all, Xn_all = [], []
    for fn in files:
        with open(fn) as f:
            rows = list(csv.DictReader(f))
        X = np.array([[float(r[k]) for k in STATE_KEYS] for r in rows])
        X_all.append(X[:-1])
        Xn_all.append(X[1:])

    x_curr = np.concatenate(X_all)
    x_next = np.concatenate(Xn_all)
    print(f"Total samples: {len(x_curr)}")

    # Fit A_cl with regularization
    lam = 0.01
    A_cl = np.linalg.solve(
        x_curr.T @ x_curr + lam * np.eye(8),
        x_curr.T @ x_next
    ).T

    # Fit noise covariance from residuals
    residuals = x_next - x_curr @ A_cl.T
    noise_cov = np.cov(residuals.T)

    # Save
    np.savez("real_data_model.npz", A_cl=A_cl, noise_cov=noise_cov,
             noise_scale=1.3)

    print(f"\nSaved real_data_model.npz")
    print(f"  A_cl shape: {A_cl.shape}")
    print(f"  noise_cov shape: {noise_cov.shape}")
    print(f"  noise_scale: 1.3 (tuned to match th1_std)")

    # Verify eigenvalues
    eigs = np.linalg.eigvals(A_cl)
    print(f"\nA_cl eigenvalues:")
    for ev in sorted(eigs, key=lambda x: -abs(x)):
        f_hz = abs(np.angle(ev)) / (2 * np.pi * TS) if abs(ev) > 1e-6 else 0
        print(f"  λ={ev.real:+.4f}{ev.imag:+.4f}j  |λ|={abs(ev):.4f}  f≈{f_hz:.2f}Hz")

    # Quick simulation to verify
    rng = np.random.RandomState(42)
    x = np.zeros(8)
    x[2] = 0.02
    th1_vals = []
    for _ in range(5000):
        x = A_cl @ x + rng.multivariate_normal(np.zeros(8), noise_cov) * 1.3
        th1_vals.append(np.rad2deg(x[2]))
        if abs(x[2]) > 0.7854:
            print(f"\n  WARNING: fell at step {len(th1_vals)}")
            break
    th1 = np.array(th1_vals)
    trim = 1000
    print(f"\nVerification ({len(th1)} steps):")
    print(f"  th1_std={np.std(th1[trim:]):.2f}°  p-p={np.ptp(th1[trim:]):.1f}°")
    print(f"  Target: th1_std=5.47°  p-p=31.4°")
