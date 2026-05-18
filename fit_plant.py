"""Fit open-loop plant model from real hardware data.

x[k+1] = A_plant @ x[k] + B_plant @ u[k] + noise

This captures PI velocity loop + motor response + friction + sensor dynamics
in the A_plant, B_plant matrices. LQR controller is NOT embedded — u is an
external input, so any controller (LQR, BC, RL) can be evaluated.

Usage:
    uv run python fit_plant.py          # fit and save real_data_model.npz
    uv run python fit_plant.py --plot   # show diagnostics
"""
import numpy as np
import csv
import glob

from balancing_robot.dynamics import compute_state_space, get_lqr_gains, TS

_, _, G, H = compute_state_space()
K = get_lqr_gains()

STATE_KEYS = ['theta_L', 'theta_R', 'theta_1', 'theta_2',
              'theta_L_dot', 'theta_R_dot', 'theta_dot_1', 'theta_dot_2']
ACTION_KEYS = ['u_L', 'u_R']

files = sorted(glob.glob("realcar/balance_*.csv"))
print(f"Loading {len(files)} files: {files}")

X_curr, U_curr, X_next = [], [], []
for fn in files:
    with open(fn) as f:
        rows = list(csv.DictReader(f))
    X = np.array([[float(r[k]) for k in STATE_KEYS] for r in rows])
    U = np.array([[float(r[k]) for k in ACTION_KEYS] for r in rows])
    print(f"  {fn}: {len(X)} samples")
    X_curr.append(X[:-1])
    U_curr.append(U[:-1])
    X_next.append(X[1:])

x = np.concatenate(X_curr)     # (N, 8)
u = np.concatenate(U_curr)     # (N, 2)
xn = np.concatenate(X_next)    # (N, 8)
print(f"\nTotal samples: {len(x)}")

# --- Fit x[k+1] = A_plant @ x[k] + B_plant @ u[k] ---
X_aug = np.concatenate([x, u], axis=1)  # (N, 10)

# Cross-validate: find best regularization
best_lam = 0.01
best_err = np.inf
for lam in [0.0, 1e-6, 0.0001, 0.001, 0.003, 0.01, 0.03, 0.1, 0.3, 1.0]:
    # 80/20 split
    n = len(x)
    idx = np.random.RandomState(0).permutation(n)
    n_train = int(n * 0.8)
    Xtr, Xte = X_aug[idx[:n_train]], X_aug[idx[n_train:]]
    ytr, yte = xn[idx[:n_train]], xn[idx[n_train:]]

    A_sys = np.linalg.solve(
        Xtr.T @ Xtr + lam * np.eye(10),
        Xtr.T @ ytr
    ).T
    err = np.mean((yte - Xte @ A_sys.T) ** 2)
    if err < best_err:
        best_err = err
        best_lam = lam

print(f"Best lambda: {best_lam}  (MSE={best_err:.6f})")

# Final fit with best lambda
A_sys = np.linalg.solve(
    X_aug.T @ X_aug + best_lam * np.eye(10),
    X_aug.T @ xn
).T  # (8, 10)

A_plant = A_sys[:, :8]
B_plant = A_sys[:, 8:]

print(f"\nA_plant (8x8): max|abs|={np.max(np.abs(A_plant)):.2f}")
print(f"B_plant (8x2): max|abs|={np.max(np.abs(B_plant)):.4f}")

# Compare with ideal G, H
print(f"\nIdeal G max|abs|={np.max(np.abs(G)):.2f}")
print(f"Ideal H max|abs|={np.max(np.abs(H)):.4f}")
print(f"||A_plant - G|| / ||G|| = {np.linalg.norm(A_plant - G) / np.linalg.norm(G):.4f}")
print(f"||B_plant - H|| / ||H|| = {np.linalg.norm(B_plant - H) / np.linalg.norm(H):.4f}")

# Noise covariance from residuals
residuals = xn - X_aug @ A_sys.T
noise_cov = np.cov(residuals.T)
print(f"\nNoise std (diag): {np.sqrt(np.diag(noise_cov))}")

# --- Verify: closed-loop LQR stability on identified plant ---
# Augmented: z[k+1] = A_plant @ z[k] + B_plant @ (-K @ z[k])
#                    = (A_plant - B_plant @ K) @ z[k]
A_cl_plant = A_plant - B_plant @ K
eigs_plant = np.linalg.eigvals(A_cl_plant)
max_ev = max(abs(eigs_plant))
n_unstable = sum(abs(eigs_plant) > 1.0)
print(f"\nIdentified plant + LQR (10ms):")
print(f"  max|λ| = {max_ev:.6f}  {'STABLE' if max_ev < 1.0 else 'UNSTABLE'}  "
      f"(#unstable={n_unstable})")
for ev in sorted(eigs_plant, key=lambda x: -abs(x))[:6]:
    f = abs(np.angle(ev)) / (2 * np.pi * TS) if abs(ev) > 1e-6 else 0
    print(f"  λ={ev.real:+.4f}{ev.imag:+.4f}j  |λ|={abs(ev):.4f}  f≈{f:.2f}Hz")

# Compare with ideal LQR
eigs_ideal = np.linalg.eigvals(G - H @ K)
print(f"\nIdeal G - H@K: max|λ| = {max(abs(eigs_ideal)):.6f}")

# --- Save ---
np.savez("real_data_model.npz",
         A_plant=A_plant, B_plant=B_plant,
         noise_cov=noise_cov, noise_scale=1.0)
print("\nSaved real_data_model.npz (A_plant, B_plant, noise_cov)")

# --- Quick time-domain verification ---
rng = np.random.RandomState(42)
x = np.zeros(8)
x[2] = 0.02
tL, tR = 0.0, 0.0
history = []
for step in range(2000):
    x_ref = np.array([tL, tR, 0, 0, 0, 0, 0, 0])
    u_cmd = -K @ (x - x_ref)
    noise = rng.multivariate_normal(np.zeros(8), noise_cov) * 1.0
    x = A_plant @ x + B_plant @ u_cmd + noise
    history.append(np.concatenate([x, u_cmd]))
    if abs(x[2]) > 0.7854:
        print(f"\nWARNING: fell at step {step}")
        break

hist = np.array(history)
trim = 500
print(f"\nTime-domain ({len(hist)} steps):")
print(f"  th1_std={np.rad2deg(np.std(hist[trim:, 2])):.2f}° "
      f"th2_std={np.rad2deg(np.std(hist[trim:, 3])):.2f}° "
      f"u_std={np.std(hist[trim:, 8]):.1f}")
print(f"  th1 peak-to-peak={np.rad2deg(np.ptp(hist[trim:, 2])):.1f}°")

# --- Plot if requested ---
if "--plot" in __import__("sys").argv:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(np.rad2deg(hist[:, 2]), label='θ₁ (body)')
    axes[0].plot(np.rad2deg(hist[:, 3]), label='θ₂ (pendulum)')
    axes[0].set_ylabel('deg'); axes[0].legend(); axes[0].grid(True)
    axes[1].plot(hist[:, 4], label='θ̇_L')
    axes[1].plot(hist[:, 5], label='θ̇_R')
    axes[1].set_ylabel('rad/s'); axes[1].legend(); axes[1].grid(True)
    axes[2].plot(hist[:, 8], label='u_L')
    axes[2].plot(hist[:, 9], label='u_R')
    axes[2].set_ylabel('u'); axes[2].set_xlabel('step (10ms)')
    axes[2].legend(); axes[2].grid(True)
    plt.suptitle('Identified plant under LQR control')
    plt.tight_layout()
    plt.show()

    # Compare eigenvalues
    fig2, ax2 = plt.subplots(figsize=(6, 6))
    ax2.add_patch(plt.Circle((0, 0), 1, fill=False, color='gray', ls='--'))
    eigs_i = np.linalg.eigvals(G - H @ K)
    eigs_p = np.linalg.eigvals(A_cl_plant)
    ax2.scatter(eigs_i.real, eigs_i.imag, marker='o', label='Ideal', s=30)
    ax2.scatter(eigs_p.real, eigs_p.imag, marker='x', label='Identified', s=30)
    ax2.axhline(0, color='gray', lw=0.5)
    ax2.axvline(0, color='gray', lw=0.5)
    ax2.set_xlabel('Re'); ax2.set_ylabel('Im')
    ax2.set_title('Closed-loop eigenvalues: Ideal vs Identified plant')
    ax2.legend()
    ax2.set_aspect('equal')
    plt.tight_layout()
    plt.show()
