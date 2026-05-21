"""Compute closed-loop eigenvalues with FULL PI model including Kp term.

PI: pwm[k+1] = pwm[k] + Ki*TS*u[k] + Kp*TS*(u[k] - u[k-1])
Since u[k] = -K@x[k]:
  pwm[k+1] = pwm[k] - Ki*TS*K@x[k] - Kp*TS*K@(x[k] - x[k-1])

This Kp term on (x[k]-x[k-1]) ≈ Kp*TS*K@dx/dt provides derivative feedback
that can stabilize the system.

Augmented state: [x(8), x_prev(8), pwm_L, pwm_R] = 18-dim
"""
import numpy as np

from balancing_robot.dynamics import compute_state_space, get_lqr_gains, TS

_, _, G, H = compute_state_space()
K = get_lqr_gains()

MOTO_KP, MOTO_KI = 25, 35
PWM_MAX = 6900


def augmented_eigenvalues(motor_gain, back_emf=0.0):
    """18-state augmented closed-loop: x, x_prev, pwm."""
    n_x = 8
    n_aug = 2 * n_x + 2  # 18

    A = np.zeros((n_aug, n_aug))

    # Row block 1: x[k+1] = G@x[k] + H_motor @ pwm[k] - back_emf correction
    A[:n_x, :n_x] = G.copy()
    # back_emf: subtract from wheel velocity diagonal
    A[4, 4] -= back_emf * TS
    A[5, 5] -= back_emf * TS

    # PWM → state via motor
    # motor_accel = motor_gain * pwm
    # x[k+1] += H @ [mg*pwm_L, mg*pwm_R]
    for j in range(8):
        A[j, 16] = H[j, 0] * motor_gain  # pwm_L → state[j]
        A[j, 17] = H[j, 1] * motor_gain  # pwm_R → state[j]

    # Row block 2: z[k+1] = x[k] (z stores x[k-1])
    A[n_x:2*n_x, :n_x] = np.eye(n_x)

    # Row block 3: pwm[k+1] = pwm[k] - Ki*TS*K@x[k] - Kp*TS*K@(x[k]-z[k])
    # = pwm[k] - (Ki+Kp)*TS*K@x[k] + Kp*TS*K@z[k]
    for j in range(8):
        # x[k] contribution
        A[16, j] = -(MOTO_KI + MOTO_KP) * TS * K[0, j]
        A[17, j] = -(MOTO_KI + MOTO_KP) * TS * K[1, j]
        # z[k] (= x[k-1]) contribution
        A[16, n_x + j] = MOTO_KP * TS * K[0, j]
        A[17, n_x + j] = MOTO_KP * TS * K[1, j]

    A[16, 16] = 1.0  # pwm_L integrator
    A[17, 17] = 1.0  # pwm_R integrator

    return np.linalg.eigvals(A)


print("=" * 70)
print("Full PI model: pwm += Ki*TS*u + Kp*TS*(u[k]-u[k-1])")
print("18-state augmented closed-loop eigenvalues")
print("=" * 70)

# Baseline
eig_ideal = np.linalg.eigvals(G - H @ K)
print(f"\nIdeal LQR:                max|λ| = {max(abs(eig_ideal)):.6f}")

# With Kp term only (no Ki)
print("\n--- With Kp term (derivative feedback) ---")
for mg in [0.001, 0.005, 0.01, 0.02, 0.05, 0.1]:
    for be in [0, 1, 2, 5, 10]:
        eigs = augmented_eigenvalues(mg, be)
        max_ev = max(abs(eigs))
        n_unstable = sum(abs(eigs) > 1.0)
        marker = "  <-- STABLE" if max_ev < 1.0 else ""
        if max_ev < 1.02:  # show near-stable
            print(f"  mg={mg:.4f}, be={be:.1f}: max|λ|={max_ev:.6f}, "
                  f"unstable_modes={n_unstable}{marker}")

# Full sweep
print("\n--- Full PI (Ki+Kp) stability sweep ---")
print(f"{'mg':>8s} {'be':>6s} {'max|λ|':>10s} {'#unstable':>10s}")
print("-" * 36)
for mg in [0.0001, 0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1]:
    for be in [0.0, 1.0, 2.0, 5.0, 10.0]:
        eigs = augmented_eigenvalues(mg, be)
        max_ev = max(abs(eigs))
        n_u = sum(abs(eigs) > 1.0)
        marker = " STABLE!" if max_ev < 1.0 else ""
        print(f"{mg:>8.5f} {be:>6.1f} {max_ev:>10.6f} {n_u:>10d}{marker}")

# ─── What eigenvalue crosses first? ───
print("\n--- Top 5 eigenvalues for mg=0.001, be=0 ---")
eigs = augmented_eigenvalues(0.001, 0.0)
for ev in sorted(eigs, key=lambda x: -abs(x))[:5]:
    f_hz = abs(np.angle(ev)) / (2 * np.pi * TS) if abs(ev) > 0 else 0
    print(f"  λ={ev.real:+.6f}{ev.imag:+.6f}j  |λ|={abs(ev):.6f}  f≈{f_hz:.2f}Hz")

print("\n--- Top 5 eigenvalues for mg=0.005, be=2.0 ---")
eigs = augmented_eigenvalues(0.005, 2.0)
for ev in sorted(eigs, key=lambda x: -abs(x))[:5]:
    f_hz = abs(np.angle(ev)) / (2 * np.pi * TS) if abs(ev) > 0 else 0
    print(f"  λ={ev.real:+.6f}{ev.imag:+.6f}j  |λ|={abs(ev):.6f}  f≈{f_hz:.2f}Hz")
