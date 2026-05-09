import numpy as np
from scipy.signal import cont2discrete

# Physical parameters (from inverted_pendulum_on_self_balancing_robot.m)
M_1 = 0.9
M_2 = 0.1
R = 0.0335
L_1 = 0.126
L_2 = 0.390
l_1 = L_1 / 2.0
l_2 = L_2 / 2.0
G = 9.8
I_1 = (1.0 / 12.0) * M_1 * L_1**2
I_2 = (1.0 / 12.0) * M_2 * L_2**2

TS = 0.005  # 5ms, matching hardware MPU6050 interrupt


def compute_p() -> np.ndarray:
    """4x4 inertia/coupling matrix p."""
    p = np.zeros((4, 4))
    p[0, 0] = 1.0
    p[0, 1] = 0.0
    p[0, 2] = 0.0
    p[0, 3] = 0.0

    p[1, 0] = 0.0
    p[1, 1] = 1.0
    p[1, 2] = 0.0
    p[1, 3] = 0.0

    p[2, 0] = (R / 2.0) * (M_1 * l_1 + M_2 * L_1)
    p[2, 1] = (R / 2.0) * (M_1 * l_1 + M_2 * L_1)
    p[2, 2] = M_1 * l_1**2 + M_2 * L_1**2 + I_1
    p[2, 3] = M_2 * L_1 * l_2

    p[3, 0] = (R / 2.0) * M_2 * l_2
    p[3, 1] = (R / 2.0) * M_2 * l_2
    p[3, 2] = M_2 * L_1 * l_2
    p[3, 3] = M_2 * l_2**2 + I_2

    return p


def compute_q() -> np.ndarray:
    """4x10 matrix q (potential + input coupling terms)."""
    q = np.zeros((4, 10))

    q[0, 8] = 1.0  # q_19
    q[1, 9] = 1.0  # q_210

    q[2, 2] = (M_1 * l_1 + M_2 * L_1) * G  # q_33

    q[3, 3] = M_2 * G * l_2  # q_44

    return q


def compute_state_space() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute continuous-time A (8x8), B (8x2) and discrete-time G, H."""
    p = compute_p()
    q = compute_q()

    temp = np.linalg.solve(p, q)  # p^(-1) * q

    # Continuous-time state-space
    A = np.block(
        [
            [np.zeros((4, 4)), np.eye(4)],
            [temp[:, :8]],
        ]
    )
    B = np.block(
        [
            [np.zeros((4, 2))],
            [temp[:, 8:10]],
        ]
    )
    C = np.eye(8)
    D = np.zeros((8, 2))

    # Discrete-time with zero-order hold
    G, H, _, _, _ = cont2discrete((A, B, C, D), TS, method="zoh")

    return A, B, G, H


def get_lqr_gains() -> np.ndarray:
    """LQR gain matrix K (2x8) from STM32 firmware control.c."""
    return np.array(
        [
            [81.2695, -10.0616, -5492.4061, 18921.7098, 100.3633, 8.0376, 447.3084, 2962.7738],
            [-10.0616, 81.2695, -5492.4061, 18921.7098, 8.0376, 100.3633, 447.3084, 2962.7738],
        ]
    )
