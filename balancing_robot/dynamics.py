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

TS = 0.01  # 10ms, matching LQR control interval (not 5ms ISR)


def compute_state_space_with_params(
    m1: float = M_1,
    m2: float = M_2,
    r: float = R,
    l1: float = L_1,
    l2: float = L_2,
    ts: float = TS,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute G, H with custom physical parameters (for domain randomization)."""
    _l1 = l1 / 2.0
    _l2 = l2 / 2.0
    _I1 = (1.0 / 12.0) * m1 * l1**2
    _I2 = (1.0 / 12.0) * m2 * l2**2

    p = np.zeros((4, 4))
    p[0, 0] = 1.0
    p[1, 1] = 1.0
    p[2, 0] = (r / 2.0) * (m1 * _l1 + m2 * l1)
    p[2, 1] = (r / 2.0) * (m1 * _l1 + m2 * l1)
    p[2, 2] = m1 * _l1**2 + m2 * l1**2 + _I1
    p[2, 3] = m2 * l1 * _l2
    p[3, 0] = (r / 2.0) * m2 * _l2
    p[3, 1] = (r / 2.0) * m2 * _l2
    p[3, 2] = m2 * l1 * _l2
    p[3, 3] = m2 * _l2**2 + _I2

    q = np.zeros((4, 10))
    q[0, 8] = 1.0
    q[1, 9] = 1.0
    q[2, 2] = (m1 * _l1 + m2 * l1) * G
    q[3, 3] = m2 * G * _l2

    temp = np.linalg.solve(p, q)
    A = np.block([[np.zeros((4, 4)), np.eye(4)], [temp[:, :8]]])
    B = np.block([[np.zeros((4, 2))], [temp[:, 8:10]]])
    Gd, Hd, _, _, _ = cont2discrete((A, B, np.eye(8), np.zeros((8, 2))), ts, method="zoh")
    return A, B, Gd, Hd


def compute_state_space() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute continuous-time A (8x8), B (8x2) and discrete-time G, H."""
    return compute_state_space_with_params()


def get_lqr_gains() -> np.ndarray:
    """LQR gain matrix K (2x8) from STM32 firmware control.c."""
    return np.array(
        [
            [81.2695, -10.0616, -5492.4061, 18921.7098, 100.3633, 8.0376, 447.3084, 2962.7738],
            [-10.0616, 81.2695, -5492.4061, 18921.7098, 8.0376, 100.3633, 447.3084, 2962.7738],
        ]
    )
