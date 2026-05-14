import numpy as np
import gymnasium
from gymnasium import spaces

from .dynamics import (
    M_1, M_2, R, L_1, L_2, TS,
    compute_state_space,
    compute_state_space_with_params,
    get_lqr_gains,
)

_A, _B, _G, _H = compute_state_space()
_K = get_lqr_gains()

# Sensor noise std per 10ms step, calibrated from real car steady-state data
NOISE_STD = np.array(
    [0.0020, 0.0019, 0.0010, 0.0022, 0.0912, 0.0836, 0.0270, 0.1058]
)


class BalancingRobotEnv(gymnasium.Env):
    """
    Two-wheeled self-balancing robot with position targets.

    Obs (10D): [theta_L, theta_R, theta_1, theta_2,
                theta_L_dot, theta_R_dot, theta_1_dot, theta_2_dot,
                target_theta_L, target_theta_R]
    Actions (2D): [u_L, u_R] — wheel angular accelerations (rad/s²)
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 200}

    def __init__(
        self,
        render_mode: str | None = None,
        max_steps: int = 3000,
        bc_beta: float = 0.0,
        inject_noise: bool = False,
        domain_rand_scale: float = 0.0,
    ):
        super().__init__()
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(10,), dtype=np.float64
        )
        self.action_space = spaces.Box(
            low=-5000.0, high=5000.0, shape=(2,), dtype=np.float64
        )

        self.max_steps = max_steps
        self.render_mode = render_mode
        self.bc_beta = bc_beta
        self.inject_noise = inject_noise
        self.domain_rand_scale = domain_rand_scale

        self.G = _G
        self.H = _H
        self.K = _K
        self.WHEEL_BASE = 0.16

        self.state = np.zeros(8)
        self.step_count = 0

        self.target_theta_L = 0.0
        self.target_theta_R = 0.0

        # Reward: Q_diag heavily weights tilt (theta_1, theta_2) and angular velocities
        self.Q_diag = np.array([0.01, 0.01, 10.0, 50.0, 0.001, 0.001, 5.0, 5.0])
        self.R_weight = 1e-5
        self.w_pos = 0.1  # position tracking weight

        self.window = None
        self.clock = None

    def reset(self, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)

        self.step_count = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.x_hist = [0.0]
        self.y_hist = [0.0]

        # Domain randomization
        if self.domain_rand_scale > 0:
            s = self.domain_rand_scale
            m1 = M_1 * (1.0 + self.np_random.uniform(-s, s))
            m2 = M_2 * (1.0 + self.np_random.uniform(-s, s))
            radius = R * (1.0 + self.np_random.uniform(-s, s))
            l1 = L_1 * (1.0 + self.np_random.uniform(-s, s))
            l2 = L_2 * (1.0 + self.np_random.uniform(-s, s))
            self.WHEEL_BASE = 0.16 * (1.0 + self.np_random.uniform(-s, s))
            _, _, self.G, self.H = compute_state_space_with_params(
                m1=m1, m2=m2, r=radius, l1=l1, l2=l2, ts=TS
            )
        else:
            self.G = _G
            self.H = _H
            self.WHEEL_BASE = 0.16

        # Position targets (rad).  Target_theta_L/R are the LQR position reference.
        # During free balance: both near 0.  During driving: they diverge.
        if options is not None and "target_theta_L" in options:
            self.target_theta_L = options["target_theta_L"]
            self.target_theta_R = options.get("target_theta_R", self.target_theta_L)
        else:
            # 30% chance: pure balance (both 0)
            # 50% chance: straight line (same target, non-zero)
            # 20% chance: turn (different targets)
            r = self.np_random.random()
            if r < 0.3:
                self.target_theta_L = 0.0
                self.target_theta_R = 0.0
            elif r < 0.8:
                t = float(self.np_random.uniform(-20, 20))
                self.target_theta_L = t
                self.target_theta_R = t
            else:
                self.target_theta_L = float(self.np_random.uniform(-20, 20))
                self.target_theta_R = float(self.np_random.uniform(-20, 20))

        # Initial state
        if options is not None and options.get("matlab_ic", False):
            self.state = np.array(
                [0.0, 0.0, -0.1745, 0.1745, 0.0, 0.0, 0.0, 0.0]
            )
        else:
            self.state = np.zeros(8)
            self.state[2] = self.np_random.uniform(-0.1745, 0.1745)
            self.state[3] = self.np_random.uniform(-0.1745, 0.1745)
            self.state[4:8] = self.np_random.uniform(-0.1, 0.1, size=4)

        return self._get_obs(), {}

    def _get_obs(self):
        return np.concatenate([self.state, [self.target_theta_L, self.target_theta_R]])

    def step(self, action: np.ndarray):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        u = action.reshape(2)

        self.state = self.G @ self.state + self.H @ u

        if self.inject_noise:
            self.state += self.np_random.normal(0, NOISE_STD).astype(np.float64)

        x = self.state

        # Quadratic state + control cost
        state_cost = float(x @ np.diag(self.Q_diag) @ x)
        control_cost = self.R_weight * float(u @ u)
        reward = -(state_cost + control_cost)

        if self.bc_beta > 0:
            x_ref = np.array(
                [self.target_theta_L, self.target_theta_R, 0, 0, 0, 0, 0, 0]
            )
            u_lqr = -self.K @ (x - x_ref)
            reward -= self.bc_beta * float(np.sum((u - u_lqr) ** 2))

        # Position tracking penalty
        pos_err = (x[0] - self.target_theta_L) ** 2 + (x[1] - self.target_theta_R) ** 2
        reward -= self.w_pos * pos_err

        # Global position (for 3D visualization)
        v_fwd = (R * x[4] + R * x[5]) / 2.0
        omega = (R * x[5] - R * x[4]) / self.WHEEL_BASE
        self.yaw += omega * TS
        self.x += v_fwd * np.cos(self.yaw) * TS
        self.y += v_fwd * np.sin(self.yaw) * TS
        self.x_hist.append(self.x)
        self.y_hist.append(self.y)

        theta_1 = x[2]
        theta_2 = x[3]
        terminated = bool(abs(theta_1) > np.deg2rad(45) or abs(theta_2) > np.deg2rad(60))

        self.step_count += 1
        truncated = self.step_count >= self.max_steps

        return (
            self._get_obs(),
            reward,
            terminated,
            truncated,
            {
                "theta_1": theta_1,
                "theta_2": theta_2,
                "cost": -reward,
                "target_theta_L": self.target_theta_L,
                "target_theta_R": self.target_theta_R,
            },
        )

    def render(self):
        if self.render_mode is None:
            return
        import pygame
        if self.window is None:
            pygame.init()
            self.window = pygame.display.set_mode((600, 400))
            self.clock = pygame.time.Clock()
        self.window.fill((255, 255, 255))
        scale = 300
        cx, cy = 300, 250
        theta_1 = self.state[2]
        theta_2_val = self.state[3]
        wheel_radius = int(R * scale)
        left_cx = cx - int(0.08 * scale)
        right_cx = cx + int(0.08 * scale)
        pygame.draw.circle(self.window, (50, 50, 50), (left_cx, cy), wheel_radius)
        pygame.draw.circle(self.window, (50, 50, 50), (right_cx, cy), wheel_radius)
        pygame.draw.line(self.window, (0, 0, 0), (left_cx - wheel_radius, cy), (right_cx + wheel_radius, cy), 3)
        body_len = int(L_1 * scale)
        btx = cx + int(body_len * np.sin(theta_1))
        bty = cy - int(body_len * np.cos(theta_1))
        pygame.draw.line(self.window, (200, 50, 50), (cx, cy), (btx, bty), 6)
        pend_len = int(L_2 * scale)
        pa = theta_1 + theta_2_val
        ptx = btx + int(pend_len * np.sin(pa))
        pty = bty - int(pend_len * np.cos(pa))
        pygame.draw.line(self.window, (50, 50, 200), (btx, bty), (ptx, pty), 4)
        pygame.draw.circle(self.window, (200, 50, 50), (btx, bty), 8)
        pygame.draw.circle(self.window, (50, 50, 200), (ptx, pty), 6)
        pygame.display.flip()
        self.clock.tick(200)

    def close(self):
        if self.window is not None:
            import pygame
            pygame.display.quit()
            pygame.quit()
            self.window = None
            self.clock = None
