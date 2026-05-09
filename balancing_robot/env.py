import numpy as np
import gymnasium
from gymnasium import spaces

from .dynamics import compute_state_space, get_lqr_gains, R, TS

_A, _B, _G, _H = compute_state_space()
_K = get_lqr_gains()


class BalancingRobotEnv(gymnasium.Env):
    """
    Two-wheeled self-balancing robot with velocity commands.

    Obs (10D): [theta_L, theta_R, theta_1, theta_2,
                theta_L_dot, theta_R_dot, theta_1_dot, theta_2_dot,
                v_cmd, omega_cmd]
    Actions (2D): [u_L, u_R] — wheel angular accelerations (rad/s²)
    """

    WHEEL_BASE = 0.16  # meters between wheels

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 200}

    def __init__(
        self,
        render_mode: str | None = None,
        max_steps: int = 1400,
        bc_beta: float = 0.0,
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

        self.G = _G
        self.H = _H
        self.K = _K

        self.state = np.zeros(8)
        self.step_count = 0

        # Velocity command (set in reset, constant per episode)
        self.v_cmd = 0.0   # target forward speed (m/s)
        self.omega_cmd = 0.0  # target turn rate (rad/s)

        # Reward weights
        self.Q_diag = np.array([0.01, 0.01, 10.0, 50.0, 0.001, 0.001, 5.0, 5.0])
        self.R_weight = 1e-5
        self.w_vel = 5.0   # velocity tracking weight

        # Rendering
        self.window = None
        self.clock = None

    def reset(self, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)

        self.step_count = 0

        # Global position
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.x_hist = [0.0]
        self.y_hist = [0.0]

        # Sample velocity command
        if options is not None and "v_cmd" in options:
            self.v_cmd = options["v_cmd"]
            self.omega_cmd = options.get("omega_cmd", 0.0)
        else:
            self.v_cmd = float(self.np_random.uniform(-0.3, 0.3))
            # Turn rate: 0 = straight ~30% of the time, else random curve
            if self.np_random.random() < 0.3:
                self.omega_cmd = 0.0
            else:
                self.omega_cmd = float(self.np_random.uniform(-1.5, 1.5))

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
        return np.concatenate([self.state, [self.v_cmd, self.omega_cmd]])

    def step(self, action: np.ndarray):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        u = action.reshape(2)

        # Discrete-time state update
        self.state = self.G @ self.state + self.H @ u

        x = self.state

        # Quadratic state + control cost
        state_cost = float(x @ np.diag(self.Q_diag) @ x)
        control_cost = self.R_weight * float(u @ u)
        reward = -(state_cost + control_cost)

        # BC regularization
        if self.bc_beta > 0:
            u_lqr = -self.K @ x
            reward -= self.bc_beta * float(np.sum((u - u_lqr) ** 2))

        # Velocity tracking penalty
        v_actual = (R * x[4] + R * x[5]) / 2.0
        omega_actual = (R * x[5] - R * x[4]) / self.WHEEL_BASE
        vel_err = (v_actual - self.v_cmd) ** 2 + (omega_actual - self.omega_cmd) ** 2
        reward -= self.w_vel * vel_err

        # Global position update
        v_fwd = (R * x[4] + R * x[5]) / 2.0
        omega = (R * x[5] - R * x[4]) / self.WHEEL_BASE
        self.yaw += omega * TS
        self.x += v_fwd * np.cos(self.yaw) * TS
        self.y += v_fwd * np.sin(self.yaw) * TS
        self.x_hist.append(self.x)
        self.y_hist.append(self.y)

        # Termination
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
                "v_actual": v_actual,
                "omega_actual": omega_actual,
                "v_cmd": self.v_cmd,
                "omega_cmd": self.omega_cmd,
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

        # Wheels
        wheel_radius = int(R * scale)
        left_wheel_cx = cx - int(0.08 * scale)
        right_wheel_cx = cx + int(0.08 * scale)
        pygame.draw.circle(self.window, (50, 50, 50), (left_wheel_cx, cy), wheel_radius)
        pygame.draw.circle(self.window, (50, 50, 50), (right_wheel_cx, cy), wheel_radius)

        pygame.draw.line(
            self.window,
            (0, 0, 0),
            (left_wheel_cx - wheel_radius, cy),
            (right_wheel_cx + wheel_radius, cy),
            3,
        )

        # Body
        body_len = int(L_1 * scale)
        body_top_x = cx + int(body_len * np.sin(theta_1))
        body_top_y = cy - int(body_len * np.cos(theta_1))
        pygame.draw.line(
            self.window, (200, 50, 50), (cx, cy), (body_top_x, body_top_y), 6
        )

        # Pendulum
        pendulum_len = int(L_2 * scale)
        pendulum_angle = theta_1 + theta_2_val
        pendulum_top_x = body_top_x + int(pendulum_len * np.sin(pendulum_angle))
        pendulum_top_y = body_top_y - int(pendulum_len * np.cos(pendulum_angle))
        pygame.draw.line(
            self.window,
            (50, 50, 200),
            (body_top_x, body_top_y),
            (pendulum_top_x, pendulum_top_y),
            4,
        )

        pygame.draw.circle(self.window, (200, 50, 50), (body_top_x, body_top_y), 8)
        pygame.draw.circle(
            self.window, (50, 50, 200), (pendulum_top_x, pendulum_top_y), 6
        )

        pygame.display.flip()
        self.clock.tick(200)

    def close(self):
        if self.window is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.window = None
            self.clock = None
