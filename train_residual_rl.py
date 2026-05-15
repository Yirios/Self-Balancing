"""PPO learns residual correction to LQR from real car data replay."""
import csv
import numpy as np
import torch
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback

from balancing_robot.dynamics import get_lqr_gains

K = get_lqr_gains()
STATE_NAMES = ["theta_L", "theta_R", "theta_1", "theta_2",
               "theta_L_dot", "theta_R_dot", "theta_dot_1", "theta_dot_2"]
REAL_FILES = [
    "realcar/balance_steady_1.csv", "realcar/balance_steady_2.csv",
    "realcar/balance_disturb_1.csv", "realcar/balance_disturb_2.csv",
    "realcar/balance_line.csv", "realcar/balance_turn.csv",
]


class RealCarReplayEnv(gym.Env):
    """Replay real car data: state from log, action = LQR + residual, next state from log.

    The residual doesn't affect the transition (since next state was from LQR alone),
    but for small residuals this is acceptable for training a correction policy.
    """

    def __init__(self, files=REAL_FILES, max_steps=3000):
        super().__init__()
        self.observation_space = spaces.Box(-np.inf, np.inf, (8,), np.float64)
        self.action_space = spaces.Box(-5000, 5000, (2,), np.float64)
        self.max_steps = max_steps

        # Load all data
        self.all_data = []
        for fn in files:
            with open(fn) as f:
                rows = list(csv.DictReader(f))
            for r in rows:
                state = np.array([float(r[n]) for n in STATE_NAMES])
                tL = float(r["Target_theta_L"])
                tR = float(r["Target_theta_R"])
                obs = np.array([
                    state[0] - tL, state[1] - tR,
                    state[2], state[3], state[4], state[5], state[6], state[7],
                ])
                self.all_data.append({
                    "obs": obs.astype(np.float32),
                    "state": state,
                    "tL": tL, "tR": tR,
                })
        self.n_samples = len(self.all_data)
        self.idx = 0
        self.step_count = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.idx = np.random.randint(0, self.n_samples - self.max_steps - 1)
        self.step_count = 0
        d = self.all_data[self.idx]
        return d["obs"].copy(), {}

    def step(self, u_residual):
        d = self.all_data[self.idx]
        # LQR baseline
        x_ref = np.array([d["tL"], d["tR"], 0, 0, 0, 0, 0, 0])
        u_lqr = -K @ (d["state"] - x_ref)
        u_total = np.clip(u_lqr + u_residual.reshape(2), -5000, 5000)

        self.idx += 1
        next_d = self.all_data[min(self.idx, self.n_samples - 1)]
        next_obs = next_d["obs"].copy()

        # Reward: penalize tilt and angular velocities
        x = next_d["state"]
        Q_diag = np.array([0.01, 0.01, 10.0, 50.0, 0.001, 0.001, 5.0, 5.0])
        state_cost = float(x @ np.diag(Q_diag) @ x)
        control_cost = 1e-5 * float(u_total @ u_total)
        reward = -(state_cost + control_cost)

        # Encourage small residuals
        reward -= 0.001 * float(np.sum(u_residual ** 2))

        terminated = abs(x[2]) > np.deg2rad(45) or abs(x[3]) > np.deg2rad(60)
        self.step_count += 1
        truncated = self.step_count >= self.max_steps

        return next_obs, reward, terminated, truncated, {}


if __name__ == "__main__":
    print("Training PPO residual on real car LQR replay data...")

    def make_env():
        return Monitor(RealCarReplayEnv())

    env = DummyVecEnv([make_env for _ in range(4)])
    env = VecNormalize(env, norm_obs=False, norm_reward=True)

    policy_kwargs = dict(net_arch=[16], activation_fn=torch.nn.ReLU)
    model = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs,
                learning_rate=3e-4, n_steps=1024, batch_size=128,
                n_epochs=5, gamma=0.99, clip_range=0.1,
                ent_coef=0.01, verbose=1, device="cpu")

    model.learn(total_timesteps=200_000)
    model.save("ppo_residual")
    print("Saved ppo_residual.zip")

    # Check residual magnitude
    print("\nResidual statistics on real data:")
    env_test = RealCarReplayEnv()
    residuals = []
    obs, _ = env_test.reset()
    for _ in range(min(5000, env_test.n_samples - 2)):
        u_res = model.predict(obs, deterministic=True)[0]
        residuals.append(np.abs(u_res))
        obs, _, _, _, _ = env_test.step(u_res)
    residuals = np.array(residuals)
    print(f"  mean|u_res| = {np.mean(residuals):.2f}  std = {np.std(residuals):.2f}")
    print(f"  p95 = {np.percentile(residuals, 95):.2f}")
    env_test.close()
