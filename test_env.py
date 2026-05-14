"""Validation tests for the BalancingRobot Gymnasium environment."""

import numpy as np
import gymnasium

import balancing_robot  # triggers registration
from balancing_robot.dynamics import get_lqr_gains

_K = get_lqr_gains()


def test_make_env():
    env = gymnasium.make("BalancingRobot-v0")
    assert env is not None
    assert env.observation_space.shape == (10,)
    assert env.action_space.shape == (2,)
    env.close()


def test_reset():
    env = gymnasium.make("BalancingRobot-v0")
    obs, info = env.reset()
    assert obs.shape == (10,)
    assert isinstance(info, dict)
    env.close()


def test_step():
    env = gymnasium.make("BalancingRobot-v0")
    env.reset(seed=42)
    obs, reward, terminated, truncated, info = env.step(np.array([0.0, 0.0]))
    assert obs.shape == (10,)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert isinstance(info, dict)
    env.close()


def test_termination_on_body_tilt():
    env = gymnasium.make("BalancingRobot-v0")
    env.reset(seed=42)
    env.unwrapped.state = np.array([0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0])
    obs, reward, terminated, truncated, info = env.step(np.array([0.0, 0.0]))
    assert terminated
    env.close()


def test_termination_on_pendulum_angle():
    env = gymnasium.make("BalancingRobot-v0")
    env.reset(seed=42)
    env.unwrapped.state = np.array([0.0, 0.0, 0.0, 1.1, 0.0, 0.0, 0.0, 0.0])
    obs, reward, terminated, truncated, info = env.step(np.array([0.0, 0.0]))
    assert terminated
    env.close()


def test_truncation_at_max_steps():
    env = gymnasium.make("BalancingRobot-v0", max_episode_steps=10)
    env.reset(seed=42)
    truncated = False
    for _ in range(10):
        _, _, _, truncated, _ = env.step(np.array([0.0, 0.0]))
    assert truncated
    env.close()


def test_seed_reproducibility():
    for seed in [0, 42, 123]:
        env1 = gymnasium.make("BalancingRobot-v0")
        env2 = gymnasium.make("BalancingRobot-v0")
        obs1, _ = env1.reset(seed=seed)
        obs2, _ = env2.reset(seed=seed)
        assert np.allclose(obs1, obs2)

        for _ in range(10):
            action = np.array([1.0, -0.5])
            obs1, _, _, _, _ = env1.step(action)
            obs2, _, _, _, _ = env2.step(action)
            assert np.allclose(obs1, obs2)

        env1.close()
        env2.close()


def test_lqr_stabilizes():
    """LQR with position reference stabilizes the system."""
    env = gymnasium.make("BalancingRobot-v0")

    obs, _ = env.reset(seed=42)
    env.unwrapped.state = np.zeros(8)
    env.unwrapped.state[2] = 0.05
    env.unwrapped.target_theta_L = 0.0
    env.unwrapped.target_theta_R = 0.0
    obs = env.unwrapped._get_obs()

    terminated = False
    max_theta_1 = 0.0

    while not terminated:
        x = obs[:8]
        x_ref = np.array([obs[8], obs[9], 0, 0, 0, 0, 0, 0])
        u = -_K @ (x - x_ref)
        obs, _, terminated, truncated, info = env.step(u)
        max_theta_1 = max(max_theta_1, abs(info["theta_1"]))
        if truncated:
            break

    assert not terminated, (
        f"LQR failed: terminated with theta_1={info['theta_1']:.3f}, "
        f"max(|theta_1|)={max_theta_1:.3f}"
    )
    assert truncated, "LQR should survive full episode"
    print(f"  LQR: max(|theta_1|)={np.rad2deg(max_theta_1):.1f}deg")
    env.close()


def test_action_clipping():
    env = gymnasium.make("BalancingRobot-v0")
    env.reset(seed=42)
    obs, reward, terminated, truncated, info = env.step(np.array([10000.0, -10000.0]))
    assert obs is not None
    env.close()


if __name__ == "__main__":
    tests = [
        test_make_env,
        test_reset,
        test_step,
        test_termination_on_body_tilt,
        test_termination_on_pendulum_angle,
        test_truncation_at_max_steps,
        test_seed_reproducibility,
        test_lqr_stabilizes,
        test_action_clipping,
    ]
    for test_fn in tests:
        try:
            test_fn()
            print(f"  PASS  {test_fn.__name__}")
        except Exception as e:
            print(f"  FAIL  {test_fn.__name__}: {e}")
