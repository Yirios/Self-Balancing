"""Collect LQR demonstrations and pre-train a policy via behavior cloning."""
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from balancing_robot.env import BalancingRobotEnv
from balancing_robot.dynamics import get_lqr_gains, R

K = get_lqr_gains()
DEVICE = "cuda"

# --- Collect demonstrations using LQR + OU noise ---


def collect_demo(n_episodes: int = 200) -> tuple[np.ndarray, np.ndarray]:
    """Run LQR with OU noise, collect (obs, action) pairs.  LQR tracks velocity commands."""
    states, actions = [], []
    env = BalancingRobotEnv()

    for ep in range(n_episodes):
        obs, _ = env.reset()
        done = False

        ou = np.zeros(2)

        while not done:
            # LQR with velocity ref: u = -K @ (x - x_ref)
            x = obs[:8]
            v_cmd, omega_cmd = obs[8], obs[9]
            v_left = v_cmd - omega_cmd * env.WHEEL_BASE / 2.0
            v_right = v_cmd + omega_cmd * env.WHEEL_BASE / 2.0
            target_L_dot = v_left / R
            target_R_dot = v_right / R
            x_ref = np.array([0, 0, 0, 0, target_L_dot, target_R_dot, 0, 0])
            u_lqr = -K @ (x - x_ref)

            ou += -0.3 * ou * 0.005 + 0.1 * np.random.randn(2) * np.sqrt(0.005)
            noise_scale = 20.0 * max(0.0, 1.0 - ep / n_episodes)
            u = np.clip(u_lqr + ou * noise_scale, -5000, 5000)

            states.append(obs.copy())
            actions.append(u.copy())

            obs, _, terminated, truncated, _ = env.step(u)
            done = terminated or truncated

        if (ep + 1) % 50 == 0:
            print(f"  Collected {ep + 1}/{n_episodes} episodes")

    env.close()
    print(f"  Total samples: {len(states)}")
    return np.array(states, dtype=np.float32), np.array(actions, dtype=np.float32)


# --- Behavior Cloning model (matches PPO net_arch=[64, 64]) ---


class BCModel(nn.Module):
    """MLP that maps obs (10,) to action (2,), matching PPO's actor architecture."""

    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(nn.Linear(10, 32), nn.ReLU())
        self.policy_net = nn.Sequential(nn.Linear(32, 32), nn.ReLU())
        self.action_net = nn.Linear(32, 2)

    def forward(self, x):
        x = self.features(x)
        x = self.policy_net(x)
        return self.action_net(x)


def train_bc(states: np.ndarray, actions: np.ndarray, epochs: int = 50) -> BCModel:
    """Train behavior cloning model to predict actions from states."""
    model = BCModel().to(DEVICE)

    X = torch.from_numpy(states).to(DEVICE)
    Y = torch.from_numpy(actions).to(DEVICE)
    dataset = TensorDataset(X, Y)
    loader = DataLoader(dataset, batch_size=256, shuffle=True)

    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()

    for epoch in range(epochs):
        total_loss = 0.0
        for batch_states, batch_actions in loader:
            pred = model(batch_states)
            loss = loss_fn(pred, batch_actions)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total_loss += loss.item() * len(batch_states)

        if (epoch + 1) % 10 == 0:
            mean_mae = (pred - batch_actions).abs().mean().item()
            print(f"  Epoch {epoch + 1}/{epochs}: loss={total_loss / len(states):.4f}, mae={mean_mae:.2f}")

    return model


# --- Load BC weights into PPO ---


def load_bc_into_ppo(bc_model: BCModel, ppo_model) -> None:
    """Copy BC model weights into PPO's policy (policy_net[0,2], action_net)."""
    policy = ppo_model.policy
    ppo_policy_net = policy.mlp_extractor.policy_net

    def _copy(dst, src):
        dst.data.copy_(src.data.to(dst.device))

    _copy(ppo_policy_net[0].weight, bc_model.features[0].weight)
    _copy(ppo_policy_net[0].bias, bc_model.features[0].bias)
    _copy(ppo_policy_net[2].weight, bc_model.policy_net[0].weight)
    _copy(ppo_policy_net[2].bias, bc_model.policy_net[0].bias)
    _copy(policy.action_net.weight, bc_model.action_net.weight)
    _copy(policy.action_net.bias, bc_model.action_net.bias)


# --- Evaluate BC model ---


def eval_bc(bc_model: BCModel, n_episodes: int = 20):
    """Test BC model without exploration noise."""
    env = BalancingRobotEnv()
    lengths = []
    for _ in range(n_episodes):
        obs, _ = env.reset()
        done = False
        steps = 0
        while not done:
            with torch.no_grad():
                inp = torch.from_numpy(obs.astype(np.float32)).to(DEVICE)
                u = bc_model(inp).cpu().numpy()
            obs, _, terminated, truncated, _ = env.step(np.clip(u, -500, 500))
            done = terminated or truncated
            steps += 1
        lengths.append(steps)
    env.close()
    print(f"  BC eval: mean episode length = {np.mean(lengths):.0f} / max = {np.max(lengths)}")


if __name__ == "__main__":
    print("Collecting LQR demonstrations...")
    states, actions = collect_demo(n_episodes=200)

    print("\nTraining behavior cloning model...")
    bc_model = train_bc(states, actions, epochs=50)

    print("\nEvaluating BC model...")
    eval_bc(bc_model)

    # Save BC model for later fine-tuning
    torch.save(bc_model.state_dict(), "bc_model.pt")
    print("BC model saved as bc_model.pt")

    print("\nInitializing PPO with BC weights...")
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
    from stable_baselines3.common.monitor import Monitor

    def make_env():
        return Monitor(BalancingRobotEnv())

    env = DummyVecEnv([make_env for _ in range(4)])
    env = VecNormalize(env, norm_obs=False, norm_reward=True)

    policy_kwargs = dict(net_arch=[32, 32], activation_fn=torch.nn.ReLU)
    model = PPO(
        "MlpPolicy",
        env,
        policy_kwargs=policy_kwargs,
        verbose=0,
        device="cpu",
    )

    load_bc_into_ppo(bc_model, model)

    # Save pretrained model and normalization stats
    model.save("ppo_balance_bot_pretrained")
    env.save("vec_normalize.pkl")
    print("Pretrained model saved as ppo_balance_bot_pretrained.zip")
    env.close()
