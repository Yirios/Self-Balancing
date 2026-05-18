"""Pre-train a policy via behavior cloning — sim demos + real data fine-tune."""
import csv
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from balancing_robot.env import BalancingRobotEnv
from balancing_robot.dynamics import get_lqr_gains

K = get_lqr_gains()
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

STATE_NAMES = [
    "theta_L", "theta_R", "theta_1", "theta_2",
    "theta_L_dot", "theta_R_dot", "theta_dot_1", "theta_dot_2",
]
REAL_FILES = [
    "realcar/balance_steady_1.csv", "realcar/balance_steady_2.csv",
    "realcar/balance_disturb_1.csv", "realcar/balance_disturb_2.csv",
    "realcar/balance_line.csv", "realcar/balance_turn.csv",
]


def load_real_data():
    """obs (8D) = [thL-targetL, thR-targetR, th1, th2, thdotL, thdotR, thdot1, thdot2]."""
    obs_list, act_list = [], []
    for fn in REAL_FILES:
        with open(fn) as f:
            rows = list(csv.DictReader(f))
        for r in rows:
            state = np.array([float(r[n]) for n in STATE_NAMES], dtype=np.float32)
            target_L = float(r["Target_theta_L"])
            target_R = float(r["Target_theta_R"])
            obs_list.append(np.array([
                state[0] - target_L, state[1] - target_R,
                state[2], state[3], state[4], state[5], state[6], state[7],
            ], dtype=np.float32))
            act_list.append([float(r["u_L"]), float(r["u_R"])])
    print(f"Real data: {len(obs_list)} samples from {len(REAL_FILES)} files")
    return np.array(obs_list, dtype=np.float32), np.array(act_list, dtype=np.float32)


def collect_sim_demos(n_episodes: int = 200) -> tuple[np.ndarray, np.ndarray]:
    """LQR + OU noise in simulation."""
    obs_list, act_list = [], []
    env = BalancingRobotEnv(inject_noise=True, domain_rand_scale=0.0,
                            pendulum_disturb_std=0.8, data_driven=True)

    for ep in range(n_episodes):
        obs, _ = env.reset()
        done = False
        ou = np.zeros(2)

        while not done:
            # LQR with position reference
            x = env.unwrapped.state
            x_ref = np.array([env.unwrapped.target_theta_L,
                              env.unwrapped.target_theta_R, 0, 0, 0, 0, 0, 0])
            u_lqr = -K @ (x - x_ref)

            ou += -0.3 * ou * 0.01 + 0.1 * np.random.randn(2) * np.sqrt(0.01)
            noise_scale = 20.0 * max(0.0, 1.0 - ep / n_episodes)
            u = np.clip(u_lqr + ou * noise_scale, -5000, 5000)

            obs_list.append(obs.copy())
            act_list.append(np.clip(u_lqr, -5000, 5000))

            obs, _, terminated, truncated, _ = env.step(u)
            done = terminated or truncated

        if (ep + 1) % 50 == 0:
            print(f"  {ep + 1}/{n_episodes} episodes")

    env.close()
    print(f"Sim demos: {len(obs_list)} samples")
    return np.array(obs_list, dtype=np.float32), np.array(act_list, dtype=np.float32)


class BCModel(nn.Module):
    """MLP 8→16→2."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(8, 16, bias=True),
            nn.ReLU(),
            nn.Linear(16, 2, bias=True),
        )

    def forward(self, x):
        return self.net(x)


def train_bc(model, X, Y, epochs, lr=1e-3, label="", smooth_coef=0.0, val_split=0.2):
    # Train/val split (sequential, not shuffled — sim data is in episode order)
    n_val = int(len(X) * val_split)
    X_train, Y_train = X[:-n_val], Y[:-n_val]
    X_val, Y_val = X[-n_val:], Y[-n_val:]

    X_train_t = torch.from_numpy(X_train).to(DEVICE)
    Y_train_t = torch.from_numpy(Y_train).to(DEVICE)
    X_val_t = torch.from_numpy(X_val).to(DEVICE)
    Y_val_t = torch.from_numpy(Y_val).to(DEVICE)

    loader = DataLoader(TensorDataset(X_train_t, Y_train_t), batch_size=256, shuffle=True)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    best_val = float('inf')
    best_state = None

    # Fixed consecutive pairs for smoothness (subset of training data, unshuffled)
    n_smooth = min(50000, len(X_train) - 1)
    step = max(1, len(X_train) // n_smooth)
    X_smooth = torch.from_numpy(X_train[::step][:n_smooth]).to(DEVICE)
    X_smooth_next = torch.from_numpy(X_train[1::step][:n_smooth]).to(DEVICE)

    for epoch in range(epochs):
        model.train()
        train_loss = 0.0
        smooth_loss_val = 0.0
        for bx, by in loader:
            pred = model(bx)
            loss = loss_fn(pred, by)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * len(bx)
        # Smoothness: penalize large action changes between consecutive states
        if smooth_coef > 0:
            smooth_loss = smooth_coef * loss_fn(model(X_smooth), model(X_smooth_next))
            optimizer.zero_grad()
            smooth_loss.backward()
            optimizer.step()
            smooth_loss_val = smooth_loss.item()

        # Validation
        model.eval()
        with torch.no_grad():
            val_loss = loss_fn(model(X_val_t), Y_val_t).item()
            val_mae = (model(X_val_t) - Y_val_t).abs().mean().item()

        if val_loss < best_val:
            best_val = val_loss
            best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}

        if (epoch + 1) % 5 == 0:
            s = f"  {label} epoch {epoch + 1}/{epochs}: train={train_loss/len(X_train):.1f} val={val_loss:.1f} val_mae={val_mae:.2f}"
            if smooth_coef > 0:
                s += f" smooth={smooth_loss_val:.1f}"
            print(s)

    if best_state is not None:
        model.load_state_dict(best_state)
        print(f"  {label} best val_loss={best_val:.1f}")


def load_bc_into_ppo(bc_model: BCModel, ppo_model) -> None:
    ppo_pn = ppo_model.policy.mlp_extractor.policy_net

    def _copy(dst, src):
        dst.data.copy_(src.data.to(dst.device))

    # BC: Linear(8,16) → ReLU → Linear(16,2)
    # PPO: policy_net[0]=Linear(8,32), policy_net[2]=Linear(32,32),
    #      action_net=Linear(32,2)
    W1 = bc_model.net[0].weight.data  # 16×8
    b1 = bc_model.net[0].bias.data    # 16
    W2 = bc_model.net[2].weight.data  # 2×16
    b2 = bc_model.net[2].bias.data    # 2
    H = 16  # BC hidden dim

    # policy_net[0] (8→32): copy BC W1 into top H rows, zero rest
    ppo_pn[0].weight.data.zero_()
    ppo_pn[0].weight.data[:H] = W1
    ppo_pn[0].bias.data.zero_()
    ppo_pn[0].bias.data[:H] = b1

    # policy_net[2] (32→32): identity for first H dims, zero rest
    nn.init.eye_(ppo_pn[2].weight)
    nn.init.zeros_(ppo_pn[2].bias)

    # action_net (32→2): copy BC W2 into first H columns, set bias
    ppo_model.policy.action_net.weight.data.zero_()
    ppo_model.policy.action_net.weight.data[:, :H] = W2
    ppo_model.policy.action_net.bias.data.zero_()
    ppo_model.policy.action_net.bias.data[:] = b2


def eval_bc(bc_model: BCModel, n_episodes: int = 20):
    env = BalancingRobotEnv(data_driven=True)
    lengths = []
    for _ in range(n_episodes):
        obs, _ = env.reset()
        done = False
        steps = 0
        while not done:
            with torch.no_grad():
                inp = torch.from_numpy(obs.astype(np.float32)).to(DEVICE)
                u = bc_model(inp).cpu().numpy()
            obs, _, terminated, truncated, _ = env.step(np.clip(u, -5000, 5000))
            done = terminated or truncated
            steps += 1
        lengths.append(steps)
    env.close()
    mean_len = np.mean(lengths)
    print(f"  BC eval: mean ep_len = {mean_len:.0f} / max = {np.max(lengths)}")
    return mean_len


if __name__ == "__main__":
    print("Phase 1: Simulated LQR+OU demos")
    sim_obs, sim_act = collect_sim_demos(200)

    model = BCModel().to(DEVICE)
    train_bc(model, sim_obs, sim_act, epochs=50, lr=1e-3, label="sim",
             smooth_coef=0.0, val_split=0.1)

    print("\nPhase 2: Real car data (fine-tune)")
    real_obs, real_act = load_real_data()
    X_mix = np.concatenate([sim_obs, real_obs]).astype(np.float32)
    Y_mix = np.concatenate([sim_act, real_act]).astype(np.float32)
    train_bc(model, X_mix, Y_mix, epochs=15, lr=3e-4, label="mix",
             smooth_coef=0.0, val_split=0.05)

    print("\nEvaluating...")
    eval_bc(model)
    torch.save(model.state_dict(), "bc_model.pt")
    print("BC model saved as bc_model.pt")
