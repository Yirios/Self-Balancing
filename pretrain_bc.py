"""Pre-train BC policy from real car data via supervised learning (MSE + Adam).

Architecture: 8→16→2, no bias, ReLU.
Normalizes inputs/outputs for stable gradient-based training.
"""
import csv
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from balancing_robot.env import BalancingRobotEnv

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

STATE_NAMES = [
    "theta_L", "theta_R", "theta_1", "theta_2",
    "theta_L_dot", "theta_R_dot", "theta_dot_1", "theta_dot_2",
]
REAL_FILES = [
    "data/realcar/balance_steady_1.csv", "data/realcar/balance_steady_2.csv",
    "data/realcar/balance_disturb_1.csv", "data/realcar/balance_disturb_2.csv",
    "data/realcar/balance_line.csv", "data/realcar/balance_turn.csv",
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


class BCModel(nn.Module):
    """MLP 8→16→2, no bias, ReLU."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(8, 16, bias=False),
            nn.ReLU(),
            nn.Linear(16, 2, bias=False),
        )

    def forward(self, x):
        return self.net(x)


def eval_bc(model, x_mean, x_std, y_mean, y_std, n_episodes=50):
    """Evaluate BC model in the data-driven environment."""
    env = BalancingRobotEnv(data_driven=True)
    lengths = []
    for _ in range(n_episodes):
        obs, _ = env.reset()
        done = False
        steps = 0
        while not done:
            with torch.no_grad():
                inp = torch.from_numpy(obs.astype(np.float32)).to(DEVICE)
                inp_s = (inp - x_mean) / x_std
                act_s = model(inp_s)
                u = (act_s * y_std + y_mean).cpu().numpy()
            obs, _, terminated, truncated, _ = env.step(np.clip(u, -5000, 5000))
            done = terminated or truncated
            steps += 1
        lengths.append(steps)
    env.close()
    survived = sum(1 for l in lengths if l >= 1999)
    print(f"  BC eval: mean={np.mean(lengths):.0f} max={np.max(lengths)} "
          f"survived={survived}/{n_episodes}")
    return lengths


def load_bc_full(model_path="models/bc_model.pt"):
    """Load BC model + standardization params."""
    ckpt = torch.load(model_path, map_location="cpu")
    model = BCModel()
    model.load_state_dict(ckpt["model_state"])
    model.eval()
    x_mean = ckpt["x_mean"]
    x_std = ckpt["x_std"]
    y_mean = ckpt["y_mean"]
    y_std = ckpt["y_std"]
    return model, x_mean, x_std, y_mean, y_std


def get_bc_weights(bc_model):
    """Extract W1 (16×8) and W2 (2×16) from BC model (raw, without normalization)."""
    return (bc_model.net[0].weight.data.clone(),
            bc_model.net[2].weight.data.clone())


if __name__ == "__main__":
    print("Loading real car data...")
    X_raw, Y_raw = load_real_data()
    X = torch.from_numpy(X_raw).to(DEVICE)
    Y = torch.from_numpy(Y_raw).to(DEVICE)

    # Standardize inputs and outputs for stable gradient-based training
    x_mean = X.mean(dim=0, keepdim=True)
    x_std = X.std(dim=0, keepdim=True) + 1e-8
    y_mean = Y.mean(dim=0, keepdim=True)
    y_std = Y.std(dim=0, keepdim=True) + 1e-8
    print(f"  x_std: {x_std.cpu().numpy().flatten()}")
    print(f"  y_std: {y_std.cpu().numpy().flatten()}")

    X_s = (X - x_mean) / x_std
    Y_s = (Y - y_mean) / y_std

    # Train/val split — shuffle since data spans multiple episodes
    n = len(X_s)
    idx = torch.randperm(n)
    n_train = int(n * 0.85)
    X_tr, Y_tr = X_s[idx[:n_train]], Y_s[idx[:n_train]]
    X_val, Y_val = X_s[idx[n_train:]], Y_s[idx[n_train:]]

    model = BCModel().to(DEVICE)
    # Small random init — no bias, ReLU needs diverse initial projections
    nn.init.kaiming_normal_(model.net[0].weight, nonlinearity="relu")
    nn.init.kaiming_normal_(model.net[2].weight, nonlinearity="linear")

    loader = DataLoader(TensorDataset(X_tr, Y_tr), batch_size=256, shuffle=True)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="min", factor=0.5, patience=10)
    loss_fn = nn.MSELoss()

    print(f"\nTraining 8→16→2 (no bias, ReLU) on {n_train} standardized samples...")
    best_val = float("inf")
    best_state = None
    for epoch in range(200):
        model.train()
        train_loss = 0.0
        for bx, by in loader:
            pred = model(bx)
            loss = loss_fn(pred, by)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * len(bx)
        train_loss /= n_train

        model.eval()
        with torch.no_grad():
            val_loss = loss_fn(model(X_val), Y_val).item()
            val_pred = model(X_val) * y_std + y_mean
            val_true = Y_val * y_std + y_mean
            val_mae = (val_pred - val_true).abs().mean().item()

        scheduler.step(val_loss)

        if val_loss < best_val:
            best_val = val_loss
            best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}

        if (epoch + 1) % 20 == 0:
            print(f"  epoch {epoch+1:3d}: train_loss={train_loss:.4f} "
                  f"val_loss={val_loss:.4f} val_mae={val_mae:.1f} "
                  f"lr={optimizer.param_groups[0]['lr']:.1e}")

    model.load_state_dict(best_state)
    print(f"  best val_loss={best_val:.4f}")

    # Save model + standardization params
    save_dict = {
        "model_state": model.state_dict(),
        "x_mean": x_mean.cpu(), "x_std": x_std.cpu(),
        "y_mean": y_mean.cpu(), "y_std": y_std.cpu(),
    }
    torch.save(save_dict, "models/bc_model.pt")

    # Evaluate
    print("\nEvaluating...")
    eval_bc(model, x_mean, x_std, y_mean, y_std)
    print("BC model saved as models/bc_model.pt")
