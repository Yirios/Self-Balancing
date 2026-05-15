"""Train MLP residual on top of frozen linear OLS model."""
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from balancing_robot.env import BalancingRobotEnv
from balancing_robot.dynamics import get_lqr_gains
from pretrain_bc import collect_sim_demos, load_real_data, eval_bc, DEVICE

K = get_lqr_gains()


class ResidualModel(nn.Module):
    """y = W_lin @ x + alpha * mlp(x).  W_lin frozen (OLS → LQR), mlp learns residuals."""

    def __init__(self, W_lin, hidden=16):
        super().__init__()
        self.W_lin = nn.Parameter(torch.from_numpy(W_lin.T.astype(np.float32)),
                                  requires_grad=False)
        self.b_lin = nn.Parameter(torch.zeros(2), requires_grad=False)
        self.alpha = nn.Parameter(torch.tensor(0.01), requires_grad=True)
        self.mlp = nn.Sequential(
            nn.Linear(8, hidden), nn.ReLU(),
            nn.Linear(hidden, 2),
        )

    def forward(self, x):
        y_lin = x @ self.W_lin.T + self.b_lin
        return y_lin + self.alpha * self.mlp(x)


def train_residual(model, X, Y, epochs, lr=1e-3, label="", alpha_reg=1.0, val_split=0.2):
    n_val = int(len(X) * val_split)
    X_train, Y_train = X[:-n_val], Y[:-n_val]
    X_val, Y_val = X[-n_val:], Y[-n_val:]

    Xt = torch.from_numpy(X_train).to(DEVICE)
    Yt = torch.from_numpy(Y_train).to(DEVICE)
    Xv = torch.from_numpy(X_val).to(DEVICE)
    Yv = torch.from_numpy(Y_val).to(DEVICE)

    loader = DataLoader(TensorDataset(Xt, Yt), batch_size=256, shuffle=True)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()
    best_val = float('inf')
    best_state = None

    for epoch in range(epochs):
        model.train()
        train_loss = 0.0
        for bx, by in loader:
            pred = model(bx)
            loss = loss_fn(pred, by)
            # L2 penalty on alpha to keep residual small
            loss = loss + alpha_reg * model.alpha ** 2
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * len(bx)

        model.eval()
        with torch.no_grad():
            val_loss = loss_fn(model(Xv), Yv).item()
        if val_loss < best_val:
            best_val = val_loss
            best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}

        if (epoch + 1) % 10 == 0:
            alpha_val = model.alpha.item()
            mae = (model(Xv) - Yv).abs().mean().item()
            print(f"  {label} epoch {epoch + 1}: train={train_loss/len(X_train):.1f} "
                  f"val={val_loss:.1f} mae={mae:.2f} alpha={alpha_val:.4f}")

    if best_state is not None:
        model.load_state_dict(best_state)
        print(f"  {label} best val={best_val:.1f} alpha={model.alpha.item():.4f}")

    # Report residual magnitude
    with torch.no_grad():
        y_full = model(Xv)
        y_lin = Xv @ model.W_lin.T + model.b_lin
        res = (y_full - y_lin).abs().mean().item()
        full = y_full.abs().mean().item()
        print(f"  Residual/total ratio: {res/full*100:.1f}%")


if __name__ == "__main__":
    print("Step 1: OLS linear fit")
    sim_obs, sim_act = collect_sim_demos(200)
    W_ols = np.linalg.lstsq(sim_obs, sim_act, rcond=None)[0]

    print(f"\nStep 2: Train residual MLP on top of frozen OLS")
    model = ResidualModel(W_ols, hidden=16).to(DEVICE)

    # Mix sim + real data
    real_obs, real_act = load_real_data()
    X_all = np.concatenate([sim_obs, real_obs])
    Y_all = np.concatenate([sim_act, real_act])

    train_residual(model, X_all, Y_all, epochs=50, lr=1e-3, label="res", alpha_reg=1.0)

    print("\nEvaluating...")
    eval_bc(model)

    # Check gains
    base = np.zeros(8, dtype=np.float32)
    model.eval()
    with torch.no_grad():
        u0 = model(torch.from_numpy(base).to(DEVICE)).cpu().numpy()
        for i, name in [(2, 'th1'), (3, 'th2'), (6, 'thdot1'), (7, 'thdot2')]:
            obs = base.copy(); obs[i] = 0.001
            u = model(torch.from_numpy(obs).to(DEVICE)).cpu().numpy()
            print(f"  d_u/d_{name:8s} = {(u[0]-u0[0])/0.001:8.0f}")

    torch.save(model.state_dict(), "bc_model.pt")
    print("Saved bc_model.pt")
