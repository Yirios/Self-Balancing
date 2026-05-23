"""Train PPO with residual bottleneck fine-tuning from BC pre-trained weights.

Architecture: 8→16→(4D bottleneck residual)→16→2
- Custom features extractor: encodes 8D obs into 16D features via residual bottleneck
- W1 (8→16) fixed as BC encoder, W2 (16→2) fixed as BC decoder
- Bottleneck: W_down (16→4) + W_up (4→16), zero-init, LeakyReLU
- Residual: h = h1 + h2 preserves LQR baseline at initialization

SB3 integration: custom features_extractor replaces default identity extractor.
mlp_extractor is set to identity (net_arch=[]), action_net gets W2 weights.
"""
import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from gymnasium import spaces

import balancing_robot

from pretrain_bc import BCModel, load_bc_full, get_bc_weights
from kl_ppo import KLRegularizedPPO, BCReference

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
FREEZE_BC = True
BC_COEF = 0.01
LEARNING_RATE = 5e-5


def make_env(inject_noise=False, domain_rand_scale=0.0, pendulum_disturb_std=0.0,
            data_driven=False):
    return Monitor(gym.make("BalancingRobot-v0",
        inject_noise=inject_noise, domain_rand_scale=domain_rand_scale,
        pendulum_disturb_std=pendulum_disturb_std, data_driven=data_driven))


class ResidualBottleneckExtractor(BaseFeaturesExtractor):
    """Feature extractor: 8D obs → 16D features via residual bottleneck.

    Standardizes input, then applies:
      Encoder (W1): 8→16 ReLU (BC pre-trained, frozen)
      Bottleneck:   16→4→16 LeakyReLU (learnable, zero-init)
      Residual:     output = encoder(obs_s) + bottleneck(encoder(obs_s))
    """

    def __init__(self, observation_space: spaces.Box, w1: torch.Tensor = None,
                 x_mean: torch.Tensor = None, x_std: torch.Tensor = None):
        super().__init__(observation_space, features_dim=16)
        H = 16
        if w1 is None:
            raise ValueError("ResidualBottleneckExtractor requires w1 (BC encoder weights)")
        self.encoder_w = nn.Parameter(w1.clone())
        self.register_buffer("x_mean", x_mean.clone() if x_mean is not None
                             else torch.zeros(8))
        self.register_buffer("x_std", x_std.clone() if x_std is not None
                             else torch.ones(8))
        self.down = nn.Linear(H, 4, bias=False)
        self.up = nn.Linear(4, H, bias=False)

        nn.init.orthogonal_(self.down.weight, gain=0.5)
        nn.init.zeros_(self.up.weight)

        if FREEZE_BC:
            self.encoder_w.requires_grad = False

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        obs_s = (obs - self.x_mean) / self.x_std
        h1 = F.relu(F.linear(obs_s, self.encoder_w))
        z = F.leaky_relu(self.down(h1), 0.01)
        h2 = F.leaky_relu(self.up(z), 0.01)
        return h1 + h2


def load_bc_into_residual_ppo(bc_model, y_mean, y_std, ppo_model):
    """Embed BC weights into the residual PPO policy.

    Bake output de-standardization into action_net:
      u = (W2_raw @ h) * y_std + y_mean
        = (W2_raw * y_std') @ h + y_mean   where y_std' reshaped for broadcast
    """
    _, W2_raw = get_bc_weights(bc_model)  # W2_raw: 2×16
    y_std_v = y_std.squeeze()              # (2,) or (1,2) → (2,)
    y_mean_v = y_mean.squeeze()            # (2,)

    action_net = ppo_model.policy.action_net
    action_net.weight.data.copy_(
        W2_raw.to(action_net.weight.device) *
        y_std_v.to(action_net.weight.device).reshape(-1, 1))
    action_net.bias.data.copy_(y_mean_v.to(action_net.bias.device))

    if FREEZE_BC:
        action_net.weight.requires_grad = False
        action_net.bias.requires_grad = False


def eval_policy(ppo_model, n_episodes=50, label=""):
    env = Monitor(gym.make("BalancingRobot-v0", data_driven=True, max_steps=2000))
    lengths = []
    for _ in range(n_episodes):
        obs, _ = env.reset()
        done = False
        steps = 0
        while not done:
            u, _ = ppo_model.predict(obs, deterministic=True)
            obs, _, term, trunc, _ = env.step(u)
            done = term or trunc
            steps += 1
        lengths.append(steps)
    env.close()
    survived = sum(1 for l in lengths if l >= 1999)
    print(f"{label} eval: mean={np.mean(lengths):.0f} max={np.max(lengths)} "
          f"survived={survived}/{n_episodes}")
    return lengths


if __name__ == "__main__":
    print("Loading BC pre-trained weights...")
    bc, x_mean, x_std, y_mean, y_std = load_bc_full("models/bc_model.pt")
    bc = bc.to(DEVICE)
    W1, _ = get_bc_weights(bc)  # raw encoder weights (trained on standardized data)

    # Train env
    env = DummyVecEnv([lambda: make_env(inject_noise=True, domain_rand_scale=0.05,
                                        pendulum_disturb_std=0.8,
                                        data_driven=True) for _ in range(4)])
    env = VecNormalize(env, norm_obs=False, norm_reward=True)

    eval_env = DummyVecEnv([lambda: make_env(inject_noise=False, domain_rand_scale=0.0,
                                            data_driven=True)])
    eval_env = VecNormalize(eval_env, norm_obs=False, norm_reward=False)

    # BC reference with standardization (must be on same device as training)
    bc_ref = BCReference(bc, x_mean=x_mean.to(DEVICE), x_std=x_std.to(DEVICE),
                         y_mean=y_mean.to(DEVICE), y_std=y_std.to(DEVICE))

    # net_arch=[] → mlp_extractor is identity (bottleneck is in features extractor)
    policy_kwargs = dict(
        net_arch=[],
        features_extractor_class=ResidualBottleneckExtractor,
        features_extractor_kwargs=dict(w1=W1,
                                       x_mean=x_mean.to(DEVICE),
                                       x_std=x_std.to(DEVICE)),
    )

    model = KLRegularizedPPO(
        "MlpPolicy",
        env,
        learning_rate=LEARNING_RATE,
        n_steps=1024,
        batch_size=128,
        n_epochs=3,
        gamma=0.99,
        clip_range=0.05,
        ent_coef=0.01,
        policy_kwargs=policy_kwargs,
        bc_policy=bc_ref,
        bc_coef=BC_COEF,
        verbose=1,
        device=DEVICE,
    )

    # Embed BC W2 (with output de-standardization) into action_net
    load_bc_into_residual_ppo(bc, y_mean, y_std, model)

    frozen_str = "frozen" if FREEZE_BC else "trainable"
    print(f"KL-PPO residual bottleneck: bc_coef={BC_COEF} lr={LEARNING_RATE} "
          f"BC={frozen_str} bottleneck=16→4→16")

    print("Starting training...")
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="./models/best_model_residual/",
        eval_freq=10000,
        verbose=1,
    )

    model.learn(total_timesteps=1_500_000, callback=eval_callback)

    model.save("models/ppo_residual")
    env.save("models/vec_normalize_residual.pkl")

    print("\nFinal evaluation...")
    eval_policy(model, label="PPO residual")
    print("Training done.")
