"""Regularized PPO: penalizes deviation from a frozen BC reference policy."""
import gymnasium as gym
import numpy as np
import torch
import torch.nn as nn
from stable_baselines3.common.utils import explained_variance
from stable_baselines3.ppo import PPO


class BCReference(nn.Module):
    """Frozen BC model, outputs reference action for a given state."""

    def __init__(self, bc_model):
        super().__init__()
        self.features = bc_model.features
        self.policy_net = bc_model.policy_net
        self.action_net = bc_model.action_net
        for p in self.parameters():
            p.requires_grad = False
        self.eval()

    def forward(self, obs):
        x = self.features(obs.float())
        x = self.policy_net(x)
        return self.action_net(x)


class KLRegularizedPPO(PPO):
    """PPO with BC regularization via action-MSE penalty.

    Loss = PPO_loss + bc_coef * ||μ_ppo - μ_bc||²_2 / 2σ²_bc
    This is equivalent to KL(π_ppo || π_bc) when both share variance σ²_bc.
    """

    def __init__(self, *args, bc_policy: BCReference = None, bc_coef: float = 0.01, **kwargs):
        super().__init__(*args, **kwargs)
        self.bc_policy = bc_policy
        self.bc_coef = bc_coef

    def train(self) -> None:
        if self.bc_policy is None or self.bc_coef <= 0:
            return super().train()

        self.policy.set_training_mode(True)
        self._update_current_progress_remaining(self.num_timesteps, self._total_timesteps)
        self.policy.optimizer.zero_grad()

        clip_range = self.clip_range(self._current_progress_remaining)
        if self.clip_range_vf is not None:
            clip_range_vf = self.clip_range_vf(self._current_progress_remaining)

        entropy_losses, pg_losses, value_losses, bc_losses = [], [], [], []
        clip_fractions, approx_kl_divs = [], []

        continue_training = True
        for epoch in range(self.n_epochs):
            for rollout_data in self.rollout_buffer.get(self.batch_size):
                actions = rollout_data.actions
                if isinstance(self.action_space, gym.spaces.Discrete):
                    actions = actions.long().flatten()

                # Get PPO policy distribution
                features = self.policy.extract_features(rollout_data.observations)
                if self.policy.share_features_extractor:
                    latent_pi, latent_vf = self.policy.mlp_extractor(features)
                else:
                    pi_features, vf_features = features
                    latent_pi = self.policy.mlp_extractor.forward_actor(pi_features)
                    latent_vf = self.policy.mlp_extractor.forward_critic(vf_features)

                distribution = self.policy._get_action_dist_from_latent(latent_pi)
                log_prob = distribution.log_prob(actions)
                values = self.policy.value_net(latent_vf)
                entropy = distribution.entropy()
                values = values.flatten()

                # BC regularization: penalize mean deviation from BC reference
                ppo_mean = distribution.distribution.mean
                with torch.no_grad():
                    bc_mean = self.bc_policy(rollout_data.observations)
                bc_loss = ((ppo_mean - bc_mean) ** 2).sum(dim=-1).mean()
                bc_losses.append(bc_loss.item())

                # Normalize advantage
                advantages = rollout_data.advantages
                if self.normalize_advantage and len(advantages) > 1:
                    advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

                # PPO clipped surrogate
                ratio = torch.exp(log_prob - rollout_data.old_log_prob)
                policy_loss_1 = advantages * ratio
                policy_loss_2 = advantages * torch.clamp(ratio, 1 - clip_range, 1 + clip_range)
                policy_loss = -torch.min(policy_loss_1, policy_loss_2).mean()
                pg_losses.append(policy_loss.item())
                clip_fraction = torch.mean((torch.abs(ratio - 1) > clip_range).float()).item()
                clip_fractions.append(clip_fraction)

                # Value loss
                if self.clip_range_vf is None:
                    values_pred = values
                else:
                    values_pred = rollout_data.old_values + torch.clamp(
                        values - rollout_data.old_values, -clip_range_vf, clip_range_vf
                    )
                value_loss = nn.functional.mse_loss(rollout_data.returns, values_pred)
                value_losses.append(value_loss.item())

                # Entropy loss
                if entropy is None:
                    entropy_loss = -torch.mean(-log_prob)
                else:
                    entropy_loss = -torch.mean(entropy)
                entropy_losses.append(entropy_loss.item())

                loss = (
                    policy_loss
                    + self.ent_coef * entropy_loss
                    + self.vf_coef * value_loss
                    + self.bc_coef * bc_loss
                )

                with torch.no_grad():
                    log_ratio = log_prob - rollout_data.old_log_prob
                    approx_kl_div = torch.mean((torch.exp(log_ratio) - 1) - log_ratio).cpu().numpy()
                    approx_kl_divs.append(approx_kl_div)

                if self.target_kl is not None and approx_kl_div > 1.5 * self.target_kl:
                    continue_training = False
                    if self.verbose >= 1:
                        print(f"Early stopping at epoch {epoch} due to max kl: {approx_kl_div:.2f}")
                    break

                self.policy.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
                self.policy.optimizer.step()

            self._n_updates += 1
            if not continue_training:
                break

        explained_var = explained_variance(
            self.rollout_buffer.values.flatten(),
            self.rollout_buffer.returns.flatten(),
        )

        self.logger.record("train/entropy_loss", np.mean(entropy_losses))
        self.logger.record("train/policy_gradient_loss", np.mean(pg_losses))
        self.logger.record("train/value_loss", np.mean(value_losses))
        self.logger.record("train/bc_reg_loss", np.mean(bc_losses))
        self.logger.record("train/approx_kl", np.mean(approx_kl_divs))
        self.logger.record("train/clip_fraction", np.mean(clip_fractions))
        self.logger.record("train/loss", loss.item())
        self.logger.record("train/explained_variance", explained_var)
        if hasattr(self.policy, "log_std"):
            self.logger.record("train/std", torch.exp(self.policy.log_std).mean().item())

        self.logger.dump(step=self.num_timesteps)

    def _excluded_save_params(self):
        return super()._excluded_save_params() + ["bc_policy"]
