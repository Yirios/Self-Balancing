"""Train PPO with BC regularization — compare KL-penalty vs reward-penalty."""
import gymnasium as gym
import torch
import balancing_robot
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor

from pretrain_bc import BCModel, load_bc_into_ppo
from kl_ppo import KLRegularizedPPO, BCReference

REG_METHOD = "kl"  # "kl" = KL divergence in loss, "reward" = LQR penalty in reward
BC_COEF = 0.01  # BC regularization strength (for KL method)
BC_BETA = 1e-7  # BC penalty in reward (for reward method)


def make_env(method):
    if method == "reward":
        return Monitor(
            gym.make("BalancingRobot-v0", bc_beta=BC_BETA, max_steps=1400)
        )
    return Monitor(gym.make("BalancingRobot-v0"))


if __name__ == "__main__":
    print(f"Method: {REG_METHOD}")

    env = DummyVecEnv([lambda: make_env(REG_METHOD) for _ in range(4)])
    env = VecNormalize(env, norm_obs=False, norm_reward=True)

    eval_env = DummyVecEnv([lambda: make_env("kl")])
    eval_env = VecNormalize(eval_env, norm_obs=False, norm_reward=False)

    policy_kwargs = dict(net_arch=[32, 32], activation_fn=torch.nn.ReLU)

    if REG_METHOD == "kl":
        bc = BCModel()
        bc.load_state_dict(torch.load("bc_model.pt", map_location="cpu"))
        bc_ref = BCReference(bc)

        model = KLRegularizedPPO(
            "MlpPolicy",
            env,
            learning_rate=1e-4,
            n_steps=1024,
            batch_size=128,
            n_epochs=5,
            gamma=0.99,
            clip_range=0.1,
            ent_coef=0.01,
            policy_kwargs=policy_kwargs,
            bc_policy=bc_ref,
            bc_coef=BC_COEF,
            verbose=1,
            device="cpu",
        )
        load_bc_into_ppo(bc, model)
        print(f"BC-reg PPO (action MSE): bc_coef={BC_COEF}")
    else:
        # reward-based regularization
        from stable_baselines3 import PPO

        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=1e-4,
            n_steps=1024,
            batch_size=128,
            n_epochs=5,
            gamma=0.99,
            clip_range=0.1,
            ent_coef=0.01,
            policy_kwargs=policy_kwargs,
            verbose=1,
            device="cpu",
        )
        bc = BCModel()
        bc.load_state_dict(torch.load("bc_model.pt", map_location="cpu"))
        load_bc_into_ppo(bc, model)
        print(f"Reward-reg PPO: bc_beta={BC_BETA}")

    print("Starting training...")
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="./best_model_reg/",
        eval_freq=10000,
        verbose=1,
    )

    model.learn(total_timesteps=1_500_000, callback=eval_callback)

    model.save(f"ppo_balance_bot_{REG_METHOD}")
    env.save("vec_normalize.pkl")
    print("Training done.")
