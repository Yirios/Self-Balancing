"""Train PPO with BC regularization — compare KL-penalty vs reward-penalty."""
import gymnasium as gym
import torch
import balancing_robot
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor

from pretrain_bc import BCModel, load_bc_into_ppo
from kl_ppo import KLRegularizedPPO, BCReference

BC_COEF = 0.01  # BC regularization strength


def make_env(inject_noise=False, domain_rand_scale=0.0):
    return Monitor(
        gym.make("BalancingRobot-v0",
                 inject_noise=inject_noise, domain_rand_scale=domain_rand_scale)
    )


if __name__ == "__main__":
    # Train with noise + domain randomization for Sim-to-Real robustness
    env = DummyVecEnv([lambda: make_env(inject_noise=True, domain_rand_scale=0.05) for _ in range(4)])
    env = VecNormalize(env, norm_obs=False, norm_reward=True)

    # Eval on clean environment
    eval_env = DummyVecEnv([lambda: make_env(inject_noise=False, domain_rand_scale=0.0)])
    eval_env = VecNormalize(eval_env, norm_obs=False, norm_reward=False)

    policy_kwargs = dict(net_arch=[32, 32], activation_fn=torch.nn.ReLU)

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
    print(f"KL-regularized PPO: bc_coef={BC_COEF}")

    print("Starting training...")
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="./best_model_reg/",
        eval_freq=10000,
        verbose=1,
    )

    model.learn(total_timesteps=1_500_000, callback=eval_callback)

    model.save("ppo_balance_bot_kl")
    env.save("vec_normalize.pkl")
    print("Training done.")
