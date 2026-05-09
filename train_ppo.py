"""Fine-tune a pretrained BC model with PPO (gentle fine-tuning)."""
import gymnasium as gym
import torch
import balancing_robot
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor

from pretrain_bc import BCModel, load_bc_into_ppo


def make_env():
    return Monitor(gym.make("BalancingRobot-v0"))


if __name__ == "__main__":
    # Load BC model (already trained)
    bc = BCModel()
    bc.load_state_dict(torch.load("bc_model.pt", map_location="cpu"))

    env = DummyVecEnv([make_env for _ in range(4)])
    env = VecNormalize(env, norm_obs=False, norm_reward=True)

    eval_env = DummyVecEnv([make_env])
    eval_env = VecNormalize(eval_env, norm_obs=False, norm_reward=False)

    # Create PPO with gentle fine-tuning hyperparams
    policy_kwargs = dict(net_arch=[32, 32], activation_fn=torch.nn.ReLU)
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

    load_bc_into_ppo(bc, model)

    print("Starting PPO fine-tuning...")
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="./best_model/",
        eval_freq=10000,
        verbose=1,
    )

    model.learn(total_timesteps=1_500_000, callback=eval_callback)
    model.save("ppo_balance_bot")
    env.save("vec_normalize.pkl")
    print("Training done.")
