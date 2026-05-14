from gymnasium.envs.registration import register

register(
    id="BalancingRobot-v0",
    entry_point="balancing_robot.env:BalancingRobotEnv",
    max_episode_steps=3000,
)
