import argparse
import os

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.logger import configure

# Assuming ActiveSuspensionEnv is implemented elsewhere in this project
from active_suspension_env import ActiveSuspensionEnv

class RewardLoggingCallback(BaseCallback):
    """Callback for logging episode rewards to a CSV file."""

    def __init__(self, log_path: str):
        super().__init__()
        self.log_path = log_path
        self.episode_rewards = []

    def _on_step(self) -> bool:
        if self.locals.get("done"):
            reward = self.locals.get("reward")
            if reward is not None:
                self.episode_rewards.append(float(reward))
        return True

    def _on_training_end(self) -> None:
        if self.log_path:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
            with open(self.log_path, "w") as f:
                for r in self.episode_rewards:
                    f.write(f"{r}\n")


def train(total_timesteps: int, checkpoint_freq: int, checkpoint_dir: str, log_dir: str, reward_log: str):
    env = DummyVecEnv([ActiveSuspensionEnv])

    os.makedirs(checkpoint_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    # Stable-Baselines3 logger configuration
    new_logger = configure(log_dir, ["stdout", "csv", "tensorboard"])

    model = PPO("MlpPolicy", env, verbose=1)
    model.set_logger(new_logger)

    checkpoint_callback = CheckpointCallback(
        save_freq=checkpoint_freq,
        save_path=checkpoint_dir,
        name_prefix="ppo_active_suspension",
    )

    reward_callback = RewardLoggingCallback(reward_log)

    model.learn(total_timesteps=total_timesteps, callback=[checkpoint_callback, reward_callback])

    model.save(os.path.join(checkpoint_dir, "ppo_final"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train PPO on ActiveSuspensionEnv")
    parser.add_argument("--timesteps", type=int, default=100_000, help="Total training timesteps")
    parser.add_argument("--checkpoint-freq", type=int, default=10_000, help="Save model every n steps")
    parser.add_argument("--checkpoint-dir", type=str, default="checkpoints", help="Directory to save checkpoints")
    parser.add_argument("--log-dir", type=str, default="logs", help="Logging directory")
    parser.add_argument("--reward-log", type=str, default="logs/rewards.csv", help="Path to save reward log")
    args = parser.parse_args()

    train(
        total_timesteps=args.timesteps,
        checkpoint_freq=args.checkpoint_freq,
        checkpoint_dir=args.checkpoint_dir,
        log_dir=args.log_dir,
        reward_log=args.reward_log,
    )

