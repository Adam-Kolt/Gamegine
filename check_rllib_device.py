
import ray
from ray.rllib.algorithms.ppo import PPOConfig
from gamegine.rl.envs.alliance_env import AllianceEnv
from gamegine.first.alliance import Alliance
from gamegine.rl.config import RobotConfig, RobotTeamConfig, SwerveConfig, EnvConfig, TrainingConfig
from gamegine.representation.game import Game
from gamegine.simulation.GameServer import DiscreteGameServer
import gymnasium as gym
from ray.tune.registry import register_env
from gamegine.examples.ReefScapeGamegine.Reefscape import Reefscape
from gamegine.examples.ReefScapeGamegine.ai_robot import SWERVE_ROBOT, init_robot_interaction

# Fix robot name for verification env
config = RobotConfig(
    name="blue_0",
    team="blue",
    robot=SWERVE_ROBOT,
    start_state={"x": 5, "y": 5, "heading": 0}
)
env_config = EnvConfig(robots=[config])

# Initialize
init_robot_interaction()

def env_creator(cfg):
    return AllianceEnv(
        game=Reefscape,
        config=env_config
    )

register_env("test_env", env_creator)

ray.init(ignore_reinit_error=True)

# Config without explicit GPU reqs
config = (
    PPOConfig()
    .environment("test_env")
    .env_runners(num_env_runners=0)
    .framework("torch")
)

algo = config.build()

# Access policy
policy = algo.get_policy()
print(f"Policy device: {policy.device}")

ray.shutdown()
