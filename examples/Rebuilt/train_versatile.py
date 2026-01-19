"""
Versatile Strategy Network Training.

Trains a single RL policy to control robots with varying physical capabilities.
Uses Domain Randomization to sample different robot stats (speed, accel) at the start of each episode.
The policy receives these stats as context in the observation space.

Usage:
    python train_versatile.py --iterations 200 --workers 4
"""

import argparse
import logging
import os
import sys
import warnings

# Suppress logging noise
os.environ["RAY_DEDUP_LOGS"] = "1"
os.environ["RAY_SCHEDULER_EVENTS"] = "0"
os.environ["PYTHONWARNINGS"] = "ignore::DeprecationWarning"
warnings.filterwarnings("ignore", category=DeprecationWarning)
logging.getLogger("ray").setLevel(logging.ERROR)

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import ray
from ray.rllib.algorithms.ppo import PPOConfig
from ray.tune.registry import register_env
from ray.rllib.policy.policy import PolicySpec

from gamegine.first.alliance import Alliance
from gamegine.rl import (
    make_alliance_env,
    RobotConfig,
    AllianceEnv,
)
from gamegine.representation.capabilities import RobotCapabilities
from gamegine.simulation.robot import RobotState
from gamegine.utils.NCIM.ncim import Inch, Degree, MetersPerSecond
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared

# Import Game
from examples.Rebuilt.Rebuilt import create_rebuilt_game, FIELD_WIDTH, FIELD_LENGTH
from examples.Rebuilt.robot import create_robot, setup_robot_interactions, ROBOT_WIDTH
from gamegine.utils.logging import SetLoggingLevel

SetLoggingLevel(logging.FATAL)


def create_robot_configs(alliance: Alliance, game, num_robots: int = 3):
    """Create robot configurations (without specific capabilities, they will be randomized)."""
    configs = []
    team_str = "red" if alliance == Alliance.RED else "blue"
    
    for i in range(num_robots):
        name = f"{team_str.capitalize()}{i+1}"
        
        # Use shared robot factory
        robot = create_robot(name, alliance)
        
        # Configure interactions
        setup_robot_interactions(robot, game)
        
        # Override start position
        if alliance == Alliance.BLUE:
            x = Inch(40 + i * 48)
            heading = Degree(0)
        else:
            x = FIELD_LENGTH - Inch(40 + i * 48)
            heading = Degree(180)
        
        y = FIELD_WIDTH / 2 + Inch(i * 20) # Center-ish
        
        start_state = RobotState(
            x=x,
            y=y,
            heading=heading,
            alliance=alliance,
        )
        
        configs.append(RobotConfig(
            robot=robot,
            start_state=start_state,
            name=name,
            team=team_str,
        ))
    
    return configs


def train_versatile(
    iterations: int = 100, 
    num_robots: int = 2, 
    num_workers: int = 2,
    checkpoint_freq: int = 10,
):
    """Run the versatile training loop."""
    print("=" * 60)
    print("Versatile Strategy Training (Rebuilt 2026)")
    print("Domain Randomization: ENABLED")
    print("Capability Context:   ENABLED")
    print("Game State Context:   ENABLED")
    print("=" * 60)
    
    ray.init(ignore_reinit_error=True)
    
    # 1. Define Capability Ranges (Min, Max)
    # The network will see capabilities uniformly sampled from these ranges
    capability_ranges = {
        "max_speed": (1.5, 5.5),           # 1.5 m/s to 5.5 m/s
        "max_acceleration": (1.0, 6.0),    # 1.0 m/s^2 to 6.0 m/s^2
        "rotational_speed": (3.0, 10.0),   # 3 rad/s to 10 rad/s
    }

    # 2. Configure Environment Factory
    def env_creator(cfg):
        # Create game instance with observation support
        game = create_rebuilt_game()
        
        # Base robots
        red_robots = create_robot_configs(Alliance.RED, game, num_robots)
        blue_robots = create_robot_configs(Alliance.BLUE, game, num_robots)
        
        # Create env with capability flags
        env = make_alliance_env(
            game=game,
            red_robots=red_robots,
            blue_robots=blue_robots,
            mode="self_play",
            fast_mode=True,         # Fast mode recommended for large training runs
            use_server_pool=True,   # Reuse servers for speed
        )
        
        # Enable versatile strategy features
        env.training_config.use_capability_context = True
        env.training_config.randomize_capabilities = True
        env.training_config.capability_ranges = capability_ranges
        
        # Enable opponent awareness
        env.training_config.observe_opponent_states = True
        env.training_config.observe_opponent_capabilities = True # Assume scouting data
        
        # Re-initialize to ensure obs spaces are correct
        # (This is a bit hacky, normally passed in config, but modifying after creation works for AllianceEnv)
        # Better way: Modify make_alliance_env to accept these, but direct property modification works 
        # because spaces are rebuilt in __init__ which ran... wait.
        # Issue: __init__ runs BEFORE we modify these flags, so observation space will be wrong.
        # We need to hack it or manually reconstruct
        
        # Proper way: Pass Config object manually or re-call internal init logic
        # For this script, we will rebuild the observation spaces manually
        
        # Force rebuild of spaces
        env._observation_spaces = {}
        for agent_id in env._agent_ids:
            robot_name = env._agent_to_robot[agent_id]
            env._observation_spaces[agent_id] = env._build_observation_space(robot_name)
        
        from gymnasium import spaces
        env.observation_space = spaces.Dict(env._observation_spaces)
        
        return env

    register_env("reefscape-versatile-v0", env_creator)
    
    # 3. Inspect Environment
    print("Inspecting environment...")
    temp_env = env_creator({})
    agent_id = list(temp_env.get_agent_ids())[0]
    obs_sample = temp_env.observation_space[agent_id].sample()
    print(f"Observation Shape: {obs_sample.shape}")
    print(f"  > Standard Dim: 10")
    print(f"  > Capability Context: {obs_sample.shape[0] - 10}")
    temp_env.close()
    
    # 4. Configure PPO
    config = (
        PPOConfig()
        .environment(env="reefscape-versatile-v0")
        .multi_agent(
            policies={
                "versatile_policy": PolicySpec(
                    policy_class=None,
                    observation_space=temp_env.observation_space[agent_id],
                    action_space=temp_env.action_space[agent_id],
                    config={},
                ),
            },
            # Map all agents to the same versatile policy
            policy_mapping_fn=lambda agent_id, *args, **kwargs: "versatile_policy",
            policies_to_train=["versatile_policy"],
        )
        .training(
            lr=3e-4,
            gamma=0.99,
            train_batch_size=4000,
        )
        .env_runners(
            num_env_runners=num_workers,
            num_envs_per_env_runner=2, 
        )
    )
    
    algo = config.build()
    
    print(f"\nTraining for {iterations} iterations...")
    
    for i in range(iterations):
        result = algo.train()
        
        # Extract metrics (handle different RLlib versions/APIs)
        env_runners = result.get("env_runners", {})
        mean_reward = env_runners.get("episode_return_mean", 0) or env_runners.get("episode_reward_mean", 0) or result.get("episode_reward_mean", 0) or 0
        episodes = env_runners.get("num_episodes", 0) or result.get("episodes_this_iter", 0) or 0
        
        print(f"Iteration {i+1:3d} | Reward: {mean_reward:7.2f} | Episodes: {episodes}")
        
        if (i+1) % checkpoint_freq == 0:
            checkpoint = algo.save()
            print(f"Saved checkpoint to {checkpoint}")
            
    algo.stop()
    ray.shutdown()
    print("\nTraining complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--robots", type=int, default=2)
    args = parser.parse_args()
    
    train_versatile(args.iterations, args.robots, args.workers)
