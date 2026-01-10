"""
Alliance-Based Training Example for Reefscape.

Demonstrates training two alliances against each other using RLlib.

Usage:
    # Install RLlib first
    pip install 'ray[rllib]'
    
    # Run training
    python train_alliance.py
    
    # Run with self-play
    python train_alliance.py --mode self_play
"""
import argparse
import sys
import os

# Suppress Ray/RLlib logging noise
os.environ["RAY_DEDUP_LOGS"] = "1"
os.environ["RAY_SCHEDULER_EVENTS"] = "0"
os.environ["PYTHONWARNINGS"] = "ignore::DeprecationWarning"
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

import logging
logging.getLogger("ray").setLevel(logging.ERROR)
logging.getLogger("ray.rllib").setLevel(logging.ERROR)

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from gamegine.rl import (
    make_alliance_env,
    make_self_play_env,
    make_solo_env,
    RobotConfig,
    AllianceEnv,
)
from gamegine.first.alliance import Alliance
from gamegine.simulation.robot import RobotState
from gamegine.utils.NCIM.ncim import Inch, Degree

# Import Reefscape game (adjust path as needed)
from examples.ReefScapeGamegine.Reefscape import Reefscape
from examples.ReefScapeGamegine.ai_robot import SWERVE_ROBOT, init_robot_interaction
from gamegine.utils.logging import SetLoggingLevel
import logging

# Initialize robot interaction configs (required for actions)
init_robot_interaction()
SetLoggingLevel(logging.FATAL)

def create_robot_configs(alliance: Alliance, num_robots: int = 3):
    """Create robot configurations for an alliance."""
    configs = []
    team_str = "red" if alliance == Alliance.RED else "blue"
    
    for i in range(num_robots):
        name = f"{team_str.capitalize()}{i+1}"
        
        # Starting positions (spread across starting zone)
        if alliance == Alliance.BLUE:
            x = Inch(40 + i * 24)  # Blue side
            heading = Degree(0)
        else:
            x = Inch(600 - 40 - i * 24)  # Red side (mirrored)
            heading = Degree(180)
        
        y = Inch(200)  # Mid-field y
        
        start_state = RobotState(
            x=x,
            y=y,
            heading=heading,
            alliance=alliance,
        )
        
        configs.append(RobotConfig(
            robot=SWERVE_ROBOT,
            start_state=start_state,
            name=name,
            team=team_str,
        ))
    
    return configs


def train_solo(num_robots: int = 3, num_iterations: int = 100, fast_mode: bool = False, num_workers: int = 2, envs_per_worker: int = 1):
    """Train one alliance in solo mode (maximize score, no opponent)."""
    print("=" * 60)
    print("Solo Training Mode")
    print("=" * 60)
    
    import ray
    from ray.rllib.algorithms.ppo import PPOConfig
    from ray.tune.registry import register_env
    from ray.rllib.policy.policy import PolicySpec
    
    ray.init(ignore_reinit_error=True)
    
    # Create environment for space inspection
    blue_robots = create_robot_configs(Alliance.BLUE, num_robots)
    test_env = make_solo_env(
        game=Reefscape,
        robots=blue_robots,
        alliance="blue",
        fast_mode=fast_mode,
        use_server_pool=fast_mode,  # Use pool when in fast mode
    )
    
    print(f"Created environment with {len(blue_robots)} blue robots")
    print(f"Agent IDs: {test_env.get_agent_ids()}")
    print(f"Observation space: {test_env.observation_space}")
    print(f"Action space: {test_env.action_space}")
    
    import torch
    if torch.backends.mps.is_available():
        print("Hardware Acceleration: Enabled (MPS/Metal)")
    else:
        print("Hardware Acceleration: Disabled (CPU only)")
    
    if fast_mode:
        print("Fast Mode: Enabled (teleport instead of trajectory)")
    
    # Register environment
    def env_creator(cfg):
        robots = create_robot_configs(Alliance.BLUE, num_robots)
        return make_solo_env(
            game=Reefscape,
            robots=robots,
            alliance="blue",
            fast_mode=fast_mode,
            use_server_pool=fast_mode,
        )
    
    register_env("reefscape-solo-v0", env_creator)
    # Get spaces from test env (individual agent space, not the Dict wrapper)
    agent_id = list(test_env.get_agent_ids())[0]
    obs_space = test_env._observation_spaces[agent_id]
    act_space = test_env._action_spaces[agent_id]
    test_env.close()
    
    # Configure PPO for multi-agent (all agents share one policy)
    config = (
        PPOConfig()
        .environment(env="reefscape-solo-v0")
        .multi_agent(
            policies={
                "blue_policy": PolicySpec(
                    policy_class=None,
                    observation_space=obs_space,
                    action_space=act_space,
                    config={},
                ),
            },
            policy_mapping_fn=lambda agent_id, *args, **kwargs: "blue_policy",
            policies_to_train=["blue_policy"],
        )
        .training(
            lr=3e-4,
            gamma=0.99,
            train_batch_size=4000,
        )
        .env_runners(
            num_env_runners=num_workers,
            num_envs_per_env_runner=envs_per_worker,
        )
        # Note: MPS (Metal) is not yet supported by RLlib, training runs on CPU
    )
    
    algo = config.build()
    
    print(f"\nTraining for {num_iterations} iterations...")
    print(f"TensorBoard logs: {algo.logdir}")
    print(f"Run: tensorboard --logdir {algo.logdir}\n")
    
    import time
    start_time = time.time()
    
    for i in range(num_iterations):
        iter_start = time.time()
        result = algo.train()
        iter_time = time.time() - iter_start
        
        # Extract metrics (new API stack uses 'return' instead of 'reward')
        env_runners = result.get("env_runners", {})
        
        mean_reward = env_runners.get("episode_return_mean", 0) or env_runners.get("episode_reward_mean", 0) or result.get("episode_reward_mean", 0) or 0
        episodes = env_runners.get("num_episodes", 0) or result.get("episodes_this_iter", 0) or 0
        timesteps = result.get("num_env_steps_sampled_lifetime", 0)
        
        # Progress bar
        progress = (i + 1) / num_iterations * 100
        elapsed = time.time() - start_time
        eta = (elapsed / (i + 1)) * (num_iterations - i - 1) if i > 0 else 0
        
        print(f"  [{i+1:3d}/{num_iterations}] {progress:5.1f}% | Reward: {mean_reward:7.2f} | Episodes: {episodes:3d} | {iter_time:.1f}s/iter | ETA: {eta/60:.1f}m")
    
    # Save checkpoint
    checkpoint = algo.save()
    print(f"\nCheckpoint saved to: {checkpoint}")
    
    algo.stop()
    ray.shutdown()
    print("\nSolo training complete!")


def train_self_play(num_robots: int = 3, num_iterations: int = 100, fast_mode: bool = False, num_workers: int = 2, envs_per_worker: int = 1):
    """Train both alliances against each other using RLlib."""
    print("=" * 60)
    print("Self-Play Training Mode")
    print("=" * 60)
    
    import ray
    import torch
    from ray.rllib.algorithms.ppo import PPOConfig
    from ray.tune.registry import register_env
    from ray.rllib.policy.policy import PolicySpec
    
    ray.init(ignore_reinit_error=True)
    
    # Create environment for space inspection
    red_robots = create_robot_configs(Alliance.RED, num_robots)
    blue_robots = create_robot_configs(Alliance.BLUE, num_robots)
    
    test_env = make_self_play_env(
        game=Reefscape,
        red_robots=red_robots,
        blue_robots=blue_robots,
        share_reward=True,
        fast_mode=fast_mode,
        use_server_pool=fast_mode,
    )
    
    print(f"Created environment with {len(red_robots)} red vs {len(blue_robots)} blue robots")
    print(f"Red agents: {test_env._red_agents}")
    print(f"Blue agents: {test_env._blue_agents}")
    print(f"Observation space: {test_env.observation_space}")
    print(f"Action space: {test_env.action_space}")
    
    if fast_mode:
        print("Fast Mode: Enabled (teleport instead of trajectory)")
    
    # Get spaces (individual agent space, not the Dict wrapper)
    agent_id = list(test_env.get_agent_ids())[0]
    obs_space = test_env._observation_spaces[agent_id]
    act_space = test_env._action_spaces[agent_id]
    test_env.close()
    
    # Register environment
    def env_creator(cfg):
        return make_self_play_env(
            game=Reefscape,
            red_robots=create_robot_configs(Alliance.RED, num_robots),
            blue_robots=create_robot_configs(Alliance.BLUE, num_robots),
            fast_mode=fast_mode,
            use_server_pool=fast_mode,
        )
    
    register_env("reefscape-alliance-v0", env_creator)
    
    # Configure PPO for multi-agent with proper PolicySpec
    config = (
        PPOConfig()
        .environment(env="reefscape-alliance-v0")
        .multi_agent(
            policies={
                "red_policy": PolicySpec(
                    policy_class=None,
                    observation_space=obs_space,
                    action_space=act_space,
                    config={},
                ),
                "blue_policy": PolicySpec(
                    policy_class=None,
                    observation_space=obs_space,
                    action_space=act_space,
                    config={},
                ),
            },
            policy_mapping_fn=lambda agent_id, *args, **kwargs: 
                "red_policy" if agent_id.startswith("red") else "blue_policy",
            policies_to_train=["red_policy", "blue_policy"],
        )
        .training(
            lr=3e-4,
            gamma=0.99,
            train_batch_size=4000,
        )
        .env_runners(
            num_env_runners=num_workers,
            num_envs_per_env_runner=envs_per_worker,
        )
        # Note: MPS (Metal) is not yet supported by RLlib, training runs on CPU
    )
    
    algo = config.build()
    
    print(f"\nTraining for {num_iterations} iterations...")
    print(f"TensorBoard logs: {algo.logdir}")
    print(f"Run: tensorboard --logdir {algo.logdir}\n")
    
    import time
    start_time = time.time()
    
    for i in range(num_iterations):
        iter_start = time.time()
        result = algo.train()
        iter_time = time.time() - iter_start
        
        # Extract per-policy rewards
        env_runners = result.get("env_runners", {})
        
        # Check for new API keys (module_episode_returns_mean)
        policy_rewards = env_runners.get("module_episode_returns_mean", {}) or env_runners.get("policy_reward_mean", {}) or result.get("policy_reward_mean", {}) or {}
        
        red_reward = policy_rewards.get("red_policy", 0) or 0
        blue_reward = policy_rewards.get("blue_policy", 0) or 0
        episodes = env_runners.get("num_episodes", 0) or result.get("episodes_this_iter", 0) or 0
        
        # Progress
        progress = (i + 1) / num_iterations * 100
        elapsed = time.time() - start_time
        eta = (elapsed / (i + 1)) * (num_iterations - i - 1) if i > 0 else 0
        
        print(f"  [{i+1:3d}/{num_iterations}] {progress:5.1f}% | Red: {red_reward:7.2f} | Blue: {blue_reward:7.2f} | Eps: {episodes:3d} | {iter_time:.1f}s | ETA: {eta/60:.1f}m")
    
    # Save checkpoint
    checkpoint = algo.save()
    print(f"\nCheckpoint saved to: {checkpoint}")
    
    algo.stop()
    ray.shutdown()
    print("\nSelf-play training complete!")


def main():
    parser = argparse.ArgumentParser(description="Train alliance-based RL agents for Reefscape")
    parser.add_argument(
        "--mode", 
        choices=["solo", "self_play"],
        default="self_play",
        help="Training mode: solo (one alliance) or self_play (both alliances compete)"
    )
    parser.add_argument(
        "--robots",
        type=int,
        default=3,
        help="Number of robots per alliance (default: 3)"
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=100,
        help="Number of training iterations (default: 100)"
    )
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Enable fast mode (teleport instead of trajectory generation)"
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=2,
        help="Number of parallel environment workers (default: 2)"
    )
    parser.add_argument(
        "--envs-per-worker",
        type=int,
        default=1,
        help="Number of sub-environments per worker (default: 1)"
    )
    
    args = parser.parse_args()
    
    if args.mode == "solo":
        train_solo(args.robots, args.iterations, args.fast, args.workers, args.envs_per_worker)
    else:
        train_self_play(args.robots, args.iterations, args.fast, args.workers, args.envs_per_worker)


if __name__ == "__main__":
    main()
