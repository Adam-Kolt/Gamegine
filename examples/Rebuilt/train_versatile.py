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
import warnings
from typing import Dict, Any, List
import multiprocessing
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
from ray.rllib.algorithms.callbacks import DefaultCallbacks

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
from examples.Rebuilt.scoring import Fuel
from examples.Rebuilt.reward_function import AdvancedRebuiltRewardFunction



from gamegine.utils.logging import SetLoggingLevel

SetLoggingLevel(logging.FATAL)


class GameMetricsCallback(DefaultCallbacks):
    """Custom callback to log game-specific metrics during training."""
    
    def on_episode_end(self, *, episode, env_runner=None, metrics_logger=None, env=None, env_index=None, rl_module=None, **kwargs):
        """Called at the end of each episode to record custom metrics."""
        # Access the underlying AllianceEnv from the wrapped env
        try:
            # Get actual env
            actual_env = env
            if actual_env is None and env_runner is not None:
                if hasattr(env_runner, "env"):
                    actual_env = env_runner.env
            
            if actual_env is None:
                return
                
            # Handle different env wrapper types
            if hasattr(actual_env, "envs"):  # VectorEnv
                actual_env = actual_env.envs[env_index if env_index is not None else 0]
            if hasattr(actual_env, "_env"):  # Wrapped
                actual_env = actual_env._env
            if hasattr(actual_env, "env"):  # Gym wrapper
                actual_env = actual_env.env
                
            # Get scores from game state
            if hasattr(actual_env, "server") and actual_env.server is not None:
                game_state = actual_env.server.match.game_state
                red_score = game_state.red_score.get()
                blue_score = game_state.blue_score.get()
                
                # New API stack: use metrics_logger
                if metrics_logger is not None:
                    metrics_logger.log_value("red_score", red_score, window=100)
                    metrics_logger.log_value("blue_score", blue_score, window=100)
                    metrics_logger.log_value("score_diff", red_score - blue_score, window=100)
                # Old API stack fallback: use episode.custom_metrics
                elif hasattr(episode, "custom_metrics"):
                    episode.custom_metrics["red_score"] = red_score
                    episode.custom_metrics["blue_score"] = blue_score
                    episode.custom_metrics["score_diff"] = red_score - blue_score
        except Exception:
            # If we can't get scores, just skip (don't crash training)
            pass


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
            gamepieces={Fuel: 8},  # Start with fuel for immediate scoring capability
        )
        
        configs.append(RobotConfig(
            robot=robot,
            start_state=start_state,
            name=name,
            team=team_str,
        ))
    
    return configs


# 1. Define Capability Ranges (Min, Max)
CAPABILITY_RANGES = {
    "max_speed": (1.5, 5.5),           # 1.5 m/s to 5.5 m/s
    "max_acceleration": (1.0, 6.0),    # 1.0 m/s^2 to 6.0 m/s^2
    "rotational_speed": (3.0, 10.0),   # 3 rad/s to 10 rad/s
}

# 2. Define Interaction Ranges (duration in seconds)
INTERACTION_RANGES = {
    "score_fuel": (0.033, 1.0),
    "pickup_1": (0.2, 1.0),
    "pickup_5": (0.5, 2.5),
    "pickup_10": (1.0, 5.0),
    "climb_level_1": (2.0, 10.0),
    "climb_level_2": (3.0, 15.0),
    "climb_level_3": (4.0, 20.0),
    "defend": (0.5, 2.0),
    "shuttle": (0.5, 2.0),
}

# 3. Define Physical Ranges
PHYSICAL_RANGES = {
    "mass": (40.0, 70.0)
}

def env_creator(cfg):
    """Create the environment for training."""
    num_robots = cfg.get("num_robots", 3)
    
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
        max_episode_steps=500,  # Ensure full match duration (160s / dt) is covered
    )
    
    # Assign custom reward function
    env.config.reward_fn = AdvancedRebuiltRewardFunction()
    
    # Enable versatile strategy features
    env.training_config.use_capability_context = True
    env.training_config.randomize_capabilities = True
    env.training_config.randomize_start_pose = True
    env.training_config.capability_ranges = CAPABILITY_RANGES
    env.training_config.interaction_ranges = INTERACTION_RANGES
    env.training_config.physical_ranges = PHYSICAL_RANGES
    
    # Enable opponent awareness
    env.training_config.observe_opponent_states = True
    env.training_config.observe_opponent_capabilities = True # Assume scouting data
    
    # Enable teammate awareness
    env.training_config.observe_teammate_states = True
    env.training_config.observe_teammate_capabilities = True
    
    # Force rebuild of spaces
    env._observation_spaces = {}
    for agent_id in env._agent_ids:
        robot_name = env._agent_to_robot[agent_id]
        env._observation_spaces[agent_id] = env._build_observation_space(robot_name)
    
    from gymnasium import spaces
    env.observation_space = spaces.Dict(env._observation_spaces)
    
    return env


def train_versatile(
    iterations: int = 100, 
    num_robots: int = 2, 
    num_workers: int = 0, # 0 = Auto-detect
    checkpoint_freq: int = 10,
    restore_path: str = None,
):
    """Run the versatile training loop."""
    print("=" * 60)
    print("Versatile Strategy Training (Rebuilt 2026)")
    print("Domain Randomization: ENABLED")
    print("Capability Context:   ENABLED")
    print("Game State Context:   ENABLED")
    print("=" * 60)
    
    # Auto-detect workers if 0
    if num_workers == 0:
        # Leave 2 cores for OS/overhead
        # M4 chips have P-cores and E-cores. Ray handles this decently.
        num_workers = max(1, multiprocessing.cpu_count() - 2)
        print(f"Auto-detected {multiprocessing.cpu_count()} cores. Using {num_workers} workers.")
    
    ray.init(ignore_reinit_error=True)

    register_env("reefscape-versatile-v0", env_creator)
    
    # 3. Inspect Environment
    print("Inspecting environment...")
    temp_env = env_creator({"num_robots": num_robots})
    agent_id = list(temp_env.get_agent_ids())[0]
    obs_sample = temp_env.observation_space[agent_id].sample()
    print(f"Observation Shape: {obs_sample.shape}")
    print(f"  > Standard Dim: 10")
    print(f"  > Capability Context: {obs_sample.shape[0] - 10}")
    temp_env.close()
    
    # 4. Configure PPO with optimized hyperparameters for overnight training
    config = (
        PPOConfig()
        .environment(env="reefscape-versatile-v0", env_config={"num_robots": num_robots})
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
        .framework("torch") # Explicitly use Torch
        .training(
            # Learning rate with decay
            lr=[
                [0, 3e-4],
                [iterations * 2000, 1e-5], # Decay over total timesteps (approx)
            ],
            # Discount factor - high for strategic games
            gamma=0.99,
            # GAE lambda for advantage estimation
            lambda_=0.95,
            # Larger batch for more stable gradients - M4 can handle this
            train_batch_size=16000, 
            # Mini-batch size - increased for M4 memory bandwidth
            minibatch_size=2048,
            # Number of SGD epochs per batch
            num_epochs=15,
            # PPO clip range
            clip_param=0.15,
            # Entropy bonus for exploration (IMPORTANT!)
            entropy_coeff=0.02,
            # Value function loss coefficient
            vf_loss_coeff=0.5,
            # Gradient clipping
            grad_clip=0.5,
        )
        .env_runners(
            num_env_runners=num_workers,
            num_envs_per_env_runner=10, # Vectorize more envs per worker to maximize CPU usage
            # Longer rollouts for strategic learning
            rollout_fragment_length=200,
        )
        .rl_module(
            # Larger network for complex multi-agent strategy
            # Obs space: ~40+ dims (robot state + capabilities + 3 opponents + game state)
            # Action space: 51 discrete actions
            model_config={
                # Reduced capacity for stability/speed (was [512, 1024, 1024, 512])
                "fcnet_hiddens": [512, 512],  
                "fcnet_activation": "relu",
                "vf_share_layers": False,  # Separate value network for stability
            }
        )
        .callbacks(GameMetricsCallback)
    )
    
    algo = config.build()
    
    if restore_path:
        print(f"Restoring from checkpoint: {restore_path}")
        algo.restore(restore_path)
    
    print(f"\nTraining for {iterations} iterations...")
    import os
    for i in range(iterations):
        result = algo.train()
        
        # Extract metrics (handle different RLlib versions/APIs)
        env_runners = result.get("env_runners", {})
        mean_reward = env_runners.get("episode_return_mean", 0) or env_runners.get("episode_reward_mean", 0) or result.get("episode_reward_mean", 0) or 0
        episodes = env_runners.get("num_episodes", 0) or result.get("episodes_this_iter", 0) or 0
        
        # Get episode length info
        ep_len = env_runners.get("episode_len_mean", 0) or result.get("episode_len_mean", 0) or 0
        
        # Get custom metrics if available (from callbacks using metrics_logger)
        # In new API, metrics_logger values appear directly in env_runners
        red_score = env_runners.get("red_score", 0)
        blue_score = env_runners.get("blue_score", 0)
        
        # Debug: show available keys on first iteration
        if i == 0:
            # Look for score-related keys
            score_keys = [k for k in env_runners.keys() if 'score' in k.lower()]
            print(f"  [DEBUG] env_runners keys with 'score': {score_keys}")
        
        # Build output line
        metrics_str = f"Iteration {i+1:3d} | Reward: {mean_reward:7.2f} | Ep: {episodes:3} | Len: {ep_len:5.1f}"
        if red_score or blue_score:
            metrics_str += f" | Red:{red_score:.0f} Blue:{blue_score:.0f}"
        print(metrics_str)
        
        if (i+1) % checkpoint_freq == 0:
            
            checkpoint_dir = os.path.join(os.path.dirname(__file__), "checkpoints")
            os.makedirs(checkpoint_dir, exist_ok=True)
            checkpoint = algo.save(checkpoint_dir)
            print(f"Saved checkpoint to {checkpoint}")

    # Always save final checkpoint
    checkpoint_dir = os.path.join(os.path.dirname(__file__), "checkpoints")
    os.makedirs(checkpoint_dir, exist_ok=True)
    checkpoint = algo.save(checkpoint_dir)
    print(f"Saved final checkpoint to {checkpoint}")
    
    algo.stop()
    ray.shutdown()
    print("\nTraining complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--workers", type=int, default=4) # 4 workers for M4
    parser.add_argument("--robots", type=int, default=3)
    parser.add_argument("--restore", type=str, default=None, help="Path to checkpoint to restore from")
    args = parser.parse_args()
    
    train_versatile(args.iterations, args.robots, args.workers, restore_path=args.restore)
