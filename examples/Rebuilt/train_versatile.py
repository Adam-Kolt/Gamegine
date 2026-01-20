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


class RebuiltRewardFunction:
    """Custom reward function for Rebuilt 2026.
    
    Adds:
    - Pickup reward: +0.1 per fuel acquired (dense signal for chain)
    - Inactive Hub penalty: -0.5 for scoring in wrong hub phase
    """
    
    def __init__(self):
        self.prev_fuel = {}  # robot_name -> fuel_count
        
    def __call__(self, game_state, robot_states, action_valid, action_names):
        rewards = {}
        current_fuel = {}
        
        for name, state in robot_states.items():
            # GET FUEL
            fuel_dict = state.gamepieces.get()
            fuel_count = fuel_dict.get(Fuel, 0)
            current_fuel[name] = fuel_count
            
            # 1. PICKUP REWARD
            # If fuel increased, give bonus
            pickup_reward = 0.0
            if name in self.prev_fuel:
                 delta = fuel_count - self.prev_fuel[name]
                 if delta > 0:
                     pickup_reward = delta * 0.1  # +0.1 per fuel (small incentive)
            
            # 2. INACTIVE HUB PENALTY
            # Check action name for "score_fuel" and inactive hub
            action_name = action_names.get(name, "")
            inactive_penalty = 0.0
            
            if "score_fuel" in action_name:
                hub_name = None
                if "Blue Hub" in action_name:
                    hub_name = "Blue Hub"
                elif "Red Hub" in action_name:
                    hub_name = "Red Hub"
                
                if hub_name:
                    try:
                        hub = game_state.get("interactables")[hub_name]
                        is_active = hub.getValue("is_active").get()
                        if not is_active:
                            # Penalty for scoring in inactive hub (valid but bad strategy)
                            inactive_penalty = -0.5
                    except KeyError:
                        pass

            rewards[name] = pickup_reward + inactive_penalty
            
        # Update prev
        self.prev_fuel = current_fuel
        
        return rewards
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
            max_episode_steps=200,  # Allow longer episodes for strategic learning
        )
        
        # Assign custom reward function
        env.config.reward_fn = RebuiltRewardFunction()
        
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
    
    # 4. Configure PPO with optimized hyperparameters for overnight training
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
            # Learning rate with decay
            lr=3e-4,
            # Discount factor - high for strategic games
            gamma=0.99,
            # GAE lambda for advantage estimation
            lambda_=0.95,
            # Larger batch for more stable gradients
            train_batch_size=8000,
            # Mini-batch size
            minibatch_size=512,
            # Number of SGD epochs per batch
            num_epochs=10,
            # PPO clip range
            clip_param=0.2,
            # Entropy bonus for exploration (IMPORTANT!)
            entropy_coeff=0.01,
            # Value function loss coefficient
            vf_loss_coeff=0.5,
            # Gradient clipping
            grad_clip=0.5,
        )
        .env_runners(
            num_env_runners=num_workers,
            num_envs_per_env_runner=2,
            # Longer rollouts for strategic learning
            rollout_fragment_length=200,
        )
        .rl_module(
            # Larger network for complex multi-agent strategy
            # Obs space: ~40+ dims (robot state + capabilities + 3 opponents + game state)
            # Action space: 51 discrete actions
            model_config={
                "fcnet_hiddens": [512, 512, 512],  # Deeper for strategic reasoning
                "fcnet_activation": "relu",
                "vf_share_layers": False,  # Separate value network for stability
            }
        )
        .callbacks(GameMetricsCallback)
    )
    
    algo = config.build()
    
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
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--robots", type=int, default=3)
    args = parser.parse_args()
    
    train_versatile(args.iterations, args.robots, args.workers)
