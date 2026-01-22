"""
Rainbow DQN Training for Rebuilt 2026.

Implements the "Rainbow" flavor of DQN, combining:
- Double Q-Learning
- Dueling Network Architecture
- Noisy Networks for Exploration
- Distributional RL (C51)
- Multi-step Returns (N-step)
- Prioritized Experience Replay (PER)

Usage:
    python train_rainbow.py --iterations 200 --workers 4
"""

import argparse
import logging
import os
import sys
import warnings
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
from ray.rllib.algorithms.dqn import DQNConfig
from ray.rllib.algorithms.algorithm import Algorithm
from ray.rllib.utils.replay_buffers.replay_buffer import ReplayBuffer
from ray.rllib.utils.from_config import from_config
from ray.tune.registry import register_env
from ray.rllib.policy.policy import PolicySpec

# === MONKEY PATCH START ===
# Patching RLlib bug where buffer config type is a Class but checked as String
def patched_create_local_replay_buffer_if_necessary(self, config):
    """Create a MultiAgentReplayBuffer instance if necessary."""
    if not config.get("replay_buffer_config") or config["replay_buffer_config"].get(
        "no_local_replay_buffer"
    ):
        return

    # BUG FIX: Handle type being a Class object
    buffer_type = config["replay_buffer_config"].get("type")
    type_name = buffer_type
    if isinstance(buffer_type, type):
        type_name = buffer_type.__name__
        
    if "EpisodeReplayBuffer" in str(type_name):
        config["replay_buffer_config"][
            "metrics_num_episodes_for_smoothing"
        ] = self.config.metrics_num_episodes_for_smoothing

    return from_config(ReplayBuffer, config["replay_buffer_config"])

# Apply Patch
Algorithm._create_local_replay_buffer_if_necessary = patched_create_local_replay_buffer_if_necessary
print("Applied monkey patch to Algorithm._create_local_replay_buffer_if_necessary")
# === MONKEY PATCH END ===


from ray.rllib.algorithms.callbacks import DefaultCallbacks

class GameMetricsCallback(DefaultCallbacks):
    """Custom callback to log game-specific metrics during training."""
    
    def on_episode_end(self, *, episode, env_runner=None, metrics_logger=None, env=None, env_index=None, rl_module=None, **kwargs):
        """Called at the end of each episode to record custom metrics."""
        try:
            # Handle Old API Stack (worker passed in kwargs or as named arg if strict signature used)
            worker = kwargs.get("worker")
            if env_runner is None and worker is not None:
                env_runner = worker
            
            # Access the underlying AllianceEnv
            actual_env = env
            if actual_env is None and env_runner is not None:
                if hasattr(env_runner, "env"):
                    actual_env = env_runner.env
            
            # Unwrap envs (handle vector envs, wrappers)
            if actual_env is None:
                # In Old API, base_env might be passed
                base_env = kwargs.get("base_env")
                if base_env:
                    actual_env = base_env

            if actual_env is None:
                # print("DEBUG: Could not find env in callback")
                return

            # Drill down to find our AllianceEnv
            # Common wrappers: VectorEnv, Normalize, Monitor, etc.
            # We want to find the one with .server
            
            current = actual_env
            found = False
            
            # Simple recursive search or iterative drill-down
            for _ in range(10): # Depth limit
                if hasattr(current, "server") and current.server is not None:
                     actual_env = current
                     found = True
                     break
                
                # Check for wrapped envs
                if hasattr(current, "envs"): # VectorEnv
                     current = current.envs[env_index if env_index is not None else 0]
                elif hasattr(current, "env"): # Gym Wrapper
                     current = current.env
                elif hasattr(current, "_env"): # Private wrapper
                     current = current._env
                else:
                     break
            
            if not found:
                # One last try check if actual_env IS the vector env and we can get sub-env
                if hasattr(actual_env, "get_sub_environments"):
                     subs = actual_env.get_sub_environments()
                     if subs and len(subs) > (env_index or 0):
                          sub = subs[env_index or 0]
                          if hasattr(sub, "server"):
                               actual_env = sub
                               found = True

            if found:
                game_state = actual_env.server.match.game_state
                red_score = game_state.red_score.get()
                blue_score = game_state.blue_score.get()
                
                # Old API stack: use episode.custom_metrics
                episode.custom_metrics["red_score_mean"] = red_score # Use keys expected by main loop extraction
                episode.custom_metrics["blue_score_mean"] = blue_score
                episode.custom_metrics["score_diff"] = red_score - blue_score
                # Note: RLlib automatically appends _mean to these when rolling up, 
                # but if we access episode.custom_metrics["red_score"], ray aggregates it.
                # In main loop we look for "red_score_mean".
                # If we log "red_score", RLlib output will show "custom_metrics/red_score_mean".
                
                # Let's log just "red_score" here, RLlib handles the mean.
                episode.custom_metrics["red_score"] = red_score
                episode.custom_metrics["blue_score"] = blue_score
                
            else:
                 pass
                 # print(f"DEBUG: Server not found in env: {type(actual_env)}")
                
        except Exception as e:
            print(f"Callback Error: {e}")
            pass


from gamegine.first.alliance import Alliance
from gamegine.rl import make_alliance_env, AllianceEnv, RobotConfig
from gamegine.utils.logging import SetLoggingLevel

from examples.Rebuilt.Rebuilt import create_rebuilt_game
from examples.Rebuilt.robot import create_robot, setup_robot_interactions
from examples.Rebuilt.scoring import Fuel
from examples.Rebuilt.reward_function import AdvancedRebuiltRewardFunction
from examples.Rebuilt.train_versatile import (
    create_robot_configs, 
    CAPABILITY_RANGES, 
    INTERACTION_RANGES, 
    PHYSICAL_RANGES
)

SetLoggingLevel(logging.FATAL)

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
        max_episode_steps=500,  # Ensure full match duration
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
    
    # Enable opponent/teammate awareness
    env.training_config.observe_opponent_states = True
    env.training_config.observe_opponent_capabilities = True
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


def train_rainbow(
    iterations: int = 100, 
    num_robots: int = 2, 
    num_workers: int = 0, # 0 = Auto-detect
    checkpoint_freq: int = 10,
    restore_path: str = None,
):
    """Run the Rainbow DQN training loop."""
    print("=" * 60)
    print("Rainbow DQN Training (Rebuilt 2026)")
    print("Features: Double Q, Dueling, Noisy, Distributional, N-Step, PER")
    print("=" * 60)
    
    # Auto-detect workers if 0
    if num_workers == 0:
        num_workers = max(1, multiprocessing.cpu_count() - 2)
        print(f"Auto-detected {multiprocessing.cpu_count()} cores. Using {num_workers} workers.")
    
    ray.init(ignore_reinit_error=True)

    register_env("reefscape-rainbow-v0", env_creator)
    
    # Inspect Environment
    print("Inspecting environment...")
    temp_env = env_creator({"num_robots": num_robots})
    agent_id = list(temp_env.get_agent_ids())[0]
    obs_space = temp_env.observation_space[agent_id]
    act_space = temp_env.action_space[agent_id]
    temp_env.close()
    
    print(f"Observation Shape: {obs_space.shape}")
    print(f"Action Space: {act_space.n}")
    
    # Configure Rainbow DQN
    config = (
        DQNConfig()
        .environment(env="reefscape-rainbow-v0", env_config={"num_robots": num_robots})
        .multi_agent(
            policies={
                "rainbow_policy": PolicySpec(
                    policy_class=None,
                    observation_space=obs_space,
                    action_space=act_space,
                    config={},
                ),
            },
            policy_mapping_fn=lambda agent_id, *args, **kwargs: "rainbow_policy",
            policies_to_train=["rainbow_policy"],
        )
        .api_stack(
            enable_rl_module_and_learner=False,
            enable_env_runner_and_connector_v2=False,
        )
        .framework("torch")
        .training(
            # === Rainbow Features ===
            double_q=True,
            dueling=True,
            num_atoms=51,
            v_min=-100.0,
            v_max=200.0,
            noisy=True,
            n_step=3,
            
            # Prioritized Experience Replay (PER)
            # The type will resolve to a Class due to DQNConfig logic, but our Monkey Patch handles it.
            replay_buffer_config={
                "type": "MultiAgentPrioritizedReplayBuffer",
                "capacity": 50000, 
                "prioritized_replay_alpha": 0.6,
                "prioritized_replay_beta": 0.4,
                "prioritized_replay_eps": 1e-6,
            },
            
            gamma=0.99,
            lr=1e-4, # Slightly lower LR for stability
            train_batch_size=256, 
            target_network_update_freq=500,
            model={
                 "fcnet_hiddens": [256, 256], 
                 "fcnet_activation": "relu",
            }
        )
        .reporting(
            # Increase reporting interval to make iterations more meaningful
            min_sample_timesteps_per_iteration=1000,
        )
        .env_runners(
            num_env_runners=num_workers,
            num_envs_per_env_runner=4,
            rollout_fragment_length=50, 
            # Exploration Settings (Moved here due to deprecation)
            exploration_config={
                "type": "EpsilonGreedy",
                "initial_epsilon": 0.0,  # Let Noisy Nets handle exploration
                "final_epsilon": 0.0,
                "epsilon_timesteps": 1, 
            },
        )
        .experimental(_validate_config=False)
        .callbacks(GameMetricsCallback)
    )
    
    algo = config.build()
    
    if restore_path:
        print(f"Restoring from checkpoint: {restore_path}")
        algo.restore(restore_path)
    
    print(f"\nTraining for {iterations} iterations...")
    for i in range(iterations):
        result = algo.train()
        
        # Metrics
        env_runners = result.get("env_runners", {})
        mean_reward = env_runners.get("episode_return_mean", 0)
        episodes = env_runners.get("num_episodes", 0)
        
        ep_len = env_runners.get("episode_len_mean", 0)
        
        # Training stats (Old API Stack DQN structure)
        # Structure is usually info -> learner -> policies -> policy_id -> learner_stats
        learner_info = result.get("info", {}).get("learner", {}).get("rainbow_policy", {})
        
        # Loss and Q are often directly in policy dict for DQN Old API
        loss = learner_info.get("mean_td_error", 0.0) 
        if loss == 0.0:
            loss = learner_info.get("loss", 0.0)
            if loss == 0.0:
                 # Check nested learner info
                 loss = learner_info.get("learner_stats", {}).get("mean_td_error", 0.0)

        mean_q = learner_info.get("mean_q", 0.0)
        # Often Q is not reported directly in some versions, but let's check learner_stats just in case
        if mean_q == 0.0:
             mean_q = learner_info.get("learner_stats", {}).get("mean_q", 0.0)

        # Custom metrics (from GameMetricsCallback)
        custom_metrics = env_runners.get("custom_metrics", {})
        red_score = custom_metrics.get("red_score_mean", 0)
        blue_score = custom_metrics.get("blue_score_mean", 0)
        
        metrics_str = (f"Iter {i+1:3d} | R: {mean_reward:7.2f} | Ep: {episodes:3} | Len: {ep_len:5.1f} | "
                       f"Loss: {loss:.4f}")
        
        if mean_q != 0.0:
            metrics_str += f" | Q: {mean_q:.2f}"
            
        if red_score or blue_score:
            metrics_str += f" | Red:{red_score:.0f} Blue:{blue_score:.0f}"
            
        print(metrics_str)
        
        if (i+1) % checkpoint_freq == 0:
            checkpoint_dir = os.path.join(os.path.dirname(__file__), "checkpoints_rainbow")
            os.makedirs(checkpoint_dir, exist_ok=True)
            checkpoint = algo.save(checkpoint_dir)
         

    # Final Save
    checkpoint_dir = os.path.join(os.path.dirname(__file__), "checkpoints_rainbow")
    os.makedirs(checkpoint_dir, exist_ok=True)
    checkpoint = algo.save(checkpoint_dir)
   
    
    algo.stop()
    ray.shutdown()
    print("\nTraining complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--robots", type=int, default=3)
    parser.add_argument("--restore", type=str, default=None, help="Path to checkpoint to restore from")
    args = parser.parse_args()
    
    train_rainbow(args.iterations, args.robots, args.workers, restore_path=args.restore)
