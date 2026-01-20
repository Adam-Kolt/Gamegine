"""
Run Policy Script for Rebuilt Game.

Loads a trained RLlib checkpoint and visualizes the policy controlling agents in the AllianceEnv.
"""

import argparse
import logging
import os
import sys
import warnings
import time

# Suppress logging noise
os.environ["RAY_DEDUP_LOGS"] = "1"
os.environ["RAY_SCHEDULER_EVENTS"] = "0"
os.environ["PYTHONWARNINGS"] = "ignore::DeprecationWarning"
warnings.filterwarnings("ignore", category=DeprecationWarning)
logging.getLogger("ray").setLevel(logging.ERROR)

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import ray
from ray.rllib.algorithms.algorithm import Algorithm
from ray.tune.registry import register_env

from gamegine.first.alliance import Alliance
from gamegine.rl import (
    make_alliance_env,
    AllianceEnv,
)
from gamegine.render.renderer import Renderer, DisplayLevel
from examples.Rebuilt.Rebuilt import create_rebuilt_game
from examples.Rebuilt.train_versatile import create_robot_configs
from examples.Rebuilt.scoring import Hub, Tower, Depot, AllianceZone, NeutralZone
from examples.Rebuilt.shooting_locations import ShootingLocation
from examples.Rebuilt.discrete_action_demo import (
    draw_shooting_location, draw_depot, draw_hub, draw_tower, 
    draw_alliance_zone, draw_neutral_zone
)
from gamegine.utils.logging import SetLoggingLevel

SetLoggingLevel(logging.FATAL)

def run_policy(
    checkpoint_path: str,
    red_robots: int = 2,
    blue_robots: int = 2,
    render: bool = True,
):
    """Load policy and run inference loop with visualization."""
    print(f"Loading checkpoint from {checkpoint_path}...")
    
    # Initialize Ray (needed for Algorithm.from_checkpoint)
    if not ray.is_initialized():
        ray.init(ignore_reinit_error=True)
    
    # helper for env creation (must match training exactly)
    def env_creator(cfg):
        game = create_rebuilt_game()
        
        red_configs = create_robot_configs(Alliance.RED, game, red_robots)
        blue_configs = create_robot_configs(Alliance.BLUE, game, blue_robots)
        
        env = make_alliance_env(
            game=game,
            red_robots=red_configs,
            blue_robots=blue_configs,
            mode="self_play",
            fast_mode=True, # Use fast mode for discrete stepping visualization
            use_server_pool=False,
            max_episode_steps=500, # Longer visualization
        )
        
        # Enable features (must match training)
        env.training_config.use_capability_context = True
        env.training_config.randomize_capabilities = True
        env.training_config.observe_opponent_states = True
        env.training_config.observe_opponent_capabilities = True
        
        # Capability ranges (doesn't strictly matter for inference as they are randomized anyway, 
        # but needed for valid generation)
        env.training_config.capability_ranges = {
             "max_speed": (1.5, 5.5),
             "max_acceleration": (1.0, 6.0),
             "rotational_speed": (3.0, 10.0),
        }

        # Force rebuild of spaces to match new config
        env._observation_spaces = {}
        for agent_id in env._agent_ids:
            robot_name = env._agent_to_robot[agent_id]
            env._observation_spaces[agent_id] = env._build_observation_space(robot_name)
        
        from gymnasium import spaces
        env.observation_space = spaces.Dict(env._observation_spaces)
        
        return env

    register_env("reefscape-versatile-v0", env_creator)
    
    # Setup Algorithm
    if checkpoint_path and checkpoint_path.lower() != "none":
        print(f"Loading checkpoint from {checkpoint_path}...")
        try:
            algo = Algorithm.from_checkpoint(checkpoint_path)
        except Exception as e:
            print(f"Error loading checkpoint: {e}")
            return
    else:
        print("No checkpoint provided. Running with UNTRAINED (random) policy.")
        # Build a fresh algorithm structure to use for inference
        # We need to construct PPO config similar to training
        from ray.rllib.algorithms.ppo import PPOConfig
        from ray.rllib.policy.policy import PolicySpec

        # Inspect temp env for spaces
        temp_env = env_creator({})
        agent_id = list(temp_env.get_agent_ids())[0]
        obs_space = temp_env.observation_space[agent_id]
        act_space = temp_env.action_space[agent_id]
        temp_env.close()

        config = (
            PPOConfig()
            .api_stack(
                enable_rl_module_and_learner=False,
                enable_env_runner_and_connector_v2=False,
            )
            .environment(env="reefscape-versatile-v0")
            .multi_agent(
                 policies={
                    "versatile_policy": PolicySpec(
                        observation_space=obs_space,
                        action_space=act_space,
                    ),
                },
                policy_mapping_fn=lambda aid, *args, **kwargs: "versatile_policy",
            )
        )
        algo = config.build()

    # Create visualization env
    env = env_creator({})
    obs, info = env.reset()
    
    print("\nStarting Inference Loop (Press Ctrl+C to stop)...")

    # State container for closure
    state = {
        "obs": obs,
        "step_count": 0
    }

    def inference_update(delta_time):
        actions = None
        # We can implement a slowdown here if needed, or rely on update_rate
        # For now, let's just step every frame (which might be fast)
        # Or add a manual delay
        
        # Compute Actions
        # Check if algorithm is using the new API stack (EnvRunner)
        # Note: 'enable_env_runner_and_connector_v2' is a good indicator
        is_new_stack = algo.config.enable_env_runner_and_connector_v2
        
        if is_new_stack:
            try:
                # New API Stack: Uses RLModule
                import torch
                import numpy as np
                
                module = algo.get_module("versatile_policy") if hasattr(algo, "get_module") else algo.module
                
                # Prepare batch input
                # Multi-agent input: Dict[str, Any]
                # We need to construct a batch for each agent
                actions = {}
                
                # We can batch all agents together for efficiency
                # Create a batch of observations
                obs_list = []
                agent_ids = []
                
                for agent_id, agent_obs in state["obs"].items():
                    obs_list.append(agent_obs)
                    agent_ids.append(agent_id)
                
                if obs_list:
                    # Convert to tensor
                    obs_tensor = torch.as_tensor(np.array(obs_list), dtype=torch.float32)
                    
                    # Forward pass
                    # RLModule inputs: {"obs": ...}
                    # RLModule outputs: {"action_dist_inputs": ...} or similar depending on catalog
                    
                    # We need to use `forward_inference` or `forward_exploration`
                    with torch.no_grad():
                        # Forward pass
                        outputs = module.forward_inference({"obs": obs_tensor})

                        # Handle RLModule outputs
                        if "action_dist_inputs" in outputs:
                            logits = outputs["action_dist_inputs"]
                            # Deterministic: Argmax
                            action_batch = torch.argmax(logits, dim=-1)
                            actions = {
                                aid: int(act.item()) 
                                for aid, act in zip(agent_ids, action_batch)
                            }
                        elif "actions" in outputs:
                            action_batch = outputs["actions"]
                            actions = {
                                aid: int(act.item()) if hasattr(act, "item") else int(act)
                                for aid, act in zip(agent_ids, action_batch)
                            }
                        else:
                            raise RuntimeError(f"Unknown RLModule output keys: {outputs.keys()}")
                else:
                    actions = {}
            
            except Exception as e:
                 print(f"Manual inference failed: {e}")
                 import traceback
                 traceback.print_exc()
                 # Fallback/Workaround for New API Stack Inference Issues
                 # We will use the 'compute_actions' which works if we FORCE the old API stack
                 # But we can't change the loaded checkpoint type easily.
                 
                 # The user says: "AttributeError: ... get_policy"
                 
                 # Let's try assuming the attribute is just misplaced.
                 pass

        # Use standard compute_actions (works for old stack) if not already computed
        if actions is None:
            try:
                 actions = algo.compute_actions(
                    observations=state["obs"],
                    policy_id="versatile_policy",
                    explore=False
                )
            except AttributeError:
                 # NEW API STACK WORKAROUND:
                 # If algo.compute_actions fails, it's because it tries to use EnvRunner.
                 # We can manually invoke the underlying RLModule if accessible.
                 
                 import torch
                 import numpy as np
                 
                 module = algo.get_module("versatile_policy") if hasattr(algo, "get_module") else algo.module
                 
                 # Prepare batch
                 obs_list = []
                 agent_key_list = []
                 for agent_id, agent_obs in state["obs"].items():
                     obs_list.append(agent_obs)
                     agent_key_list.append(agent_id)
                 
                 if obs_list:
                     obs_tensor = torch.as_tensor(np.array(obs_list), dtype=torch.float32)
                     with torch.no_grad():
                         # Forward pass
                         outputs = module.forward_inference({"obs": obs_tensor})

                         # Handle RLModule outputs
                         if "action_dist_inputs" in outputs:
                            logits = outputs["action_dist_inputs"]
                            # Deterministic: Argmax
                            action_batch = torch.argmax(logits, dim=-1)
                            actions = {
                                aid: int(act.item()) 
                                for aid, act in zip(agent_key_list, action_batch)
                            }
                         elif "actions" in outputs:
                             action_batch = outputs["actions"]
                             actions = {
                                aid: int(act.item()) if hasattr(act, "item") else int(act)
                                for aid, act in zip(agent_key_list, action_batch)
                            }
                         else:
                            raise RuntimeError(f"Unknown RLModule output keys: {outputs.keys()}")
                 else:
                     actions = {}
        
        # Step Env
        # Ensure fallback actions are also handled if needed (though compute_actions usually OK)
        # But if actions is array from compute_actions?
        # Let's sanitize actions dict just in case
        sanitized_actions = {}
        for k, v in actions.items():
            if hasattr(v, "item"):
                 sanitized_actions[k] = int(v.item())
            elif isinstance(v, (np.ndarray, list)):
                 sanitized_actions[k] = int(v[0]) if len(v) > 0 else 0
            else:
                 sanitized_actions[k] = int(v)
                 
        obs, rewards, terminateds, truncateds, infos = env.step(sanitized_actions)
        state["obs"] = obs
        state["step_count"] += 1
        
        if not render and state["step_count"] % 10 == 0:
             print(f"Step {state['step_count']} | Rewards: {rewards}")

        if terminateds.get("__all__", False) or truncateds.get("__all__", False):
            print(f"Episode finished. Terminated: {terminateds.get('__all__')}, Truncated: {truncateds.get('__all__')}")
            # Check if timer exists safe access
            timer_time = "N/A"
            if hasattr(env.server.match, "timer") and env.server.match.timer:
                 timer_time = env.server.match.timer.time
            print(f"Match Time: {timer_time}")
            state["obs"], _ = env.reset()
            
            # Update Renderer logic if needed
            if render and "renderer" in locals():
                # We need to remove old robots and add new ones
                # We can try to access the renderer from outer scope
                # Assuming renderer is available or passed in?
                # Actually, 'renderer' variable is defined below. 
                # Closures should capture it if we are careful.
                pass
                
            # Actually, to avoid complexity with stale 'renderer' variable or closure issues,
            # We can use a trick: env.server updates, but we need to tell renderer about new objects.
            # Best way: access renderer from state or if render is True.
            
            # Let's use a mutable list for current robots in 'state'
            if render and "robot_refs" in state:
                # Remove old
                for r in state["robot_refs"]:
                    state["renderer_ref"].remove(r)
                
                # Add new
                new_robots = list(env.server.robots.values())
                for r in new_robots:
                     state["renderer_ref"].add(r)
                state["robot_refs"] = new_robots

    if render:
        try:
            from gamegine.render.renderer import ObjectRendererRegistry
            
            # Register INTERACTABLE handlers for visualization
            ObjectRendererRegistry.register_handler(ShootingLocation, draw_shooting_location)
            ObjectRendererRegistry.register_handler(Depot, draw_depot)
            ObjectRendererRegistry.register_handler(Hub, draw_hub)
            ObjectRendererRegistry.register_handler(Tower, draw_tower)
            ObjectRendererRegistry.register_handler(AllianceZone, draw_alliance_zone)
            ObjectRendererRegistry.register_handler(NeutralZone, draw_neutral_zone)
            
            # Create renderer with game instance
            game = env.server.match.game
            renderer = Renderer.create(game=game, width=1200, height=600)
            renderer.display_level = DisplayLevel.SHOWCASE
            
            # Add obstacles
            for obs in game.get_obstacles():
                renderer.add(obs)
            
            # Add all interactables (Hub, Tower, Depot, etc)
            for interactable in game.get_interactables():
                renderer.add(interactable)
            
            # Add dynamic robot states (rendered each frame from current game state)
            # Robot states stored at game_state.get_robot(name), which returns RobotState
            for robot_name in env.server.robots.keys():
                # Create closure to capture robot_name properly
                def make_robot_getter(name):
                    def get_robot_state():
                        return env.server.match.game_state.get_robot(name)
                    return get_robot_state
                renderer.add_dynamic(make_robot_getter(robot_name), "robot")
            
            # Store refs in state for reset updates
            state["renderer_ref"] = renderer
            state["robot_names"] = list(env.server.robots.keys())
            
            # Register update callback
            renderer.on_update_callback(inference_update)
            
            # Run arcade loop
            renderer.run()
        except KeyboardInterrupt:
             pass
    else:
        # Manual loop if no render
        try:
            while True:
                inference_update(0.1)
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
            
    print("Stopping...")
    env.close()
    ray.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=str, default="None", help="Path to checkpoint directory (or 'None' for random)")
    parser.add_argument("--red-robots", type=int, default=3)
    parser.add_argument("--blue-robots", type=int, default=3)
    parser.add_argument("--no-render", action="store_true", help="Disable visualization")
    args = parser.parse_args()
    
    run_policy(
        args.checkpoint, 
        args.red_robots,
        args.blue_robots,
        render=not args.no_render
    )
