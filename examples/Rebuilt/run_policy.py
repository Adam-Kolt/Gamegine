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
    draw_alliance_zone, draw_neutral_zone,
    AnimationLayer, ProjectileAnimation, draw_animation_layer
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
            use_server_pool=True,
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

    register_env("rebuilt-versatile-v0", env_creator)
    
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
            .environment(env="rebuilt-versatile-v0")
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
        
        # Update animation layer if rendering
        if render and "animation_layer" in state:
            state["animation_layer"].update(delta_time)
            
            # Trigger animations based on action results
            from gamegine.utils.NCIM.Dimensions.spatial import Inch, Feet
            import random
            
            for agent_id, info in infos.items():
                if agent_id == "__all__":
                    continue
                    
                action_name = info.get("action_name", "")
                action_valid = info.get("action_valid", False)
                robot_name = info.get("robot_name", agent_id)
                
                if not action_valid:
                    continue
                    
                robot_state = env.server.game_state.get_robot(robot_name)
                if not robot_state:
                    continue
                
                robot_x = robot_state.x.get() if hasattr(robot_state.x, 'get') else robot_state.x
                robot_y = robot_state.y.get() if hasattr(robot_state.y, 'get') else robot_state.y
                
                # Detect pickup actions - animate ball coming TO robot
                if "pickup" in action_name:
                    # Get interactable position (rough estimate from field layout)
                    # Animate from a nearby zone toward robot
                    start_x = robot_x + Feet(random.uniform(-3, 3))
                    start_y = robot_y + Feet(random.uniform(-3, 3))
                    arc_offset = random.uniform(-30, 30)  # Curve the ball
                    anim = ProjectileAnimation(
                        (start_x, start_y), (robot_x, robot_y),
                        duration=0.3, arc_offset=arc_offset
                    )
                    state["animation_layer"].add(anim)
                
                # Detect shoot/score actions - animate ball FROM robot
                elif "shoot" in action_name or "score" in action_name:
                    # Animate from robot toward hub (estimate hub position)
                    if "red" in robot_name.lower():
                        hub_x = Feet(2)  # Red Hub is on left
                    else:
                        hub_x = Feet(52)  # Blue Hub is on right
                    hub_y = Feet(13.5)  # Field center
                    arc_offset = random.uniform(-40, 40)  # Curve the ball outward
                    anim = ProjectileAnimation(
                        (robot_x, robot_y), (hub_x, hub_y),
                        duration=0.5, arc_offset=arc_offset
                    )
                    state["animation_layer"].add(anim)
        
        # Detailed step logging (always in no-render mode, every step)
        if not render:
            game_state = env.server.game_state
            match_time = game_state.current_time.get() if hasattr(game_state, 'current_time') else 0
            red_score = game_state.red_score.get()
            blue_score = game_state.blue_score.get()
            
            print(f"\n{'='*60}")
            print(f"STEP {state['step_count']} | Match Time: {match_time:.1f}s | Score: RED {red_score} - BLUE {blue_score}")
            print(f"{'='*60}")
            
            # Print each agent's action and reward
            for agent_id in sorted(sanitized_actions.keys()):
                action_idx = sanitized_actions[agent_id]
                reward = rewards.get(agent_id, 0.0)
                info = infos.get(agent_id, {})
                action_name = info.get("action_name", f"Action[{action_idx}]")
                action_valid = info.get("action_valid", True)
                valid_str = "✓" if action_valid else "✗"
                
                # Get robot state
                robot_name = info.get("robot_name", agent_id)
                robot_state = env.server.game_state.get_robot(robot_name)
                fuel_count = 0
                if robot_state:
                    fuel_dict = robot_state.gamepieces.get()
                    from examples.Rebuilt.scoring import Fuel
                    fuel_count = fuel_dict.get(Fuel, 0)
                
                print(f"  {agent_id:12} | {action_name:30} [{valid_str}] | Reward: {reward:+.2f} | Fuel: {fuel_count}")
            
            print(f"  {'─'*56}")
            print(f"  Total Rewards: RED={sum(rewards.get(k, 0) for k in rewards if 'red' in k.lower()):.2f}, "
                  f"BLUE={sum(rewards.get(k, 0) for k in rewards if 'blue' in k.lower()):.2f}")

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
            import math
            import arcade
            from gamegine.render.renderer import ObjectRendererRegistry
            from gamegine.utils.NCIM.Dimensions.spatial import Inch
            from gamegine.utils.NCIM.Dimensions.angular import Degree
            
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
            
            # Add bump zones
            for zone in game.get_zones():
                renderer.add(zone)
            
            # Add all interactables (Hub, Tower, Depot, etc)
            for interactable in game.get_interactables():
                renderer.add(interactable)
            
            # PolicyRobotDisplay class - draws all robots from env
            class PolicyRobotDisplay:
                def __init__(self, env_ref):
                    self._env = env_ref
            
            def draw_policy_robots(display_ref, canvas, theme, display_level, renderer_ref=None):
                """Draw all robots from the env's game state."""
                env_instance = display_ref._env
                if env_instance is None:
                    return
                
                game_state = env_instance.server.match.game_state
                
                for robot_name in env_instance.server.robots.keys():
                    robot_state = game_state.get_robot(robot_name)
                    if robot_state is None:
                        continue
                    
                    # Get position
                    x = canvas.to_pixels(robot_state.x.get() if hasattr(robot_state.x, 'get') else robot_state.x)
                    y = canvas.to_pixels(robot_state.y.get() if hasattr(robot_state.y, 'get') else robot_state.y)
                    
                    # Draw robot as colored square
                    robot_size = canvas.to_pixels(Inch(28))
                    half = robot_size / 2
                    
                    heading = float((robot_state.heading.get() if hasattr(robot_state.heading, 'get') else robot_state.heading).to(Degree))
                    rad = math.radians(heading)
                    cos_h, sin_h = math.cos(rad), math.sin(rad)
                    
                    # Compute rotated corners
                    corners = []
                    for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
                        rx = dx * half
                        ry = dy * half
                        corners.append((
                            x + rx * cos_h - ry * sin_h,
                            y + rx * sin_h + ry * cos_h
                        ))
                    
                    # Color based on alliance
                    if "red" in robot_name.lower():
                        color = (220, 53, 69)  # Red
                    else:
                        color = (0, 123, 255)  # Blue
                    
                    arcade.draw_polygon_filled(corners, (*color, 200))
                    arcade.draw_polygon_outline(corners, (*color, 255), 2)
                    
                    # Draw robot name
                    arcade.draw_text(
                        robot_name, x, y + robot_size * 0.7,
                        (255, 255, 255), 10, anchor_x="center"
                    )
                    
                    # Draw fuel count
                    from examples.Rebuilt.scoring import Fuel
                    fuel_dict = robot_state.gamepieces.get() if hasattr(robot_state.gamepieces, 'get') else {}
                    fuel_count = fuel_dict.get(Fuel, 0)
                    arcade.draw_text(
                        f"Fuel: {fuel_count}", x, y - robot_size * 0.7,
                        (255, 255, 0), 9, anchor_x="center"
                    )
            
            # Register the robot display
            ObjectRendererRegistry.register_handler(PolicyRobotDisplay, draw_policy_robots)
            robot_display = PolicyRobotDisplay(env)
            renderer.add(robot_display)
            
            # Add animation layer for ball animations
            ObjectRendererRegistry.register_handler(AnimationLayer, draw_animation_layer)
            animation_layer = AnimationLayer()
            renderer.add(animation_layer)
            
            # Store refs in state for reset updates and animations
            state["renderer_ref"] = renderer
            state["robot_names"] = list(env.server.robots.keys())
            state["animation_layer"] = animation_layer
            
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
