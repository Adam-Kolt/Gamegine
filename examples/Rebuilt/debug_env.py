
import argparse
import sys
import os
import numpy as np

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from gamegine.first.alliance import Alliance
from gamegine.rl import make_alliance_env, AllianceEnv
from gamegine.representation.capabilities import RobotCapabilities
from gamegine.utils.NCIM.ncim import Inch, Degree
from examples.Rebuilt.Rebuilt import create_rebuilt_game, FIELD_WIDTH, FIELD_LENGTH
from examples.Rebuilt.robot import create_robot, setup_robot_interactions
from examples.Rebuilt.scoring import Fuel
from examples.Rebuilt.train_versatile import create_robot_configs, CAPABILITY_RANGES, INTERACTION_RANGES, PHYSICAL_RANGES

def debug_env():
    print("=" * 60)
    print("Debug: Observations and Action Masking")
    print("=" * 60)
    
    # 1. Setup Environment (Copying config from train_versatile.py)
    num_robots = 2 # 2 vs 2 for debug
    game = create_rebuilt_game()
    red_robots = create_robot_configs(Alliance.RED, game, num_robots)
    blue_robots = create_robot_configs(Alliance.BLUE, game, num_robots)
    
    env = make_alliance_env(
        game=game,
        red_robots=red_robots,
        blue_robots=blue_robots,
        mode="self_play",
        fast_mode=True, 
        use_server_pool=False,
    )
    
    # Enable versatile training features
    env.training_config.use_capability_context = True
    env.training_config.randomize_capabilities = True
    env.training_config.capability_ranges = CAPABILITY_RANGES
    env.training_config.interaction_ranges = INTERACTION_RANGES
    env.training_config.physical_ranges = PHYSICAL_RANGES
    
    # Enable opponent/teammate awareness
    env.training_config.observe_opponent_states = True
    env.training_config.observe_opponent_capabilities = True 
    env.training_config.observe_teammate_states = True
    env.training_config.observe_teammate_capabilities = True
    
    # Action Masking
    env.training_config.use_action_masking = True
    
    # Force rebuild of spaces
    env._observation_spaces = {}
    for agent_id in env._agent_ids:
        robot_name = env._agent_to_robot[agent_id]
        env._observation_spaces[agent_id] = env._build_observation_space(robot_name)
    
    # 2. Reset and Inspect
    print("\n--- Resetting Environment ---")
    obs, infos = env.reset()
    
    # Pick one agent to inspect (Red 0)
    agent_id = "red_0"
    if agent_id not in obs:
        agent_id = list(obs.keys())[0]
        
    print(f"Inspecting Agent: {agent_id}")
    robot_name = env._agent_to_robot[agent_id]
    print(f"Robot Name: {robot_name}")
    
    # Interpret Observation
    raw_obs = obs[agent_id]
    print(f"\nObservation Vector Size: {len(raw_obs)}")
    
    # Deconstruct Obs (based on AllianceEnv._get_observation)
    # Standard: 10
    # Semi-Markov: 2
    # Caps: 10
    # Game: 5 (Neutral, BlueDepot, RedDepot, BlueZone, RedZone)
    # Extended: 3 (Fuel, OwnHub, OppHub)
    # Opponents: 3 * (5 state + 10 caps) = 45
    # Teammates: 1 * (6 state + 10 caps) = 16 (assuming 2 robots total per alliance)
    
    # Let's print the first few standard values
    print("\n--- Standard Observations ---")
    print(f"Position (norm): ({raw_obs[0]:.2f}, {raw_obs[1]:.2f})")
    print(f"Heading (norm):  {raw_obs[2]:.2f}")
    print(f"Time (norm):     {raw_obs[5]:.2f}")
    print(f"Red Score:       {raw_obs[6]:.2f}")
    print(f"Blue Score:      {raw_obs[7]:.2f}")
    
    # Semi-Markov
    print(f"Is Busy:         {raw_obs[10]}")
    print(f"Time Remaining:  {raw_obs[11]}")
    
    # Extended (Fuel)
    # 3 capability context + 4 dummy + game piece capability... 
    # It's hard to calculate exact indices dynamically without the code logic.
    # But we know Extended Obs (Fuel, Hubs) are appended after Game Obs.
    # Game Obs are appended after Capability Context.
    
    # Let's just inspect the action mask directly
    print("\n--- Action Mask Inspection ---")
    mask = infos[agent_id].get("action_mask")
    if mask is None:
        print("No action mask found in infos!")
        mask = env._get_action_mask(robot_name)
    
    action_map = env._action_maps[robot_name]
    
    print(f"Mask Size: {len(mask)}")
    print(f"{'IDX':<3} | {'VALID':<5} | {'ACTION (Abstract, Interaction)':<50}")
    print("-" * 70)
    
    for i, (is_valid, action) in enumerate(zip(mask, action_map)):
        status = "YES" if is_valid > 0.5 else " NO"
        print(f"{i:<3} | {status:<5} | {str(action):<50}")
        
    # Verify why specific actions are masked
    print("\n--- State Context for Masking ---")
    robot = env.server.game_state.get_robot(robot_name)
    fuel_count = sum(robot.gamepieces.get().values()) if robot and robot.gamepieces else 0
    print(f"Robot Inventory: {fuel_count} fuel")
    
    # Test: Give robot fuel and see if mask updates
    print("\n--- Testing Mask Update (Adding Fuel) ---")
    if robot:
        current_inv = robot.gamepieces.get().copy()
        current_inv[Fuel] = 5 # Full
        robot.gamepieces.set(current_inv)
        
    mask_full = env._get_action_mask(robot_name)
    print(f"New Inventory: {sum(robot.gamepieces.get().values())}")
    
    print(f"{'IDX':<3} | {'VALID':<5} | {'ACTION':<50}")
    print("-" * 70)
    for i, (is_valid, action) in enumerate(zip(mask_full, action_map)):
        status = "YES" if is_valid > 0.5 else " NO"
        # Only print changed or interesting ones
        if status != ("YES" if mask[i] > 0.5 else " NO"):
             print(f"{i:<3} | {status:<5} | {str(action):<50} <--- CHANGED")
        elif "score" in str(action) or "pickup" in str(action):
             print(f"{i:<3} | {status:<5} | {str(action):<50}")

    print("\n--- Testing Mask Update (Empty Inventory) ---")
    if robot:
        current_inv = robot.gamepieces.get().copy()
        current_inv[Fuel] = 0 # Empty
        robot.gamepieces.set(current_inv)
        
    mask_empty = env._get_action_mask(robot_name)
    print(f"New Inventory: {sum(robot.gamepieces.get().values())}")
    
    print(f"{'IDX':<3} | {'VALID':<5} | {'ACTION':<50}")
    print("-" * 70)
    for i, (is_valid, action) in enumerate(zip(mask_empty, action_map)):
        status = "YES" if is_valid > 0.5 else " NO"
        # Only print score/pickup/shuttle
        if "score" in str(action) or "pickup" in str(action) or "shuttle" in str(action):
             print(f"{i:<3} | {status:<5} | {str(action):<50}")

    print("\nDone.")

if __name__ == "__main__":
    debug_env()
