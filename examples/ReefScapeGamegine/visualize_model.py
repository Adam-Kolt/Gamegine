
"""
Visualizer for Trained Alliance Models.

Loads a saved RLlib checkpoint and runs a visual simulation of the environment.
Usage:
    python visualize_model.py --mode solo --robots 1
    python visualize_model.py --checkpoint /path/to/checkpoint
"""
import argparse
import os
import sys
import glob
import time
import pygame
import ray
from ray.rllib.algorithms.algorithm import Algorithm

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from gamegine.rl import make_solo_env, make_self_play_env, RobotConfig
from gamegine.first.alliance import Alliance
from gamegine.simulation.robot import RobotState
from examples.ReefScapeGamegine.Reefscape import Reefscape
from examples.ReefScapeGamegine.ai_robot import SWERVE_ROBOT, init_robot_interaction
from examples.ReefScapeGamegine.marl_reefscape import MARLVisualizer
from gamegine.utils.NCIM.ncim import Inch, Degree, Meter
from gamegine.utils.logging import SetLoggingLevel
import logging

def find_latest_checkpoint(root_dir="~/ray_results"):
    root_dir = os.path.expanduser(root_dir)
    # Find PPO experiments
    experiments = glob.glob(os.path.join(root_dir, "PPO_*"))
    if not experiments:
        print(f"No experiments found in {root_dir}")
        return None
    
    # Sort by modification time
    latest_exp = max(experiments, key=os.path.getmtime)
    print(f"Found latest experiment: {os.path.basename(latest_exp)}")
    
    # Find checkpoints (recursive)
    checkpoints = glob.glob(os.path.join(latest_exp, "**", "checkpoint_*"), recursive=True)
    # Filter out temp files or non-directories if needed
    checkpoints = [c for c in checkpoints if os.path.isdir(c)]
    
    if not checkpoints:
        print("No checkpoints found in experiment.")
        return None
        
    latest_ckpt = max(checkpoints, key=os.path.getmtime)
    print(f"Latest checkpoint: {latest_ckpt}")
    return latest_ckpt

def create_robot_configs(alliance: Alliance, num_robots: int = 1):
    configs = []
    prefix = alliance.name.lower()
    for i in range(num_robots):
        start_state = RobotState(
            x=Inch(100),
            y=Inch(100 + i * 50),
            heading=Degree(0)
        )
        configs.append(RobotConfig(SWERVE_ROBOT, start_state, f"{prefix}_{i}", prefix))
    return configs

def main():
    parser = argparse.ArgumentParser(description="Visualize trained model")
    parser.add_argument("--checkpoint", type=str, help="Path to checkpoint directory (default: latest)")
    parser.add_argument("--mode", type=str, default="solo", choices=["solo", "self_play"])
    parser.add_argument("--robots", type=int, default=1, help="Robots per alliance")
    parser.add_argument("--fast", action="store_true", help="Use fast mode (teleport) for visualization")
    args = parser.parse_args()

    # Init Ray (needed for Algorithm)
    ray.init(ignore_reinit_error=True, log_to_driver=False)
    
    # Find checkpoint
    checkpoint_path = args.checkpoint
    if not checkpoint_path:
        checkpoint_path = find_latest_checkpoint()
        if not checkpoint_path:
            print("Could not find any checkpoints. Please specify with --checkpoint.")
            sys.exit(1)
    
    print(f"Loading checkpoint: {checkpoint_path}")
    algo = Algorithm.from_checkpoint(checkpoint_path)
    
    # Init robot assets
    init_robot_interaction()
    
    # Create environment
    print("Creating environment...")
    if args.mode == "solo":
        robots = create_robot_configs(Alliance.BLUE, args.robots)
        env = make_solo_env(
            game=Reefscape,
            robots=robots,
            alliance="blue",
            fast_mode=args.fast,
            use_server_pool=False, # No pool needed for single env
        )
    else:
        env = make_self_play_env(
            game=Reefscape,
            red_robots=create_robot_configs(Alliance.RED, args.robots),
            blue_robots=create_robot_configs(Alliance.BLUE, args.robots),
            fast_mode=args.fast,
            use_server_pool=False,
        )
        
    # Setup visualizer
    visualizer = MARLVisualizer(
        field_width=env.game.field.width.to(Meter),
        field_height=env.game.field.height.to(Meter),
        width=1200,
        height=600  # Adjust as needed (Reefscape is long?)
    )
    
    # Run loop
    print("Starting visualization (Press ESC to quit)...")
    try:
        obs, info = env.reset()
        running = True
        clock = pygame.time.Clock()
        total_reward = {id: 0.0 for id in env.possible_agents}
        
        while running:
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    running = False
            
            # Compute actions
            actions = {}
            # algo.compute_actions returns (actions, states, infos)
            # But specific signatue depends on usage.
            # Usually: algo.compute_actions(obs_dict) -> actions_dict
            
            # Note: RLlib policies expect raw observations, but sometimes need preprocessing.
            # Algorithm.compute_actions handles this usually.
            
            # Get actions for all agents
            # Note: compute_actions can take a dictionary of observations for multi-agent
            actions = algo.compute_actions(obs, explore=False) # Use deterministic mode for eval
            
            # Step env
            next_obs, rewards, terms, truncs, infos = env.step(actions)
            
            # Accumulate reward
            for id, r in rewards.items():
                total_reward[id] += r
                
            # Render
            # Need to get trajectories from physics engine if not in fast mode
            # But AllianceEnv hides physics engine inside server.
            # Access logic:
            trajectories = {}
            active_trajectories = {}
            
            # If fast mode, no traj. If slow mode, server.physics_engine active_trajectories
            if hasattr(env.server, 'physics_engine'):
                active_trajectories = env.server.physics_engine.active_trajectories
            
            visualizer.render(
                env.server.match.game_state,
                trajectories=trajectories,
                active_trajectories=active_trajectories
            )
            
            # Update obs
            obs = next_obs
            
            # Check termination
            if terms["__all__"] or truncs["__all__"]:
                print(f"Episode finished. Rewards: {total_reward}")
                # Reset
                obs, info = env.reset()
                total_reward = {id: 0.0 for id in env.possible_agents}
            
            # Cap FPS
            clock.tick(60) # 60 FPS
            
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        env.close()
        pygame.quit()
        ray.shutdown()

if __name__ == "__main__":
    main()
