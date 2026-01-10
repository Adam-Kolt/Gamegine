"""
Dynamic Obstacle Avoidance Demo

This script demonstrates multi-robot trajectory generation with dynamic obstacle avoidance.
It repeatedly forces two robots to swap positions (crossing paths) to show how the
second robot generates a trajectory that avoids the first one.

Features:
- PhysicsEngine integration
- Time-aware collision checking
- MARLVisualizer for rendering
"""
import math
import random
import time
from typing import Dict, Tuple

import pygame
import numpy as np

# Gamegine imports
from examples.ReefScapeGamegine.marl_reefscape import create_marl_env, MARLVisualizer
from gamegine.simulation.robot import RobotState
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Inch
from gamegine.utils.NCIM.Dimensions.angular import Degree, Radian
from gamegine.utils.NCIM.Dimensions.temporal import Second


def generate_crossing_scenario(physics, robots, current_time=0.0):
    """Generate two trajectories that cross each other."""
    
    # Scenario: Diagonal swap in a 4m x 4m area
    # Robot 1: Bottom-Left -> Top-Right
    start1 = (Meter(1.0), Meter(1.0), Degree(0))
    end1 = (Meter(5.0), Meter(5.0), Degree(180))
    
    # Robot 2: Top-Left -> Bottom-Right
    start2 = (Meter(1.0), Meter(5.0), Degree(0))
    end2 = (Meter(5.0), Meter(1.0), Degree(180))
    
    # Add some randomness to make it interesting
    offset = random.uniform(-0.5, 0.5)
    start1 = (start1[0] + Meter(offset), start1[1], start1[2])
    end1 = (end1[0] - Meter(offset), end1[1], end1[2])
    
    trajectories = {}
    traj_times = {}
    
    # --- Robot 1: Priority Robot ---
    # Generates optimal path first
    r1_name = "Blue1"
    r1_robot = robots[0]
    
    # 1. Pathfind
    traversal1 = physics.prepare_traversal_space(r1_name, r1_robot, [], (Meter(10), Meter(10)))
    path1 = physics.pathfind(r1_name, start1[0], start1[1], end1[0], end1[1], traversal1)
    
    # 2. Generate Trajectory
    # Note: avoid_other_robots=True even for first robot (in case there are existing ones)
    traj1 = physics.generate_trajectory(
        r1_name, r1_robot, start1, end1, path1, traversal1,
        start_time=current_time,
        avoid_other_robots=True 
    )
    
    # 3. Register as active obstacle
    r1_radius = r1_robot.get_bounding_radius().to(Meter)
    physics.register_active_trajectory(r1_name, traj1, current_time, r1_radius)
    
    trajectories[r1_name] = traj1
    traj_times[r1_name] = traj1.get_travel_time().to(Second)

    # --- Robot 2: Avoidance Robot ---
    # Generates path that must avoid Robot 1
    r2_name = "Blue2"
    r2_robot = robots[1]
    
    # 1. Pathfind
    traversal2 = physics.prepare_traversal_space(r2_name, r2_robot, [], (Meter(10), Meter(10)))
    path2 = physics.pathfind(r2_name, start2[0], start2[1], end2[0], end2[1], traversal2)
    
    # 2. Generate Trajectory (will see Robot 1 as dynamic obstacle)
    print(f"Generating {r2_name} trajectory with active obstacles: {len(physics.active_trajectories)}")
    start_gen = time.time()
    traj2 = physics.generate_trajectory(
        r2_name, r2_robot, start2, end2, path2, traversal2,
        start_time=current_time,
        avoid_other_robots=True
    )
    slowdown = getattr(traj2, 'slowdown_segments', [])
    print(f"Generation took {time.time() - start_gen:.4f}s, slowdown_segments={slowdown}")
    
    # 3. Register (for completeness, though no more robots here)
    r2_radius = r2_robot.get_bounding_radius().to(Meter)
    physics.register_active_trajectory(r2_name, traj2, current_time, r2_radius)
    
    trajectories[r2_name] = traj2
    traj_times[r2_name] = traj2.get_travel_time().to(Second)
    
    return trajectories, traj_times


def run_demo():
    print("="*60)
    print("Dynamic Obstacle Avoidance Demo")
    print("Forcing 2 robots to cross paths repeatedly.")
    print("Robot 2 (Blue2) should deviate to avoid Robot 1 (Blue1).")
    print("="*60)
    
    # 1. Setup Env (headless) & Viz
    env = create_marl_env(num_agents=2)
    viz = MARLVisualizer(env, env.game)
    physics = env.server.physics_engine
    
    # Get actual robot objects
    robots = [env.server.robots["Blue1"], env.server.robots["Blue2"]]
    
    episode = 0
    while True:
        episode += 1
        print(f"\n--- Scenario {episode}: Crossing Paths ---")
        
        # Clear previous state
        physics.clear_all_active_trajectories()
        current_time = 0.0 # Reset time for simplicity of demo
        
        try:
            # Generate collision scenario
            trajectories, traj_times = generate_crossing_scenario(physics, robots, current_time)
            
            # Construct infos for visualizer
            infos = {}
            for i, name in enumerate(["Blue1", "Blue2"]):
                infos[name] = {
                    "trajectory": trajectories[name],
                    "trajectory_time": traj_times[name],
                    "action_time": traj_times[name] + 0.5,  # Small pause at end
                    "action_name": "CROSS_FIELD",
                }
            
            # Print timing info - Blue2 should take longer due to slowdown
            for name in ["Blue1", "Blue2"]:
                t = traj_times[name]
                slowdown = getattr(trajectories[name], 'slowdown_segments', [])
                print(f"  {name}: travel={t:.2f}s, slowdown_segments={slowdown}")
            
            # Animate
            print("Animating...")
            if not viz.animate_step(infos, speed_multiplier=1.5):
                print("Window closed.")
                return

        except Exception as e:
            print(f"Error generating scenario: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(1)

if __name__ == "__main__":
    run_demo()
