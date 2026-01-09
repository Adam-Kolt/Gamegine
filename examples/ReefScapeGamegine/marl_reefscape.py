"""
Multi-Agent Reinforcement Learning Example for Reefscape.

This example demonstrates using the gamegine.rl module to train
multiple robots cooperatively on the Reefscape game.

Features:
- Proper field visualization with obstacles, safety corridors
- Smooth robot animation along trajectories
- Action pauses during scoring/interactions

Usage:
    python marl_reefscape.py
"""
import logging
import math
import time
from copy import deepcopy
from typing import Dict

import numpy as np

# Gamegine imports
from Reefscape import Names, Reefscape, StartingPositionsBlue, get_start_position
from gamepieces import Coral, Algae
from scoring import ReefState

from gamegine.first.alliance import Alliance
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.render import Renderer  # Import from package to auto-register handlers
from gamegine.representation.bounds import Rectangle
from gamegine.representation.interactable import RobotInteractionConfig
from gamegine.representation.robot import PhysicalParameters, SwerveRobot
from gamegine.simulation.GameServer import DiscreteGameServer
from gamegine.simulation.robot import RobotState
from gamegine.utils.NCIM.ncim import Inch, Pound, Ampere, Feet, Degree, Second, Radian
from gamegine.utils.NCIM.Dimensions.spatial import Meter
from gamegine.utils.logging import GetLogger, SetLoggingLevel

# RL imports
from gamegine.rl import make_env, EnvConfig, RobotConfig

# Suppress verbose logging
SetLoggingLevel(logging.WARNING)

# Initialize pygame early
import pygame
pygame.init()
print("Pygame initialized successfully")


# =============================================================================
# ROBOT DEFINITIONS
# =============================================================================

ROBOT_WIDTH = Inch(32)
ROBOT_LENGTH = Inch(32)
ROBOT_MASS = Pound(120)
ROBOT_MOI = Pound(120) * Inch(30) ** 2

ROBOT_SWERVE = SwerveConfig(
    SwerveModule(
        motors.MotorConfig(
            motors.KrakenX60,
            motors.PowerConfig(Ampere(60), Ampere(240), 1.0),
        ),
        gearing.MK4I.L3,
        motors.MotorConfig(
            motors.KrakenX60,
            motors.PowerConfig(Ampere(60), Ampere(240), 1.0),
        ),
        gearing.MK4I.L3,
    )
)

ROBOT_GEOMETRY = [
    Rectangle.from_center((Inch(0), Inch(0)), ROBOT_WIDTH, ROBOT_LENGTH).get_3d(
        Inch(0), Feet(3)
    )
]

# Interaction timing
CORAL_PICKUP_TIME = Second(0.5)
L1_SCORE_TIME = Second(0.5)
L2_SCORE_TIME = Second(0.5)
L3_SCORE_TIME = Second(0.5)
L4_SCORE_TIME = Second(0.5)
ALGAE_DISLODGE_TIME = Second(1)
PROCESSOR_SCORE_TIME = Second(0.5)


def create_robot(name: str, mass_multiplier: int) -> SwerveRobot:
    """Create a new SwerveRobot instance with the given name."""
    robot = SwerveRobot(
        name,
        ROBOT_SWERVE,
        ROBOT_GEOMETRY,
        PhysicalParameters(ROBOT_MASS * mass_multiplier, ROBOT_MOI),
    )
    robot.override_bounding_radius(Inch(16))
    return robot


def add_robot_interactions(robot: SwerveRobot):
    """Add interaction configurations to a robot."""
    # Coral pickup stations
    robot.add_interaction_config(
        RobotInteractionConfig(
            Names.TopCoralStation,
            "PickupCoral",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: CORAL_PICKUP_TIME,
        )
    )
    robot.add_interaction_config(
        RobotInteractionConfig(
            Names.BottomCoralStation,
            "PickupCoral",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: CORAL_PICKUP_TIME,
        )
    )

    # Reef scoring positions
    scoring_columns = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]
    Reef_Position = (Inch(176.746), Reefscape.half_field_y())
    away = Inch(32.5800055942) + ROBOT_LENGTH + Inch(3)
    angle_step = Degree(360) / 6
    curr_angle = Degree(180)
    
    for i, column in enumerate(scoring_columns):
        nav_angle = Degree(180) + curr_angle
        vector = np.array(
            [math.cos(nav_angle.to(Radian)), math.sin(nav_angle.to(Radian))]
        )
        compressed_angle = Radian(np.arctan2(vector[1], vector[0]))
        nav_point = (
            Reef_Position[0] + away * math.cos(curr_angle.to(Radian)),
            Reef_Position[1] + away * math.sin(curr_angle.to(Radian)),
            compressed_angle,
        )

        for level, score_time in [("l1", L1_SCORE_TIME), ("l2", L2_SCORE_TIME), 
                                   ("l3", L3_SCORE_TIME), ("l4", L4_SCORE_TIME)]:
            robot.add_interaction_config(
                RobotInteractionConfig(
                    Names.Reef,
                    f"{level}_{column}",
                    lambda statespace, robotstate, gamestate: True,
                    lambda statespace, robotstate, gamestate, t=score_time: t,
                    navigation_point=nav_point,
                )
            )
        
        if (i + 1) % 2 == 0:
            curr_angle += angle_step

    # Algae interactions
    algae_sides = [["A", "B"], ["C", "D"], ["E", "F"], ["G", "H"], ["I", "J"], ["K", "L"]]
    for side in algae_sides:
        robot.add_interaction_config(
            RobotInteractionConfig(
                Names.Reef,
                f"dislodge_{side[0]}{side[1]}",
                lambda statespace, robotstate, gamestate: True,
                lambda statespace, robotstate, gamestate: ALGAE_DISLODGE_TIME,
            )
        )
        robot.add_interaction_config(
            RobotInteractionConfig(
                Names.Reef,
                f"pickup_{side[0]}{side[1]}",
                lambda statespace, robotstate, gamestate: True,
                lambda statespace, robotstate, gamestate: ALGAE_DISLODGE_TIME,
            )
        )

    # Processor
    robot.add_interaction_config(
        RobotInteractionConfig(
            Names.Processor,
            "processor",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: PROCESSOR_SCORE_TIME,
        )
    )


# =============================================================================
# MULTI-AGENT ENVIRONMENT SETUP
# =============================================================================

def create_marl_env(num_agents: int = 3, render_mode: str = None):
    """Create a multi-agent discrete environment for Reefscape.
    
    :param num_agents: Number of robots on the blue alliance.
    :param render_mode: "human" for visualization, None for headless.
    :returns: MultiAgentDiscreteEnv instance.
    """
    # Create unique robots for each agent
    robots = []
    start_positions = [
        StartingPositionsBlue.A,
        StartingPositionsBlue.B,
        StartingPositionsBlue.C,
    ]
    
    for i in range(num_agents):
        robot = create_robot(f"Blue{i+1}", mass_multiplier=i+1)
        add_robot_interactions(robot)
        
        start_pos = start_positions[i % len(start_positions)]
        start_state = RobotState(
            x=start_pos[0],
            y=start_pos[1],
            heading=Degree(0),
            alliance=Alliance.BLUE,
            gamepieces={Coral: 1, Algae: 0},
        )
        
        robots.append(RobotConfig(
            robot=robot,
            start_state=start_state,
            name=f"Blue{i+1}",
            team="blue",
        ))
    
    # Create environment config
    config = EnvConfig(
        mode="discrete",
        multi_agent=True,
        robots=robots,
        include_opponent_obs=False,  # No opponents in this example
        include_velocity_obs=False,  # Discrete mode doesn't track velocity
        use_event_stepping=True,
        invalid_action_penalty=-1.0,
        max_episode_steps=50,
    )
    
    # Create environment
    env = make_env(Reefscape, DiscreteGameServer, config)
    
    return env


# =============================================================================
# VISUALIZATION (Matching ai_tuner.py style)
# =============================================================================

class MARLVisualizer:
    """Visualizer for multi-agent Reefscape environments.
    
    Matches the proven ai_tuner.py rendering pattern exactly.
    """
    
    def __init__(self, env, game):
        self.env = env
        self.game = game
        self.renderer = Renderer.create(game=game)
        
    def reset(self):
        """Reset for new episode (no state to clear with this simpler implementation)."""
        pass
        
    def animate_step(self, infos: Dict, speed_multiplier: float = 1.0):
        """Animate all agents through their trajectories, exactly like ai_tuner.py.
        
        :param infos: Info dict from env.step() containing trajectories and timing
        :param speed_multiplier: Animation speed (1.0 = real-time)
        :returns: True to continue, False if window closed
        """
        # Collect trajectory and timing info
        trajectories = {}
        traj_times = {}
        action_times = {}
        action_names = {}
        
        for agent in self.env.agents:
            info = infos.get(agent, {})
            trajectories[agent] = info.get("trajectory")
            traj_times[agent] = info.get("trajectory_time", 0.0)
            action_times[agent] = info.get("action_time", 0.1)
            action_names[agent] = info.get("action_name", "WAIT")
        
        # Calculate animation duration (max of all agents' action times)
        max_traj_time = max(traj_times.values()) if traj_times else 0.0
        max_action_time = max(action_times.values()) if action_times else 0.1
        
        # Action pause = time spent on interaction after driving
        action_pause = max(0, max_action_time - max_traj_time)
        total_anim_time = max_traj_time + action_pause
        
        if total_anim_time < 0.05:
            total_anim_time = 0.05  # Minimum animation time
        
        # Get traversal space for safety padding/mesh
        first_agent = self.env.agents[0]
        traversal_space = self.env.server.get_traversal_space(first_agent)
        
        # === SETUP RENDERER (exactly like ai_tuner.py) ===
        self.renderer.clear_objects()
        self.renderer.clear_dynamic()
        
        # Add obstacles to proper layers
        self.renderer.add_obstacles(self.game.get_obstacles())
        if traversal_space:
            self.renderer.add_safety_padding(traversal_space.obstacles)
            self.renderer.add(traversal_space.traversal_map)
        
        # Add trajectories
        for agent, traj in trajectories.items():
            if traj is not None:
                self.renderer.add(traj)
        
        # === ANIMATED RENDER LOOP (exactly like ai_tuner.py) ===
        frame_time = 1 / 60  # 60 FPS
        anim_time = 0.0
        
        while anim_time < total_anim_time:
            # Render frame
            self.renderer.begin_frame()
            
            # Draw static layers
            for layer_name in ["safety_padding", "obstacles", "mesh"]:
                if layer_name in self.renderer._layers:
                    for obj in self.renderer._layers[layer_name]:
                        if layer_name == "safety_padding":
                            self.renderer._draw_safety_padding_object(obj)
                        else:
                            self.renderer.draw_element(obj)
            
            # Draw trajectories
            if "trajectory" in self.renderer._layers:
                for traj in self.renderer._layers["trajectory"]:
                    self.renderer.draw_element(traj)
            
            # Draw each robot at their current animation position
            for agent in self.env.agents:
                traj = trajectories[agent]
                traj_duration = traj_times[agent]
                
                if traj is not None and traj_duration > 0:
                    if anim_time < traj_duration:
                        # Driving phase - interpolate along trajectory
                        robot_state = traj.get_at_time(Second(anim_time))
                    else:
                        # Action phase - robot at end of trajectory
                        robot_state = traj.get_at_time(traj.get_travel_time())
                    self.renderer.draw_element(robot_state)
                else:
                    # No trajectory - draw at server state
                    robot_state = self.env._get_robot_state(agent)
                    if robot_state:
                        self.renderer.draw_element(robot_state)
            
            # Draw HUD
            score = self.env.server.game_state.score.get()
            game_time = self.env.server.game_state.current_time.get()
            
            self.renderer.draw_rect(0, self.renderer.height - 95, 600, 95, (255, 255, 255))
            self.renderer.draw_text(
                f"Score: {score}  |  Game Time: {game_time:.1f}s",
                10, self.renderer.height - 25, (0, 0, 0), 18
            )
            self.renderer.draw_text(
                f"Anim: {anim_time:.1f}s / {total_anim_time:.1f}s (Drive: {max_traj_time:.1f}s + Action: {action_pause:.1f}s)",
                10, self.renderer.height - 45, (0, 0, 0), 14
            )
            
            # Show agent status
            y_offset = 65
            for agent in self.env.agents:
                action_name = action_names.get(agent, "?")
                traj_duration = traj_times[agent]
                
                if traj_duration > 0 and anim_time < traj_duration:
                    status = f"DRIVING: {action_name} ({anim_time:.1f}/{traj_duration:.1f}s)"
                    color = (0, 100, 200)  # Blue
                elif action_names[agent] != "WAIT":
                    status = f"ACTION: {action_name}"
                    color = (200, 150, 0)  # Orange
                else:
                    status = "READY"
                    color = (0, 128, 0)  # Green
                
                self.renderer.draw_text(
                    f"{agent}: {status}",
                    10, self.renderer.height - y_offset, 
                    color, 14
                )
                y_offset += 16
            
            self.renderer.end_frame()
            
            # Process events
            if not self.renderer.loop():
                return False
            
            time.sleep(frame_time / speed_multiplier)
            anim_time += frame_time
        
        return True


# =============================================================================
# MAIN: RANDOM POLICY DEMO
# =============================================================================

def run_random_demo():
    """Run a demo with random actions to test the SMDP environment."""
    print("=" * 60)
    print("Multi-Agent Reefscape Demo (SMDP Event-Stepping)")
    print("=" * 60)
    
    # Create environment with 2 agents
    env = create_marl_env(num_agents=3)
    viz = MARLVisualizer(env, Reefscape)
    
    print(f"Agents: {env.agents}")
    print(f"Action spaces: {[env.action_spaces[a].n for a in env.agents]}")
    
    # Reset
    observations, infos = env.reset()
    viz.reset()  # Reset visualizer trajectory tracking
    print(f"\nInitial observations received for {len(observations)} agents")
    
    total_rewards = {a: 0.0 for a in env.agents}
    done = False
    step = 0
    
    print("Starting SMDP event-stepping loop...")
    print("(Time advances to next event - agents finish at different times)")
    print()
    
    while not done:
        step += 1
        
        # Get current time and free agents
        current_time = env.server.game_state.current_time.get()
        free_agents = env.get_free_agents()
        
        # Select actions for all agents
        actions = {}
        for agent in env.agents:
            mask = env.action_mask(agent)
            valid_actions = np.where(mask == 1)[0]
            
            if len(valid_actions) > 0:
                actions[agent] = np.random.choice(valid_actions)
            else:
                actions[agent] = 0  # WAIT
        
        # Print which agents are acting
        if free_agents:
            print(f"Step {step} (t={current_time:.1f}s): Free agents: {free_agents}")
        
        # Step environment
        observations, rewards, terminations, truncations, infos = env.step(actions)
        
        # Get new time to show delta
        new_time = env.server.game_state.current_time.get()
        time_delta = new_time - current_time
        
        # Accumulate rewards
        for agent in env.agents:
            total_rewards[agent] += rewards[agent]
        
        # Check done
        done = any(terminations.values())
        
        # Animate
        if not viz.animate_step(infos, speed_multiplier=2.0):
            break
        
        # Print completed actions
        for agent in env.agents:
            info = infos.get(agent, {})
            if rewards[agent] != 0:
                print(f"  -> {agent} completed: {info.get('action_name', '?')} "
                      f"(reward: {rewards[agent]:+.1f})")
    
    print("\n" + "=" * 60)
    print(f"Episode complete after {step} steps")
    print(f"Final Score: {env.server.game_state.score.get()}")
    print(f"Final Time: {env.server.game_state.current_time.get():.1f}s")
    print(f"Total Rewards: {total_rewards}")
    print("=" * 60)
    
    env.close()


if __name__ == "__main__":
    run_random_demo()

