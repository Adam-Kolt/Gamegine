"""
Policy Visualizer for Rebuilt Game.

Loads a trained RLlib checkpoint and visualizes the policy controlling agents
with proper trajectory animation and per-robot action timing.
"""

import argparse
import logging
import os
import sys
import warnings
import time
import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Tuple

# Suppress logging noise
os.environ["RAY_DEDUP_LOGS"] = "1"
os.environ["RAY_SCHEDULER_EVENTS"] = "0"
os.environ["PYTHONWARNINGS"] = "ignore::DeprecationWarning"
warnings.filterwarnings("ignore", category=DeprecationWarning)
logging.getLogger("ray").setLevel(logging.ERROR)

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import arcade
import numpy as np
import ray
from ray.rllib.algorithms.algorithm import Algorithm
from ray.tune.registry import register_env

from gamegine.first.alliance import Alliance
from gamegine.render.renderer import Renderer, DisplayLevel, ObjectRendererRegistry
from gamegine.simulation.GameServer import DiscreteGameServer, ServerConfig
from gamegine.simulation.robot import RobotState
from gamegine.simulation.game import GameState
from gamegine.utils.NCIM.Dimensions.spatial import Meter, Feet, Inch
from gamegine.utils.NCIM.Dimensions.angular import Degree
from gamegine.utils.NCIM.Dimensions.temporal import Second
from gamegine.utils.NCIM.Dimensions.mass import Pound

from examples.Rebuilt.Rebuilt import create_rebuilt_game, FIELD_LENGTH, FIELD_WIDTH
from examples.Rebuilt.robot import create_robot, setup_robot_interactions, ROBOT_WIDTH
from examples.Rebuilt.scoring import Fuel, Hub, Tower, Depot, AllianceZone, NeutralZone
from examples.Rebuilt.shooting_locations import ShootingLocation
from examples.Rebuilt.discrete_action_demo import (
    draw_shooting_location, draw_depot, draw_hub, draw_tower, 
    draw_alliance_zone, draw_neutral_zone,
    AnimationLayer, ProjectileAnimation, draw_animation_layer,
    RobotAnimState, select_next_action, GameStateHUD,
)
from gamegine.utils.logging import SetLoggingLevel

SetLoggingLevel(logging.FATAL)


# =============================================================================
# ROBOT CONFIGURATION
# =============================================================================

@dataclass
class PolicyRobotConfig:
    """Configuration for a robot controlled by RL policy."""
    name: str
    start_x: Any  # SpatialMeasurement
    start_y: Any
    alliance: Alliance
    color: Tuple[int, int, int]


HALF_WIDTH = FIELD_WIDTH / 2

# 3 Blue + 3 Red robots
BLUE_CONFIGS = [
    PolicyRobotConfig("Blue1", Feet(6), HALF_WIDTH, Alliance.BLUE, (52, 152, 219)),
    PolicyRobotConfig("Blue2", Feet(6), HALF_WIDTH + Feet(6), Alliance.BLUE, (41, 128, 185)),
    PolicyRobotConfig("Blue3", Feet(6), HALF_WIDTH - Feet(6), Alliance.BLUE, (26, 82, 118)),
]

RED_CONFIGS = [
    PolicyRobotConfig("Red1", FIELD_LENGTH - Feet(6), HALF_WIDTH, Alliance.RED, (231, 76, 60)),
    PolicyRobotConfig("Red2", FIELD_LENGTH - Feet(6), HALF_WIDTH + Feet(6), Alliance.RED, (192, 57, 43)),
    PolicyRobotConfig("Red3", FIELD_LENGTH - Feet(6), HALF_WIDTH - Feet(6), Alliance.RED, (169, 50, 38)),
]

ALL_CONFIGS = BLUE_CONFIGS + RED_CONFIGS


# =============================================================================
# POLICY VISUALIZER DEMO
# =============================================================================

class PolicyVisualizerDemo:
    """Visualizes RL policy with proper trajectory animation.
    
    Uses per-robot animation states with:
    - Trajectory interpolation for driving
    - Wait times for action execution
    - Ball projectile animations
    """
    
    def __init__(self, checkpoint_path: Optional[str] = None, num_robots: int = 3):
        # Create game
        self.game = create_rebuilt_game()
        
        # Game server
        self.server = DiscreteGameServer(ServerConfig())
        self.server.load_from_game(self.game)
        
        # Create and add all robots
        self.robots: Dict[str, Any] = {}  # name -> SwerveRobot
        self.robot_states: Dict[str, RobotAnimState] = {}  # name -> RobotAnimState
        self.robot_configs: Dict[str, PolicyRobotConfig] = {}  # name -> config
        
        configs = BLUE_CONFIGS[:num_robots] + RED_CONFIGS[:num_robots]
        
        for i, config in enumerate(configs):
            robot = create_robot(config.name, config.alliance, Pound(120))
            self.robots[config.name] = robot
            self.server.add_robot(robot)
            
            # Initialize robot state in server
            initial_state = RobotState(
                config.start_x, config.start_y, 
                Degree(0) if config.alliance == Alliance.BLUE else Degree(180),
                alliance=config.alliance,
                gamepieces={Fuel: 8}  # Start with fuel
            )
            initial_state.setValue("name", config.name)
            self.server.init_robot(config.name, initial_state)
            
            # Configure interactions for this robot
            setup_robot_interactions(robot, self.game)
            
            # Create animation state (using DemoAction's RobotAnimState for compatibility)
            # We need to create a compatible config object
            class CompatConfig:
                def __init__(self, cfg):
                    self.name = cfg.name
                    self.start_x = cfg.start_x
                    self.start_y = cfg.start_y
                    self.policy = "cycle"  # Default policy
                    self.color = cfg.color
                    self.alliance = cfg.alliance
                    
            self.robot_states[config.name] = RobotAnimState(
                robot_name=config.name,
                config=CompatConfig(config),
                current_state=RobotState(config.start_x, config.start_y, 
                                         Degree(0) if config.alliance == Alliance.BLUE else Degree(180)),
            )
            self.robot_configs[config.name] = config
        
        # Demo state
        self.match_time = 0.0
        self.match_duration = 160.0
        
        # Animation layer for projectiles
        self.animation_layer = AnimationLayer()
        
        # RL Policy (optional - if None, use scripted AI)
        self.algo = None
        self.use_policy = False
        
        if checkpoint_path and checkpoint_path.lower() != "none":
            self._load_checkpoint(checkpoint_path)
        
        # Setup renderer
        self._setup_renderer()
        
    def _load_checkpoint(self, checkpoint_path: str):
        """Load RLlib checkpoint for policy inference."""
        try:
            if not ray.is_initialized():
                ray.init(ignore_reinit_error=True)
            
            self.algo = Algorithm.from_checkpoint(checkpoint_path)
            self.use_policy = True
            print(f"Loaded policy from {checkpoint_path}")
        except Exception as e:
            print(f"Failed to load checkpoint: {e}")
            print("Using scripted AI instead")
            self.use_policy = False
    
    def _setup_renderer(self):
        """Setup arcade renderer with all game elements."""
        # Register handlers
        ObjectRendererRegistry.register_handler(ShootingLocation, draw_shooting_location)
        ObjectRendererRegistry.register_handler(Depot, draw_depot)
        ObjectRendererRegistry.register_handler(Hub, draw_hub)
        ObjectRendererRegistry.register_handler(Tower, draw_tower)
        ObjectRendererRegistry.register_handler(AllianceZone, draw_alliance_zone)
        ObjectRendererRegistry.register_handler(NeutralZone, draw_neutral_zone)
        ObjectRendererRegistry.register_handler(AnimationLayer, draw_animation_layer)
        
        # Create renderer
        self.renderer = Renderer.create(game=self.game, width=1200, height=600)
        self.renderer.display_level = DisplayLevel.SHOWCASE
        
        # Add obstacles
        for obs in self.game.get_obstacles():
            self.renderer.add(obs)
        
        # Add zones
        for zone in self.game.get_zones():
            self.renderer.add(zone)
        
        # Add interactables
        for interactable in self.game.get_interactables():
            self.renderer.add(interactable)
        
        # Add animation layer
        self.renderer.add(self.animation_layer)
        
        # Register robot drawing
        class RobotDisplayRef:
            def __init__(self, demo):
                self._demo = demo
        
        def draw_robots(display_ref, canvas, theme, display_level, renderer=None):
            demo = display_ref._demo
            if demo is None:
                return
            
            for name, anim_state in demo.robot_states.items():
                config = demo.robot_configs[name]
                state = anim_state.current_state
                
                x = canvas.to_pixels(state.x.get() if hasattr(state.x, 'get') else state.x)
                y = canvas.to_pixels(state.y.get() if hasattr(state.y, 'get') else state.y)
                
                # Draw robot as colored square
                robot_size = canvas.to_pixels(Inch(28))
                half = robot_size / 2
                
                heading = float((state.heading.get() if hasattr(state.heading, 'get') else state.heading).to(Degree))
                rad = math.radians(heading)
                cos_h, sin_h = math.cos(rad), math.sin(rad)
                
                corners = []
                for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
                    rx = dx * half
                    ry = dy * half
                    corners.append((
                        x + rx * cos_h - ry * sin_h,
                        y + rx * sin_h + ry * cos_h
                    ))
                
                color = config.color
                arcade.draw_polygon_filled(corners, (*color, 200))
                arcade.draw_polygon_outline(corners, (*color, 255), 2)
                
                # Draw robot name
                arcade.draw_text(
                    name, x, y + robot_size * 0.7,
                    (255, 255, 255), 10, anchor_x="center"
                )
                
                # Draw status
                if anim_state.has_climbed:
                    status = "✓ CLIMBED"
                    status_color = (0, 255, 0)
                elif anim_state.is_animating:
                    status = "▶ DRIVING"
                    status_color = (255, 255, 0)
                elif anim_state.is_waiting:
                    status = "⏳ ACTION"
                    status_color = (255, 165, 0)
                else:
                    status = "● IDLE"
                    status_color = (128, 128, 128)
                
                arcade.draw_text(
                    status, x, y - robot_size * 0.7,
                    status_color, 9, anchor_x="center"
                )
                
                # Draw fuel count
                robot_state = demo.server.match.game_state.get_robot(name)
                if robot_state:
                    fuel_dict = robot_state.gamepieces.get()
                    fuel_count = fuel_dict.get(Fuel, 0)
                    arcade.draw_text(
                        f"Fuel: {fuel_count}", x, y - robot_size * 1.2,
                        (255, 200, 0), 8, anchor_x="center"
                    )
        
        ObjectRendererRegistry.register_handler(RobotDisplayRef, draw_robots)
        self.robot_display = RobotDisplayRef(self)
        self.renderer.add(self.robot_display)
        
        # Add HUD
        self.hud = GameStateHUD(self.server.match.game_state, self)
        self.renderer.add(self.hud)
        
        # Register update callback
        self.renderer.on_update_callback(self._on_update)
    
    def _on_update(self, delta_time: float):
        """Main update loop - called every frame."""
        self.match_time += delta_time
        
        # Update all robots
        for name, anim_state in self.robot_states.items():
            self._update_robot(name, anim_state, delta_time)
        
        # Update animations
        self.animation_layer.update(delta_time)
    
    def _update_robot(self, name: str, anim_state: RobotAnimState, dt: float):
        """Update a single robot's state machine."""
        if anim_state.has_climbed:
            return
        
        if anim_state.is_animating:
            # Update trajectory animation
            anim_state.anim_time += dt
            traj = anim_state.current_trajectory
            
            if traj:
                travel_time = float(traj.get_travel_time().to(Second))
                
                if anim_state.anim_time >= travel_time:
                    # Trajectory complete
                    state = traj.get_at_time(traj.get_travel_time())
                    anim_state.current_state = RobotState(state.x, state.y, state.theta)
                    anim_state.is_animating = False
                    anim_state.is_waiting = True
                    anim_state.wait_time = 1.0  # Action time
                    
                    # Update server state
                    self.server.match.game_state.get_robot(name).x.set(state.x)
                    self.server.match.game_state.get_robot(name).y.set(state.y)
                    self.server.match.game_state.get_robot(name).heading.set(state.theta)
                else:
                    # Update position along trajectory
                    state = traj.get_at_time(Second(anim_state.anim_time))
                    anim_state.current_state = RobotState(state.x, state.y, state.theta)
        
        elif anim_state.is_waiting:
            # Process waiting/action phase
            anim_state.wait_time -= dt
            anim_state.spawn_timer += dt
            
            # Process animation queue
            remaining = []
            for item in anim_state.action_queue:
                if anim_state.spawn_timer >= item["delay"]:
                    self.animation_layer.add(item["anim"])
                else:
                    remaining.append(item)
            anim_state.action_queue = remaining
            
            if anim_state.wait_time <= 0:
                anim_state.is_waiting = False
                anim_state.actions_performed += 1
                anim_state.action_queue = []
        
        else:
            # Idle - select and execute next action
            self._execute_robot_action(name, anim_state)
    
    def _execute_robot_action(self, name: str, anim_state: RobotAnimState):
        """Select and execute next action for a robot."""
        config = self.robot_configs[name]
        game_state = self.server.match.game_state
        robot_state = game_state.get_robot(name)
        
        if robot_state is None:
            return
        
        # Select action (use scripted AI for now - policy integration TODO)
        action = select_next_action(
            name, robot_state, game_state,
            policy="cycle",  # Could be "zone" or "defender"
            actions_performed=anim_state.actions_performed,
            has_climbed=anim_state.has_climbed,
            alliance=config.alliance,
            hub_active=True  # TODO: Check actual hub state
        )
        
        if action is None:
            return
        
        interactable_name = action.interactable
        interaction_name = action.interaction
        
        # Get navigation point
        nav_point = self.server.match.get_navigation_point(
            interactable_name, interaction_name, name
        )
        
        if nav_point is None:
            # No navigation needed (e.g., WAIT)
            anim_state.is_waiting = True
            anim_state.wait_time = 0.5
            return
        
        # Use drive_and_process_action for trajectory generation
        try:
            result = self.server.drive_and_process_action(
                interactable_name, interaction_name, name, None
            )
            
            # Handle return value
            if isinstance(result, tuple):
                success, trajectory = result
            else:
                success = bool(result)
                trajectory = None
            
            if not success or trajectory is None:
                anim_state.is_waiting = True
                anim_state.wait_time = 0.5
                return
            
            # Start driving animation
            anim_state.current_trajectory = trajectory
            anim_state.anim_time = 0.0
            anim_state.is_animating = True
            anim_state.spawn_timer = 0.0
            anim_state._last_action = f"{interactable_name}:{interaction_name}"
            
            # Get trajectory end position for animations
            end_state = trajectory.get_at_time(trajectory.get_travel_time())
            end_x = end_state.x
            end_y = end_state.y
            
            # Queue ball animations for certain actions
            if "pickup" in interaction_name:
                # Parse quantity from interaction name (e.g., "pickup_5" -> 5)
                quantity = 1
                try:
                    quantity = int(interaction_name.split("_")[-1])
                except:
                    pass
                self._queue_pickup_animation(name, anim_state, end_x, end_y, quantity)
            elif "score" in interaction_name or "shoot" in interaction_name:
                # Hub "score_fuel" and ShootingLocation "shoot" both score exactly 1 FUEL per action
                self._queue_shoot_animation(name, anim_state, config.alliance, end_x, end_y, quantity=1)
            
        except Exception as e:
            print(f"Action failed for {name}: {e}")
            anim_state.is_waiting = True
            anim_state.wait_time = 1.0
    
    def _queue_pickup_animation(self, name: str, anim_state: RobotAnimState, 
                                 dest_x, dest_y, quantity: int):
        """Queue ball pickup animations."""
        # Queue balls coming TO robot destination
        for i in range(quantity):
            start_x = dest_x + Feet(random.uniform(-2, 2))
            start_y = dest_y + Feet(random.uniform(-2, 2))
            arc_offset = random.uniform(-20, 20)
            
            anim = ProjectileAnimation(
                (start_x, start_y), (dest_x, dest_y),
                duration=0.3, arc_offset=arc_offset
            )
            anim_state.action_queue.append({"delay": i * 0.1, "anim": anim})
    
    def _queue_shoot_animation(self, name: str, anim_state: RobotAnimState, 
                                alliance: Alliance, robot_x, robot_y, quantity: int):
        """Queue ball shooting animations."""
        # Hub positions from Rebuilt.py
        blue_hub_x = Meter(4.629)
        red_hub_x = FIELD_LENGTH - Meter(4.629)
        hub_y = HALF_WIDTH
        
        # Blue robots shoot at BLUE hub, Red robots shoot at RED hub
        if alliance == Alliance.BLUE:
            hub_x = blue_hub_x
        else:
            hub_x = red_hub_x
        
        # Queue balls FROM robot TO hub
        for i in range(quantity):
            arc_offset = random.uniform(-40, 40)
            anim = ProjectileAnimation(
                (robot_x, robot_y), (hub_x, hub_y),
                duration=0.5, arc_offset=arc_offset
            )
            anim_state.action_queue.append({"delay": i * 0.08, "anim": anim})
    
    def run(self):
        """Start the visualization."""
        arcade.run()


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="Visualize trained policy")
    parser.add_argument("--checkpoint", type=str, default=None,
                        help="Path to RLlib checkpoint (optional)")
    parser.add_argument("--robots", type=int, default=3,
                        help="Number of robots per alliance (1-3)")
    args = parser.parse_args()
    
    demo = PolicyVisualizerDemo(
        checkpoint_path=args.checkpoint,
        num_robots=min(3, max(1, args.robots))
    )
    demo.run()


if __name__ == "__main__":
    main()
