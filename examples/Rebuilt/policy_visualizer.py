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
    RobotAnimState, select_next_action,
)
from gamegine.utils.logging import SetLoggingLevel

SetLoggingLevel(logging.FATAL)


# =============================================================================
# ENHANCED HUD FOR POLICY VISUALIZER
# =============================================================================

class PolicyMatchHUD:
    """Enhanced HUD showing both alliance scores, match time, and mode indicator."""
    
    def __init__(self, game_state, demo: "PolicyVisualizerDemo"):
        self.game_state = game_state
        self.demo = demo
    
    def draw(self, canvas, theme, display_level, renderer=None):
        """Draw the enhanced match state HUD."""
        if renderer:
            width = renderer.width
            height = renderer.height
        else:
            width, height = 1200, 600
        
        # === TOP BAR: Scores and Time ===
        bar_height = 70
        arcade.draw_lbwh_rectangle_filled(0, height - bar_height, width, bar_height, (20, 20, 30, 230))
        
        # Match time in center
        match_time = self.demo.match_time
        remaining = max(0, self.demo.match_duration - match_time)
        minutes = int(remaining) // 60
        seconds = int(remaining) % 60
        
        time_str = f"{minutes}:{seconds:02d}"
        arcade.draw_text(
            time_str, width // 2, height - 45,
            (255, 255, 255), 32, bold=True, anchor_x="center", anchor_y="center"
        )
        
        # Phase indicator
        if match_time < 15:
            phase = "AUTO"
            phase_color = (255, 200, 50)
        elif remaining < 30:
            phase = "ENDGAME"
            phase_color = (255, 100, 100)
        else:
            phase = "TELEOP"
            phase_color = (100, 255, 100)
        
        arcade.draw_text(
            phase, width // 2, height - 15,
            phase_color, 14, bold=True, anchor_x="center", anchor_y="center"
        )
        
        # Blue score (left side)
        blue_score = self.game_state.blue_score.get() if hasattr(self.game_state, 'blue_score') else 0
        arcade.draw_lbwh_rectangle_filled(20, height - bar_height + 10, 180, 50, (30, 100, 200, 200))
        arcade.draw_text(
            "BLUE", 110, height - 25,
            (150, 200, 255), 12, bold=True, anchor_x="center"
        )
        arcade.draw_text(
            str(blue_score), 110, height - 48,
            (255, 255, 255), 28, bold=True, anchor_x="center"
        )
        
        # Red score (right side)
        red_score = self.game_state.red_score.get() if hasattr(self.game_state, 'red_score') else 0
        arcade.draw_lbwh_rectangle_filled(width - 200, height - bar_height + 10, 180, 50, (200, 50, 50, 200))
        arcade.draw_text(
            "RED", width - 110, height - 25,
            (255, 150, 150), 12, bold=True, anchor_x="center"
        )
        arcade.draw_text(
            str(red_score), width - 110, height - 48,
            (255, 255, 255), 28, bold=True, anchor_x="center"
        )
        
        # === MODE INDICATOR (bottom right) ===
        mode_box_width = 150
        mode_box_height = 30
        mode_x = width - mode_box_width - 20
        mode_y = 20
        
        if self.demo.use_policy:
            mode_text = "🤖 RL POLICY"
            mode_color = (50, 200, 50)
            bg_color = (20, 80, 20, 200)
        else:
            mode_text = "📜 SCRIPTED"
            mode_color = (200, 200, 50)
            bg_color = (80, 80, 20, 200)
        
        arcade.draw_lbwh_rectangle_filled(mode_x, mode_y, mode_box_width, mode_box_height, bg_color)
        arcade.draw_lbwh_rectangle_outline(mode_x, mode_y, mode_box_width, mode_box_height, mode_color, 2)
        arcade.draw_text(
            mode_text, mode_x + mode_box_width // 2, mode_y + mode_box_height // 2,
            mode_color, 14, bold=True, anchor_x="center", anchor_y="center"
        )
        
        # === STATS (bottom left) ===
        stats_x = 20
        stats_y = 20
        line_height = 18
        
        # Count robots per alliance
        blue_robots = sum(1 for n in self.demo.robot_states if n.startswith("Blue"))
        red_robots = sum(1 for n in self.demo.robot_states if n.startswith("Red"))
        
        arcade.draw_text(
            f"Robots: {blue_robots}v{red_robots}", stats_x, stats_y + line_height,
            (150, 150, 150), 12
        )
        arcade.draw_text(
            f"Elapsed: {match_time:.1f}s", stats_x, stats_y,
            (150, 150, 150), 12
        )


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
        
        # Build action maps for each robot (same as AllianceEnv)
        self._action_maps: Dict[str, List[Tuple[str, str]]] = {}
        self._build_action_maps()
        
        # Demo state
        self.match_time = 0.0
        self.match_duration = 160.0
        self.num_robots = num_robots
        
        # Animation layer for projectiles
        self.animation_layer = AnimationLayer()
        
        # RL Policy (optional - if None, use scripted AI)
        self.algo = None
        self.rl_module = None  # RLModule for inference
        self.use_policy = False
        
        if checkpoint_path and checkpoint_path.lower() != "none":
            self._load_checkpoint(checkpoint_path)
        
        # Setup renderer
        self._setup_renderer()
        
    def _load_checkpoint(self, checkpoint_path: str):
        """Load RLlib checkpoint for policy inference."""
        import torch
        from pathlib import Path
        
        try:
            if not ray.is_initialized():
                ray.init(ignore_reinit_error=True, logging_level=logging.ERROR)
            
            # Register the environment BEFORE loading checkpoint
            self._register_training_env()
            
            # Try to load the RLModule directly from checkpoint
            # The new RLlib API uses RLModule.forward_inference() for action computation
            rl_module_path = Path(checkpoint_path) / "learner_group" / "learner" / "rl_module"
            
            if rl_module_path.exists():
                from ray.rllib.core.rl_module.rl_module import RLModule
                
                # Load Multi-Agent RLModule from checkpoint
                self.rl_module = RLModule.from_checkpoint(str(rl_module_path))
                self.use_policy = True
                print(f"Loaded RLModule from {rl_module_path}")
                print(f"Available policies: {list(self.rl_module.keys()) if hasattr(self.rl_module, 'keys') else 'single'}")
            else:
                # Fallback: try loading algorithm (may not work with new API)
                print(f"RLModule path not found at {rl_module_path}, trying Algorithm.from_checkpoint...")
                self.algo = Algorithm.from_checkpoint(checkpoint_path)
                self.use_policy = True
                print(f"Loaded algorithm from {checkpoint_path}")
                
        except Exception as e:
            print(f"Failed to load checkpoint: {e}")
            import traceback
            traceback.print_exc()
            print("Using scripted AI instead")
            self.use_policy = False
    
    def _register_training_env(self):
        """Register the same environment used during training."""
        from gamegine.rl import make_alliance_env, RobotConfig
        from gamegine.simulation.robot import RobotState
        from gamegine.utils.NCIM.ncim import Inch
        from examples.Rebuilt.reward_function import AdvancedRebuiltRewardFunction
        
        def env_creator(cfg):
            # Create game instance
            game = create_rebuilt_game()
            
            # Create robot configs (same as training)
            def create_robot_configs(alliance, game, num_robots=3):
                from gamegine.utils.NCIM.Dimensions.mass import Pound
                configs = []
                team_str = "red" if alliance == Alliance.RED else "blue"
                
                for i in range(num_robots):
                    name = f"{team_str.capitalize()}{i+1}"
                    robot = create_robot(name, alliance)
                    setup_robot_interactions(robot, game)
                    
                    if alliance == Alliance.BLUE:
                        x = Inch(40 + i * 48)
                        heading = Degree(0)
                    else:
                        x = FIELD_LENGTH - Inch(40 + i * 48)
                        heading = Degree(180)
                    
                    y = FIELD_WIDTH / 2 + Inch(i * 20)
                    
                    start_state = RobotState(
                        x=x, y=y, heading=heading,
                        alliance=alliance,
                        gamepieces={Fuel: 8},
                    )
                    
                    configs.append(RobotConfig(
                        robot=robot,
                        start_state=start_state,
                        name=name,
                        team=team_str,
                    ))
                return configs
            
            red_robots = create_robot_configs(Alliance.RED, game, 3)
            blue_robots = create_robot_configs(Alliance.BLUE, game, 3)
            
            env = make_alliance_env(
                game=game,
                red_robots=red_robots,
                blue_robots=blue_robots,
                mode="self_play",
                fast_mode=True,
                use_server_pool=True,
                max_episode_steps=200,
            )
            
            # Same config as training
            env.config.reward_fn = AdvancedRebuiltRewardFunction()
            env.training_config.use_capability_context = True
            env.training_config.randomize_capabilities = True
            env.training_config.observe_opponent_states = True
            env.training_config.observe_opponent_capabilities = True
            
            # Rebuild observation spaces (same as training)
            env._observation_spaces = {}
            for agent_id in env._agent_ids:
                robot_name = env._agent_to_robot[agent_id]
                env._observation_spaces[agent_id] = env._build_observation_space(robot_name)
            
            from gymnasium import spaces
            env.observation_space = spaces.Dict(env._observation_spaces)
            
            return env
        
        register_env("reefscape-versatile-v0", env_creator)
        print("Registered environment: reefscape-versatile-v0")
    
    def _build_action_maps(self):
        """Build action maps for each robot (matching training format)."""
        for name in self.robots.keys():
            actions = self.server.get_actions_set(name)
            # Add WAIT/NO_OP as action 0 (same as AllianceEnv)
            self._action_maps[name] = [("WAIT", "NO_OP")] + list(actions)
    
    def _get_agent_id(self, robot_name: str) -> str:
        """Convert robot name to agent ID format used in training."""
        # Extract alliance and index from name like "Blue1" or "Red2"
        if robot_name.startswith("Blue"):
            idx = int(robot_name[4:]) - 1  # Blue1 -> blue_0
            return f"blue_{idx}"
        else:
            idx = int(robot_name[3:]) - 1  # Red1 -> red_0
            return f"red_{idx}"
    
    def _build_observation(self, robot_name: str) -> np.ndarray:
        """Build observation for a robot matching the training format."""
        game_state = self.server.match.game_state
        robot_state = game_state.get_robot(robot_name)
        
        # Build observation vector (matching AllianceEnv._get_observation)
        obs = np.zeros(10, dtype=np.float32)
        
        if robot_state:
            obs[0] = float(robot_state.x.get())
            obs[1] = float(robot_state.y.get())
            obs[2] = float(robot_state.heading.get())
            obs[3] = 0.0  # vx
            obs[4] = 0.0  # vy
        
        obs[5] = float(game_state.current_time.get())
        obs[6] = float(game_state.red_score.get())
        obs[7] = float(game_state.blue_score.get())
        
        # Semi-markovian state: is_busy and time_remaining
        anim_state = self.robot_states.get(robot_name)
        is_busy = 1.0 if (anim_state and (anim_state.is_animating or anim_state.is_waiting)) else 0.0
        time_remaining = anim_state.wait_time if (anim_state and anim_state.is_waiting) else 0.0
        obs = np.concatenate([obs, np.array([is_busy, time_remaining], dtype=np.float32)])
        
        # Capability context (matching training config)
        from gamegine.representation.capabilities import RobotCapabilities
        robot = self.robots.get(robot_name)
        if robot and hasattr(robot, "capabilities") and robot.capabilities:
            cap_vec = robot.capabilities.to_vector()
            obs = np.concatenate([obs, np.array(cap_vec, dtype=np.float32)])
        else:
            obs = np.concatenate([obs, np.zeros(RobotCapabilities.vector_size(), dtype=np.float32)])
        
        # Game-specific observations
        if self.game:
            game_obs = self.game.get_observation(game_state)
            if game_obs:
                obs = np.concatenate([obs, np.array(game_obs, dtype=np.float32)])
        
        # Opponent observations (3 opponents * 5 state vars + 3 * capability size)
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        config = self.robot_configs.get(robot_name)
        if config:
            if config.alliance == Alliance.BLUE:
                opponent_names = [n for n in self.robots.keys() if n.startswith("Red")]
            else:
                opponent_names = [n for n in self.robots.keys() if n.startswith("Blue")]
            
            opponent_names.sort()
            max_opponents = 3
            
            for i in range(max_opponents):
                if i < len(opponent_names):
                    opp_name = opponent_names[i]
                    opp_state = game_state.get_robot(opp_name)
                    if opp_state:
                        field_size = self.game.get_field_size()
                        fx = float(field_size[0].to(Meter))
                        fy = float(field_size[1].to(Meter))
                        opp_obs = np.array([
                            float(opp_state.x.get().to(Meter)) / fx,
                            float(opp_state.y.get().to(Meter)) / fy,
                            float(opp_state.heading.get().to(Degree)) / 360.0,
                            0.0, 0.0  # vx, vy
                        ], dtype=np.float32)
                    else:
                        opp_obs = np.zeros(5, dtype=np.float32)
                else:
                    opp_obs = np.zeros(5, dtype=np.float32)
                obs = np.concatenate([obs, opp_obs])
                
                # Opponent capabilities
                if i < len(opponent_names):
                    opp_robot = self.robots.get(opponent_names[i])
                    if opp_robot and hasattr(opp_robot, "capabilities") and opp_robot.capabilities:
                        opp_caps = opp_robot.capabilities.to_vector()
                    else:
                        opp_caps = np.zeros(RobotCapabilities.vector_size(), dtype=np.float32)
                else:
                    opp_caps = np.zeros(RobotCapabilities.vector_size(), dtype=np.float32)
                obs = np.concatenate([obs, np.array(opp_caps, dtype=np.float32)])
        
        return obs
    
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
        
        # Add enhanced HUD for policy visualizer
        ObjectRendererRegistry.register_handler(
            PolicyMatchHUD, 
            lambda obj, canvas, theme, display_level, renderer=None: obj.draw(canvas, theme, display_level, renderer)
        )
        self.hud = PolicyMatchHUD(self.server.match.game_state, self)
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
        
        interactable_name = None
        interaction_name = None
        
        # Use RL policy if loaded, otherwise fall back to scripted AI
        if self.use_policy and self.rl_module is not None:
            try:
                import torch
                
                # Build observation matching training format
                obs = self._build_observation(name)
                
                # Convert to torch tensor and add batch dimension
                obs_tensor = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
                
                # Get the policy module (MultiAgentRLModule wraps individual policies)
                if hasattr(self.rl_module, 'get'):
                    # Multi-agent case
                    policy_module = self.rl_module.get("versatile_policy")
                else:
                    policy_module = self.rl_module
                
                if policy_module is None:
                    raise ValueError("Could not find 'versatile_policy' in RLModule")
                
                # Use forward_inference to get action distribution
                with torch.no_grad():
                    output = policy_module.forward_inference({"obs": obs_tensor})
                
                # Get action from output (depends on action space type)
                if "action_dist_inputs" in output:
                    # For discrete actions, take argmax of logits
                    logits = output["action_dist_inputs"]
                    action_idx = int(torch.argmax(logits, dim=-1).item())
                elif "actions" in output:
                    action_idx = int(output["actions"].item())
                else:
                    raise ValueError(f"Unknown output format: {output.keys()}")
                
                # Convert action index to (interactable, interaction)
                action_map = self._action_maps.get(name, [])
                if 0 <= action_idx < len(action_map):
                    interactable_name, interaction_name = action_map[action_idx]
                    print(f"[POLICY] {name}: action {action_idx} -> {interactable_name}:{interaction_name}")
                else:
                    print(f"[POLICY] {name}: invalid action index {action_idx}")
                    interactable_name, interaction_name = "WAIT", "NO_OP"
            except Exception as e:
                print(f"[POLICY ERROR] {name}: {e}")
                import traceback
                traceback.print_exc()
                # Fall back to scripted AI
                interactable_name = None
        
        # Fall back to scripted AI if policy didn't provide action
        if interactable_name is None:
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
        
        if interactable_name == "WAIT" or interactable_name is None:
            # Explicit wait or no action
            anim_state.is_waiting = True
            anim_state.wait_time = 0.5
            return

        # Get navigation point
        try:
            nav_point = self.server.match.get_navigation_point(
                interactable_name, interaction_name, name
            )
        except KeyError:
            # Handle case where interactable doesn't exist or is invalid
            print(f"Warning: Invalid interactable '{interactable_name}' for robot {name}")
            anim_state.is_waiting = True
            anim_state.wait_time = 0.5
            return
        
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
