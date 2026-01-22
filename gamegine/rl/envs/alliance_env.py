"""
Alliance-based environment for competitive multi-agent training.

Implements a RLlib MultiAgentEnv for:
- Self-play: Both alliances are RL agents competing
- Solo training: One alliance trains, opponent is inactive
- Vs opponent: Train against a frozen/scripted policy

Semi-Markovian Design:
- Each robot tracks its own action execution state
- Time advances to the minimum action completion time
- Only "ready" robots (not busy) can choose new actions
- Busy robots continue executing their current action

Agent IDs follow the pattern: "red_0", "red_1", "blue_0", "blue_1"
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
import functools

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from ray.rllib.env.multi_agent_env import MultiAgentEnv

from gamegine.representation.game import Game
from gamegine.simulation.GameServer import AbstractGameServer, DiscreteGameServer
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.first.alliance import Alliance
from gamegine.rl.config import EnvConfig, RobotConfig, TrainingConfig
from gamegine.rl.envs.base import BaseGamegineEnv
from gamegine.representation.capabilities import RobotCapabilities
from gamegine.utils.NCIM.ncim import Inch, Degree
from gamegine.utils.NCIM.Dimensions.spatial import Meter
from gamegine.utils.NCIM.Dimensions.temporal import Second
import random


@dataclass
class RobotActionState:
    """Tracks a robot's action execution state for semi-markovian dynamics.
    
    Each robot independently tracks:
    - Whether it's currently busy executing an action
    - When the current action will complete
    - The action being executed (for logging/visualization)
    - Any trajectory for smooth animation across steps
    - Accumulated reward during action execution
    """
    is_busy: bool = False
    action_start_time: float = 0.0  # Game time when action started  
    action_end_time: float = 0.0    # Game time when action will complete
    current_action: Optional[Tuple[str, str]] = None  # (interactable, interaction)
    trajectory: Optional[Any] = None  # For visualization continuity
    trajectory_progress: float = 0.0  # How far along trajectory (0.0-1.0)
    pending_reward: float = 0.0       # Reward accumulated during action
    
    def time_remaining(self, current_time: float) -> float:
        """Return time until action completes."""
        if not self.is_busy:
            return 0.0
        return max(0.0, self.action_end_time - current_time)
    
    def is_complete(self, current_time: float) -> bool:
        """Check if action has completed."""
        return not self.is_busy or current_time >= self.action_end_time


class AllianceEnv(MultiAgentEnv):
    """
    Alliance-based multi-agent environment for competitive RL training.
    
    Inherits from RLlib's MultiAgentEnv for full compatibility.
    
    Agent IDs: "red_0", "red_1", ..., "blue_0", "blue_1", ...
    
    Rewards:
    - If share_reward=True: All agents on an alliance get the same reward
    - If competitive_reward=True: reward = Δ(our_score - opponent_score)
    - Otherwise: reward = Δ(our_score)
    """
    
    metadata = {"render_modes": ["human"], "name": "gamegine_alliance_v0"}
    
    def __init__(
        self,
        game: Game,
        server_class: type = DiscreteGameServer,
        config: EnvConfig = None,
    ):
        super().__init__()
        
        # Store references (no longer from BaseGamegineEnv)
        self.game = game
        self.server_class = server_class
        self.config = config
        self.training_config = config.training
        self.training_config = config.training
        self.server = None
        self._action_maps: Dict[str, List[Tuple[str, str]]] = {}
        # Mapping from abstract action (e.g. "OWN_HUB:shoot") to concrete action ("Blue Hub:shoot") per robot
        self._abstract_to_concrete: Dict[str, Dict[Tuple[str, str], Tuple[str, str]]] = {}
        
        # Build agent lists per alliance
        self._red_agents: List[str] = []
        self._blue_agents: List[str] = []
        self._agent_to_robot: Dict[str, str] = {}  # agent_id -> robot_name
        self._robot_to_agent: Dict[str, str] = {}  # robot_name -> agent_id
        
        # Assign agent IDs based on robot config
        red_idx = 0
        blue_idx = 0
        for robot_cfg in config.robots:
            if robot_cfg.team.lower() == "red":
                agent_id = f"red_{red_idx}"
                self._red_agents.append(agent_id)
                red_idx += 1
            else:
                agent_id = f"blue_{blue_idx}"
                self._blue_agents.append(agent_id)
                blue_idx += 1
            
            self._agent_to_robot[agent_id] = robot_cfg.name
            self._robot_to_agent[robot_cfg.name] = agent_id
        
        # RLlib/PettingZoo required attributes
        self._agent_ids = set(self._red_agents + self._blue_agents)
        self.possible_agents = list(self._agent_ids)
        self.agents = self.possible_agents.copy()
        
        # Track previous scores for reward calculation
        self._prev_red_score: int = 0
        self._prev_blue_score: int = 0
        
        # Episode step tracking for truncation
        self._step_count = 0
        self._max_episode_steps = config.max_episode_steps or 100  # Default 100 steps
        
        # Semi-markovian state tracking
        self._game_time: float = 0.0  # Current simulated game time in seconds
        self._robot_action_states: Dict[str, RobotActionState] = {}  # robot_name -> state
        
        # Action duration configuration (can be customized per-game)
        self._default_action_durations = {
            "WAIT": 0.5,        # Idle wait
            "pickup_1": 0.3,   # Quick pickup
            "pickup_5": 1.0,   # Medium pickup  
            "pickup_10": 2.0,  # Slow pickup
            "score_fuel": 1.0, # Scoring at hub
            "shoot": 0.8,      # Shooting from location
            "shuttle": 1.5,    # Shuttling balls
            "climb_level_1": 3.0,
            "climb_level_2": 5.0,
            "climb_level_3": 7.0,
            "defend": 2.0,     # Defense action
        }
        
        # Optimization settings
        self._fast_mode = config.fast_mode
        self._server_pool = None
        if config.use_server_pool:
            from gamegine.rl.server_pool import GameServerPool
            from gamegine.simulation.GameServer import ServerConfig
            server_config = ServerConfig(fast_mode=config.fast_mode)
            self._server_pool = GameServerPool(game, server_class, server_config, pool_size=4)
        
        # Initialize server to get action counts
        self._setup_server()
        
        # Build action maps for each robot
        self._build_action_maps()
        
        # Build observation/action spaces per agent
        self._observation_spaces = {}
        self._action_spaces = {}
        for agent_id in self._agent_ids:
            robot_name = self._agent_to_robot[agent_id]
            self._observation_spaces[agent_id] = self._build_observation_space(robot_name)
            self._action_spaces[agent_id] = self._build_action_space(robot_name)
        
        # RLlib MultiAgentEnv expects observation_space and action_space to be dicts
        self.observation_space = spaces.Dict(self._observation_spaces)
        self.action_space = spaces.Dict(self._action_spaces)
    
    def _setup_server(self) -> None:
        """Initialize the game server and register all robots."""
        from gamegine.simulation.GameServer import ServerConfig
        
        if self._server_pool is not None:
            self.server = self._server_pool.acquire()
        else:
            server_config = ServerConfig(fast_mode=self._fast_mode)
            self.server = self.server_class(server_config)
            self.server.load_from_game(self.game)
        
        # Register robots (needed each reset for pool too)
        for robot_config in self.config.robots:
            robot_name = robot_config.name
            if robot_name not in self.server.robots:
                self.server.match.add_robot(robot_name, robot_config.robot)
            self.server.init_robot(robot_name, robot_config.start_state)
            
            # Inject alliance into state for scoring logic (validation)
            state = self.server.game_state.get_robot(robot_name)
            if state and robot_config.robot:
                state.setValue("alliance", robot_config.robot.alliance)
    
    def _build_action_maps(self) -> None:
        """Build the mapping from abstract action indices to abstract (interactable, interaction) tuples."""
        # Use field size for mirroring logic not needed here, just name parsing
        
        for robot_config in self.config.robots:
            robot_name = robot_config.name
            team = robot_config.team.lower()  # "blue" or "red"
            is_red = (team == "red")
            
            # Get all concrete actions available on the server
            concrete_actions = list(self.server.get_actions_set(robot_name))
            
            # If mirroring is disabled, use concrete actions directly (backward compatibility)
            if not self.training_config.mirror_red_alliance:
                self._action_maps[robot_name] = [("WAIT", "NO_OP")] + concrete_actions
                # Identity mapping
                self._abstract_to_concrete[robot_name] = {a: a for a in self._action_maps[robot_name]}
                continue
                
            # If mirroring enabled, build abstract action space
            abstract_map = []
            concrete_map = {}  # abstract -> concrete
            
            # recursive helper to abstract names
            def abstract_name(name: str) -> str:
                if name.startswith("Blue "):
                    suffix = name[5:]
                    return f"OWN_{suffix}" if team == "blue" else f"OPP_{suffix}"
                elif name.startswith("Red "):
                    suffix = name[4:]
                    return f"OPP_{suffix}" if team == "blue" else f"OWN_{suffix}"
                return name  # e.g. "Neutral Zone"

            # 0. Always add WAIT
            abstract_map.append(("WAIT", "NO_OP"))
            concrete_map[("WAIT", "NO_OP")] = ("WAIT", "NO_OP")
            
            # Process concrete actions
            # We want a CONSISTENT ordering of abstract actions across all robots
            # So we sort them by abstract name
            temp_actions = []
            for interactable, interaction in concrete_actions:
                abs_interactable = abstract_name(interactable)
                # Interaction names typically don't change, but we could abstract them if they had side-specifics
                abs_interaction = interaction 
                
                abs_tuple = (abs_interactable, abs_interaction)
                temp_actions.append((abs_tuple, (interactable, interaction)))
                
            # Sort by abstract tuple to ensure index consistency
            temp_actions.sort(key=lambda x: x[0])
            
            for abs_act, conc_act in temp_actions:
                if abs_act not in concrete_map: # Avoid duplicates if multiple concrete map to same abstract (unlikely)
                    abstract_map.append(abs_act)
                    concrete_map[abs_act] = conc_act
            
            self._action_maps[robot_name] = abstract_map
            self._abstract_to_concrete[robot_name] = concrete_map
    
    def _build_observation_space(self, robot_name: str) -> spaces.Space:
        """Build observation space for an agent."""
        # Basic observation: position, heading, time, scores
        # This can be extended based on config
        obs_dim = 10  # x, y, heading, vx, vy, time, red_score, blue_score, gamepieces, ...
        
        # Semi-markovian state (is_busy flag, time_remaining)
        obs_dim += 2
        
        if self.training_config.use_capability_context:
            obs_dim += RobotCapabilities.vector_size()
            
        if self.server and self.server.match and self.server.match.game:
             obs_dim += self.server.match.game.get_observation_space_size()
        
        # Extended Observations for Stagnation Fix
        # 1. Fuel inventory (1 float)
        # 2. Hub Activity (2 floats: Own, Opponent)
        obs_dim += 3
             
        # Add opponent observations
        if self.training_config.observe_opponent_states:
            # 3 opponents * 5 vars (x,y,theta,vx,vy)
            obs_dim += 3 * 5
            
        if self.training_config.observe_opponent_capabilities:
            # 3 opponents * capability vector size
            obs_dim += 3 * RobotCapabilities.vector_size()

        # Add teammate observations (excluding self)
        # Add teammate observations (excluding self)
        if self.training_config.observe_teammate_states:
            # 2 teammates (assuming 3 vs 3) * 6 vars (x,y,theta,vx,vy,fuel)
            obs_dim += 2 * 6
            
        if self.training_config.observe_teammate_capabilities:
            # 2 teammates * capability vector size
            obs_dim += 2 * RobotCapabilities.vector_size()
            
        return spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
    
    def _build_action_space(self, robot_name: str) -> spaces.Space:
        """Build action space for an agent."""
        # Use the action map from base class (built in _setup_server -> _build_action_maps)
        n_actions = len(self._action_maps.get(robot_name, []))
        return spaces.Discrete(max(1, n_actions))
    
    def _build_action_maps(self) -> None:
        """Map abstract actions to concrete Env actions.
        
        Creates:
        - self._action_maps[robot_name] = [(abstract_interactable, abstract_interaction), ...]
        - self._abstract_to_concrete[robot_name] = [(concrete_interactable, concrete_interaction), ...]
        """
        self._action_maps = {}
        self._abstract_to_concrete = {}
        
        for robot_config in self.config.robots:
            robot_name = robot_config.name
            robot = self.server.robots.get(robot_name)
            if not robot:
                continue
                
            abstract_map = []
            concrete_map = []
            
            # Map standard interactions
            # Interactables are stored in the match controller
            interactables = self.server.match.interactables if self.server.match else {}
            for interactable_name, interactable in interactables.items():
                # Filter interactables that this robot can interact with
                # (For now, all robots can interact with all, masking comes later)
                
                # We need to map Interactable Name -> Abstract Name
                # Blue Hub -> OWN_Hub (if Blue)
                # Red Hub -> OPP_Hub (if Blue)
                
                is_blue = robot_config.team.lower() == "blue"
                
                abstract_target = interactable_name
                # Apply Symmetry Mapping if enabled
                if self.training_config.mirror_red_alliance:
                    # Blue View: Blue=OWN, Red=OPP
                    if is_blue:
                        if "Blue" in interactable_name:
                             abstract_target = interactable_name.replace("Blue", "OWN")
                        elif "Red" in interactable_name:
                             abstract_target = interactable_name.replace("Red", "OPP")
                        elif "BLUE" in interactable_name:
                             abstract_target = interactable_name.replace("BLUE", "OWN")
                        elif "RED" in interactable_name:
                             abstract_target = interactable_name.replace("RED", "OPP")
                    # Red View: Red=OWN, Blue=OPP
                    else:
                        if "Red" in interactable_name:
                             abstract_target = interactable_name.replace("Red", "OWN")
                        elif "Blue" in interactable_name:
                             abstract_target = interactable_name.replace("Blue", "OPP")
                        elif "RED" in interactable_name:
                             abstract_target = interactable_name.replace("RED", "OWN")
                        elif "BLUE" in interactable_name:
                             abstract_target = interactable_name.replace("BLUE", "OPP")
                
                # Get interactions
                for interaction_option in interactable.get_interactions():
                    interaction_name = interaction_option.identifier
                    # For each interaction, add to map
                    abstract_map.append((abstract_target, interaction_name))
                    concrete_map.append((interactable_name, interaction_name))
            
            # Add WAIT action
            abstract_map.append(("WAIT", "NO_OP"))
            concrete_map.append(("WAIT", "NO_OP"))
            
            # Add Shuttling / Navigation abstract actions?
            # For now, discrete space usually implies high level interactions + navigation
            # Navigation is handled by NavigationMap usually or implicit.
            # In this env, we might have added "MoveTo X" actions?
            # The current _build_action_maps loop only does interactions.
            # If we want navigation, we need to add points of interest.
            
            # Assuming the basic set is Interactions + Wait.
            
            # SORT ACTIONS to ensure canonical ordering (Symmetry)
            # This ensures Index X always corresponds to "OWN_Hub:score", regardless of whether OWN was Blue or Red originally.
            combined = list(zip(abstract_map, concrete_map))
            combined.sort(key=lambda x: str(x[0])) # Sort by Abstract Tuple
            
            if combined:
                abstract_map, concrete_map = zip(*combined)
                self._action_maps[robot_name] = list(abstract_map)
                self._abstract_to_concrete[robot_name] = list(concrete_map)
            else:
                self._action_maps[robot_name] = []
                self._abstract_to_concrete[robot_name] = []
    
    def _build_action_space(self, robot_name: str) -> spaces.Space:
        """Build action space for an agent."""
        # Use the action map from base class (built in _setup_server -> _build_action_maps)
        n_actions = len(self._action_maps.get(robot_name, []))
        return spaces.Discrete(max(1, n_actions))
    
    def get_agent_ids(self):
        """Return set of agent IDs (RLlib requirement)."""
        return self._agent_ids
    
    def observation_space_sample(self, agent_ids: Optional[List[str]] = None):
        """Sample observation space for agents."""
        if agent_ids is None:
            agent_ids = list(self._agent_ids)
        return {aid: self._observation_spaces[aid].sample() for aid in agent_ids}
    
    def action_space_sample(self, agent_ids: Optional[List[str]] = None):
        """Sample action space for agents."""
        if agent_ids is None:
            agent_ids = list(self._agent_ids)
        return {aid: self._action_spaces[aid].sample() for aid in agent_ids}
    
    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, dict]]:
        """Reset the environment."""
        if seed is not None:
            np.random.seed(seed)
        
        # Release old server to pool if using pooling
        if self._server_pool is not None and self.server is not None:
            self._server_pool.release(self.server)
        
        # Re-setup server for fresh state
        self._setup_server()
        
        # Reset score and step tracking
        self._prev_red_score = 0
        self._prev_blue_score = 0
        self._step_count = 0
        
        # Reset semi-markovian state
        self._game_time = 0.0
        self._robot_action_states = {}
        for robot_name in self._agent_to_robot.values():
            self._robot_action_states[robot_name] = RobotActionState()
        
        if self.training_config.randomize_start_pose:
            self._randomize_start_poses()

        if self.training_config.randomize_capabilities:
            self._randomize_capabilities()
        
        # Build initial observations
        observations = {}
        infos = {}
        for agent_id in self._agent_ids:
            robot_name = self._agent_to_robot[agent_id]
            observations[agent_id] = self._get_observation(agent_id)
            infos[agent_id] = {"robot_name": robot_name}
            
            # Action Masking
            if self.training_config.use_action_masking:
                infos[agent_id]["action_mask"] = self._get_action_mask(robot_name)
            
        return observations, infos

    def _randomize_start_poses(self) -> None:
        """Randomize robot start positions within defined alliance zones."""
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        
        # Get field dimensions
        field_size = self.game.get_field_size()
        field_length = float(field_size[0].to(Meter))
        field_width = float(field_size[1].to(Meter))
        
        # User defined constraints: 4ft to 6ft on X
        # 1 ft = 0.3048 m
        feet_to_meter = 0.3048
        x_min_blue = 4.0 * feet_to_meter
        x_max_blue = 6.0 * feet_to_meter
        
        # For Red, mirror across field center
        x_min_red = field_length - x_max_blue
        x_max_red = field_length - x_min_blue
        
        y_margin = 0.5 # Avoid walls
        
        for robot_config in self.config.robots:
            robot_state = self.server.game_state.get_robot(robot_config.name)
            if not robot_state:
                continue
                
            team = robot_config.team.lower()
            
            if team == "blue":
                x = np.random.uniform(x_min_blue, x_max_blue)
            else: # red
                x = np.random.uniform(x_min_red, x_max_red)
                
            y = np.random.uniform(y_margin, field_width - y_margin)
            
            # Update robot state (assumes float is Meters)
            robot_state.x.set(Meter(x))
            robot_state.y.set(Meter(y))

    def _randomize_capabilities(self):
        """Randomize robot capabilities based on configuration ranges.
        
        Randomizes:
        - Movement capabilities (speed, acceleration, rotation)
        - Interaction profiles (per-action durations and accuracy)
        - Physical properties (mass, etc.)
        """
        cap_ranges = self.training_config.capability_ranges
        interaction_ranges = self.training_config.interaction_ranges
        physical_ranges = self.training_config.physical_ranges
        
        # Helper to sample float range
        def sample(ranges_dict, key, default):
            if key in ranges_dict:
                low, high = ranges_dict[key]
                return random.uniform(low, high)
            return default
            
        for robot_name in self._agent_to_robot.values():
            robot = self.server.robots.get(robot_name)
            if not robot:
                continue
            
            # Create new random capabilities
            from gamegine.utils.NCIM.ncim import MetersPerSecond, Meter, RadiansPerSecond, Second
            from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
            from gamegine.representation.capabilities import RobotCapabilities, InteractionProfile
            
            # Default baselines from existing capabilities or standard defaults
            base_caps = robot.capabilities or RobotCapabilities()
            
            # Randomize interaction profiles
            new_profiles = {}
            for interaction_name, (min_dur, max_dur) in interaction_ranges.items():
                # Sample duration
                duration = random.uniform(min_dur, max_dur)
                
                # Optionally vary accuracy based on duration (faster robots = less accurate?)
                # For now, use a simple inverse relationship
                # Fastest (min_dur) -> 0.95 accuracy, Slowest (max_dur) -> 1.0 accuracy
                if max_dur > min_dur:
                    speed_factor = (duration - min_dur) / (max_dur - min_dur)  # 0 = fastest, 1 = slowest
                    accuracy = 0.90 + speed_factor * 0.10  # 0.90 to 1.0
                else:
                    accuracy = 1.0
                
                new_profiles[interaction_name] = InteractionProfile(
                    base_duration=Second(duration),
                    accuracy=accuracy,
                )
            
            # Merge with existing profiles (randomized ones override)
            merged_profiles = base_caps.interaction_profiles.copy()
            merged_profiles.update(new_profiles)
            
            new_caps = RobotCapabilities(
                max_speed=MetersPerSecond(sample(cap_ranges, "max_speed", float(base_caps.max_speed))),
                max_acceleration=MeterPerSecondSquared(sample(cap_ranges, "max_acceleration", float(base_caps.max_acceleration))),
                rotational_speed=RadiansPerSecond(sample(cap_ranges, "rotational_speed", float(base_caps.rotational_speed))),
                gamepiece_capacity=base_caps.gamepiece_capacity.copy(),
                interaction_profiles=merged_profiles,
            )
            
            # Apply to robot
            robot.capabilities = new_caps
            
            # Update physical parameters
            if robot.physics:
                robot.physics.max_acceleration = new_caps.max_acceleration
                
                # Randomize mass if configured
                if "mass" in physical_ranges:
                    from gamegine.utils.NCIM.Dimensions.mass import Kilogram
                    new_mass = sample(physical_ranges, "mass", 50.0)
                    robot.physics.mass = Kilogram(new_mass)
    
    def step(
        self,
        actions: Dict[str, int],
    ) -> Tuple[
        Dict[str, np.ndarray],  # observations
        Dict[str, float],       # rewards
        Dict[str, bool],        # terminations
        Dict[str, bool],        # truncations
        Dict[str, dict],        # infos
    ]:
        """Execute actions for ready agents, advance time to next completion.
        
        Semi-Markovian Design:
        1. Only READY robots (not busy) process new actions
        2. Busy robots continue their current action (skip action processing)
        3. Time advances to the minimum action completion time
        4. Any robots that complete are marked ready for next step
        """
        prev_red = self.server.game_state.red_score.get()
        prev_blue = self.server.game_state.blue_score.get()
        
        action_results = {}
        action_names = {}
        newly_started_actions = {}  # robot_name -> end_time
        
        # Phase 1: Process actions for READY robots only
        for agent_id, action_idx in actions.items():
            if agent_id not in self._agent_ids:
                continue
            
            robot_name = self._agent_to_robot[agent_id]
            robot_state = self._robot_action_states.get(robot_name)
            
            # Skip busy robots - they're still executing their current action
            if robot_state and robot_state.is_busy:
                action_names[robot_name] = robot_state.current_action[0] + ":" + robot_state.current_action[1] if robot_state.current_action else "BUSY"
                action_results[robot_name] = True  # Not a failure, just still executing
                continue
            
            # Get abstract action from map
            action_map = self._action_maps.get(robot_name, [])
            if action_idx < 0 or action_idx >= len(action_map):
                action_results[robot_name] = False
                action_names[robot_name] = "INVALID_INDEX"
                continue
            
            abstract_interactable, abstract_interaction = action_map[action_idx]
            
            # Map to concrete action
            concrete_map = self._abstract_to_concrete.get(robot_name, [])
            if action_idx < len(concrete_map):
                interactable_name, interaction_name = concrete_map[action_idx]
            else:
                # Fallback (shouldn't happen)
                interactable_name, interaction_name = abstract_interactable, abstract_interaction

            # Log the ABSTRACT name for training info/consistency (or concrete? usually abstract for debugging policy)
            # Let's log abstract for "action_name" so tensorboard makes sense across alliances
            action_names[robot_name] = f"{abstract_interactable}:{abstract_interaction}"
            
            # Calculate action duration (uses concrete names lookup)
            action_duration = self._get_action_duration(robot_name, interactable_name, interaction_name)
            
            # Start the action (uses concrete names)
            action_success, trajectory = self._start_robot_action(
                robot_name, interactable_name, interaction_name
            )
            action_results[robot_name] = action_success
            
            if action_success:
                # Mark robot as busy
                if robot_state:
                    robot_state.is_busy = True
                    robot_state.action_start_time = self._game_time
                    robot_state.action_end_time = self._game_time + action_duration
                    # Store Abstract action in state for consistency
                    robot_state.current_action = (abstract_interactable, abstract_interaction)
                    robot_state.trajectory = trajectory
                    robot_state.trajectory_progress = 0.0
                    newly_started_actions[robot_name] = robot_state.action_end_time
        
        # Phase 2: Advance time to the minimum completion time
        self._advance_time_to_next_completion()
        
        # Phase 3: Complete any finished actions
        completed_robots = []
        for robot_name, robot_state in self._robot_action_states.items():
            if robot_state.is_busy and robot_state.is_complete(self._game_time):
                # Mark as complete
                robot_state.is_busy = False
                robot_state.trajectory_progress = 1.0
                completed_robots.append(robot_name)
        
        # Get current scores (after time advancement and completions)
        game_state = self.server.game_state
        current_red = game_state.red_score.get()
        current_blue = game_state.blue_score.get()
        
        # Compute rewards
        rewards = self._compute_rewards(current_red, current_blue, action_results, action_names)
        
        # Update previous scores
        self._prev_red_score = current_red
        self._prev_blue_score = current_blue
        
        # Increment step counter
        self._step_count += 1
        
        # Check termination (game time) and truncation (step limit)
        game_over = self.server.match.is_game_over()
        truncated = self._step_count >= self._max_episode_steps
        
        # Build outputs
        observations = {}
        terminations = {}
        truncations = {}
        infos = {}
        
        for agent_id in self._agent_ids:
            robot_name = self._agent_to_robot[agent_id]
            robot_state = self._robot_action_states.get(robot_name)
            
            observations[agent_id] = self._get_observation(agent_id)
            terminations[agent_id] = game_over
            truncations[agent_id] = truncated
            infos[agent_id] = {
                "robot_name": robot_name,
                "action_name": action_names.get(robot_name, "WAIT"),
                "action_valid": action_results.get(robot_name, True),
                "is_busy": robot_state.is_busy if robot_state else False,
                "time_remaining": robot_state.time_remaining(self._game_time) if robot_state else 0.0,
                "trajectory": robot_state.trajectory if robot_state else None,
                "trajectory_progress": robot_state.trajectory_progress if robot_state else 0.0,
                "game_time": self._game_time,
            }
        
        # Add __all__ keys for PettingZoo compatibility
        terminations["__all__"] = game_over
        truncations["__all__"] = truncated
        
        
        # Build masks for next step
        action_masks = {}
        if self.training_config.use_action_masking:
            for agent_id, robot_name in self._agent_to_robot.items():
                # Only need masks for agents that are effectively making decisions
                # (Though RLlib might want masks for all if simple optimizer)
                action_masks[agent_id] = self._get_action_mask(robot_name)
                infos[agent_id]["action_mask"] = action_masks[agent_id]
        
        return observations, rewards, terminations, truncations, infos
    
    def _get_action_duration(self, robot_name: str, interactable_name: str, interaction_name: str) -> float:
        """Calculate how long an action will take.
        
        Uses robot capabilities if available, otherwise falls back to defaults.
        
        For driving + action, this includes:
        - Trajectory travel time (if not fast_mode)
        - Action execution time (from capability profiles)
        """
        # Get robot and its capabilities
        robot = self.server.robots.get(robot_name)
        base_duration = None
        
        # Try to get duration from robot capabilities
        if robot and robot.capabilities:
            # Check both interactable:interaction combo and just interaction_name
            profile = robot.capabilities.get_interaction_profile(interaction_name)
            if profile and float(profile.base_duration) > 0:
                base_duration = float(profile.base_duration)
            else:
                # Try composite key
                composite_key = f"{interactable_name}:{interaction_name}"
                profile = robot.capabilities.get_interaction_profile(composite_key)
                if profile and float(profile.base_duration) > 0:
                    base_duration = float(profile.base_duration)
        
        # Fall back to hardcoded defaults if no capability profile
        if base_duration is None:
            if interactable_name in self._default_action_durations:
                base_duration = self._default_action_durations[interactable_name]
            elif interaction_name in self._default_action_durations:
                base_duration = self._default_action_durations[interaction_name]
            else:
                base_duration = 1.0  # Default fallback
        
        # In fast mode, skip driving time
        if self._fast_mode:
            return base_duration
        
        # For full mode, estimate trajectory time based on distance and robot speed
        try:
            robot_state = self.server.game_state.get_robot(robot_name)
            if robot_state:
                nav_point = self.server.match.get_navigation_point(
                    interactable_name, interaction_name, robot_name
                )
                if nav_point:
                    from gamegine.utils.NCIM.Dimensions.spatial import Meter
                    dx = float((nav_point[0] - robot_state.x.get()).to(Meter))
                    dy = float((nav_point[1] - robot_state.y.get()).to(Meter))
                    distance = (dx**2 + dy**2)**0.5
                    
                    # Use robot's max speed if available, otherwise default
                    avg_speed = 3.0  # Default m/s
                    if robot and robot.capabilities:
                        # Use ~60% of max speed as average for trajectory
                        avg_speed = float(robot.capabilities.max_speed) * 0.6
                        avg_speed = max(avg_speed, 1.0)  # At least 1 m/s
                    
                    drive_time = distance / avg_speed
                    return base_duration + drive_time
        except:
            pass
        
        return base_duration + 1.0  # Default extra time for driving
    
    def _start_robot_action(self, robot_name: str, interactable_name: str, interaction_name: str) -> Tuple[bool, Any]:
        """Start an action for a robot. Returns (success, trajectory)."""
        if interactable_name == "WAIT":
            return True, None
        
        try:
            if self._fast_mode:
                # Fast mode: teleport and process
                nav_point = self.server.match.get_navigation_point(
                    interactable_name, interaction_name, robot_name
                )
                if nav_point is not None:
                    self.server.drive_robot_fast(robot_name, nav_point[0], nav_point[1], nav_point[2])
                success = self.server.process_action(
                    interactable_name, interaction_name, robot_name, None
                )
                return success, None
            else:
                # Full trajectory mode
                result = self.server.drive_and_process_action(
                    interactable_name, interaction_name, robot_name, None
                )
                if isinstance(result, tuple):
                    return result[0], result[1]
                return bool(result), None
        except Exception as e:
            import logging
            logging.warning(f"Action failed for {robot_name}: {interactable_name}:{interaction_name} - {type(e).__name__}: {e}")
            return False, None
    
    def _advance_time_to_next_completion(self):
        """Advance game time to the earliest action completion among busy robots.
        
        This is the key efficiency gain of semi-markovian: we jump directly to
        when the next robot finishes, rather than stepping through time.
        """
        # Find minimum completion time among busy robots
        min_completion = float('inf')
        completing_robot = None
        for robot_name, robot_state in self._robot_action_states.items():
            if robot_state.is_busy and robot_state.action_end_time < min_completion:
                min_completion = robot_state.action_end_time
                completing_robot = robot_name
        
        if min_completion == float('inf') or min_completion <= self._game_time:
            # No busy robots or already past completion - no time advancement needed
            # This happens when all robots just started WAIT actions with 0 duration
            pass
        else:
            # Jump directly to the earliest completion time
            self._game_time = min_completion
        
        # Update trajectory progress for all busy robots based on new time
        for robot_state in self._robot_action_states.values():
            if robot_state.is_busy and robot_state.trajectory:
                total_duration = robot_state.action_end_time - robot_state.action_start_time
                if total_duration > 0:
                    elapsed = self._game_time - robot_state.action_start_time
                    robot_state.trajectory_progress = min(1.0, max(0.0, elapsed / total_duration))
    
    def is_robot_ready(self, agent_id: str) -> bool:
        """Check if a robot is ready to accept a new action."""
        robot_name = self._agent_to_robot.get(agent_id)
        if not robot_name:
            return False
        robot_state = self._robot_action_states.get(robot_name)
        if not robot_state:
            return True
        return not robot_state.is_busy
    
    def _compute_rewards(
        self,
        current_red: int,
        current_blue: int,
        action_results: Dict[str, bool],
        action_names: Dict[str, str],
    ) -> Dict[str, float]:
        """Compute rewards for all agents based on alliance scores and custom function."""
        rewards = {}
        
        # Score deltas
        red_delta = current_red - self._prev_red_score
        blue_delta = current_blue - self._prev_blue_score
        
        # Compute alliance-level rewards
        if self.training_config.competitive_reward:
            # Competitive: reward = Δ(our_score - opponent_score)
            red_reward = red_delta - blue_delta
            blue_reward = blue_delta - red_delta
        else:
            # Solo: reward = Δ(our_score)
            red_reward = red_delta
            blue_reward = blue_delta
        
        # Calculate custom game-specific rewards if configured
        custom_rewards = {}
        if self.config.reward_fn:
            # Gather robot states for the callback
            robot_states = {}
            if self.server and self.server.robots:
                for name, _ in self.server.robots.items():
                    robot_states[name] = self.server.match.game_state.get_robot(name)
            
            custom_rewards = self.config.reward_fn(
                self.server.match.game_state,
                robot_states,
                action_results,
                action_names
            )
        
        # Assign to agents
        for agent_id in self._agent_ids:
            robot_name = self._agent_to_robot[agent_id]
            
            if agent_id.startswith("red"):
                base_reward = red_reward
            else:
                base_reward = blue_reward
            
            # Add invalid action penalty
            if not action_results.get(robot_name, True):
                base_reward += self.config.invalid_action_penalty
                
            # Add custom reward if available (agent_id or robot_name based?)
            # Usually reward_fn returns by robot_name
            if robot_name in custom_rewards:
                base_reward += custom_rewards[robot_name]
            
            if self.training_config.share_reward:
                rewards[agent_id] = float(base_reward)
            else:
                # Individual reward - could add per-robot credit assignment here
                rewards[agent_id] = float(base_reward)
        
        return rewards
    
    def _get_observation(self, agent_id: str) -> np.ndarray:
        """Get observation for an agent."""
        robot_name = self._agent_to_robot[agent_id]
        game_state = self.server.game_state
        robot_state = game_state.get_robot(robot_name)
        
        # Build observation vector
        obs = np.zeros(10, dtype=np.float32)
        
        if robot_state:
            from gamegine.utils.NCIM.Dimensions.spatial import Meter
            from gamegine.utils.NCIM.Dimensions.angular import Degree
            
            field_length = float(self.game.get_field_size()[0].to(Meter))
            field_width = float(self.game.get_field_size()[1].to(Meter))
            
            # Normalize Position (0-1)
            obs[0] = float(robot_state.x.get().to(Meter)) / field_length
            obs[1] = float(robot_state.y.get().to(Meter)) / field_width
            
            # Normalize Heading (0-1)
            obs[2] = float(robot_state.heading.get().to(Degree)) / 360.0
            # Velocities would come from trajectory if available
            obs[3] = 0.0  # vx
            obs[4] = 0.0  # vy
        
        # Normalize Time: 0.0 to 1.0 (Approx 150s max)
        # Using max_episode_steps if available, else 150.0
        # Actually step is abstract. Time is game time. Standard match is 150s (15 auto + 135 teleop)
        # Let's use 150.0 as normalization constant
        obs[5] = float(game_state.current_time.get()) / 150.0
        
        # Normalize Scores: Soft normalization (divide by 100). Scores can go higher but 100 is a reasonable scaling factor.
        obs[6] = float(game_state.red_score.get()) / 100.0
        obs[7] = float(game_state.blue_score.get()) / 100.0
        
        # Semi-markovian state: is_busy and time_remaining
        # Semi-markovian state: is_busy and time_remaining
        action_state = self._robot_action_states.get(robot_name)
        is_busy = 1.0 if (action_state and action_state.is_busy) else 0.0
        
        # Normalize time remaining (Divide by ~20s max duration)
        raw_time_remaining = action_state.time_remaining(self._game_time) if action_state else 0.0
        norm_time_remaining = raw_time_remaining / 20.0
        
        obs = np.concatenate([obs, np.array([is_busy, norm_time_remaining], dtype=np.float32)])
        
        if self.training_config.mirror_red_alliance and "red" in agent_id:
            # === RED ALLIANCE MIRRORING ===
            # Flip inputs so Red sees the world as if it were Blue
            
            # Since normalized 0-1, mirroring is just 1.0 - x
            obs[0] = 1.0 - obs[0]  # x' = 1 - x
            obs[1] = 1.0 - obs[1]  # y' = 1 - y
            
            # Mirror Heading: (h + 0.5) % 1.0
            obs[2] = (obs[2] + 0.5) % 1.0
            field_width = float(self.game.get_field_size()[1].to(Meter))
            

            # Mirror Velocity
            obs[3] = -obs[3]
            obs[4] = -obs[4]
            
            # Swap Scores
            obs[6], obs[7] = obs[7], obs[6]
            
            # Note: We aren't flipping gamepiece observation vectors here yet because
            # we rely on the game-specific `get_observation` which might return raw values.
            # If strict symmetry is needed, `game.get_observation` needs a `mirrored` flag too.
            # Assuming for now game state obs are symmetric enough or handled elsewhere.
            # (RebuiltGame.get_observation returns specific indices for Blue/Red depots/zones)
            # We must swap those too.
            if self.server and self.server.match and self.server.match.game:
                 # Rebuilt obs: [neutral, blue_depot, red_depot, blue_zone, red_zone]
                 # Indices: 0, 1, 2, 3, 4 (appended after cap context)
                 # We need to find where they are in `obs`.
                 # They are appended end. 
                 # Let's handle generic game obs swapping if possible, or leave for later task.
                 # For Rebuilt specifically:
                 obs_len_before_game = 12 + (RobotCapabilities.vector_size() if self.training_config.use_capability_context else 0)
                 # Game obs start at obs_len_before_game
                 # [neutral, blue_d, red_d, blue_z, red_z]
                 # Swap 1<->2 and 3<->4
                 if len(obs) >= obs_len_before_game + 5:
                     base = obs_len_before_game
                     # Current (Blue View): [Neutral, BlueDepot (Own), RedDepot (Opp), BlueZone (Own), RedZone (Opp)]
                     # Target (Red View): [Neutral, RedDepot (Own), BlueDepot (Opp), RedZone (Own), BlueZone (Opp)]
                     # So swap +1 (BlueD) with +2 (RedD)
                     # And swap +3 (BlueZ) with +4 (RedZ)
                     
                     # Note: RebuiltGame.get_observation returns specific indices:
                     # 0=NZ, 1=BlueDepot, 2=RedDepot, 3=BlueZone, 4=RedZone
                     
                     # If I am Red, I want index 1 to be "My Depot" (RedDepot).
                     # Currently index 1 holds BlueDepot.
                     # So I swap 1 and 2. 
                     # Index 2 will then hold BlueDepot (My Opponent's Depot). Correct.
                     obs[base+1], obs[base+2] = obs[base+2], obs[base+1]
                     obs[base+3], obs[base+4] = obs[base+4], obs[base+3]

        # Could add gamepiece counts, opponent positions, etc.
        
        if self.training_config.use_capability_context:
            # Append capability vector
            robot = self.server.robots.get(robot_name)
            if robot and hasattr(robot, "capabilities") and robot.capabilities:
                cap_vec = robot.capabilities.to_vector()
                # Extend obs array
                obs = np.concatenate([obs, np.array(cap_vec, dtype=np.float32)])
            else:
                # Pad with default/zeros if no capabilities
                obs = np.concatenate([obs, np.zeros(RobotCapabilities.vector_size(), dtype=np.float32)])
        
        if self.server and self.server.match and self.server.match.game:
             game_obs = self.server.match.game.get_observation(game_state)
             if game_obs:
                  obs = np.concatenate([obs, np.array(game_obs, dtype=np.float32)])
        
        # === EXTENDED OBSERVATIONS: FUEL & HUB ACTIVITY ===
        # 1. Fuel Inventory
        fuel_count = 0.0
        max_capacity = 1.0 # Avoid div by zero
        if hasattr(robot_state, "gamepieces"):
             # gamepieces is a dict, usually Class -> count
             # Generic sum if we don't know the exact class
             gamepieces = robot_state.gamepieces.get()
             fuel_count = sum(gamepieces.values())
             
             # Get capacity for normalization
             robot_obj = self.server.robots.get(robot_name)
             if robot_obj and robot_obj.capabilities and robot_obj.capabilities.gamepiece_capacity:
                  max_capacity = sum(robot_obj.capabilities.gamepiece_capacity.values())
        
        norm_fuel = min(1.0, fuel_count / max(1.0, max_capacity))
        
        # 2. Hub Activity
        own_hub_active = 0.0
        opp_hub_active = 0.0
        
        # Access interactables
        interactables = self.server.match.interactables if self.server.match else {}
        if interactables:
            # Determine Own/Opp based on robot alliance
            robot_obj = self.server.robots.get(robot_name)
            is_blue = robot_obj.alliance.name.lower() == "blue" if robot_obj else "blue" in agent_id
            
            own_hub_name = "Blue Hub" if is_blue else "Red Hub"
            opp_hub_name = "Red Hub" if is_blue else "Blue Hub"
            
            try:
                if own_hub_name in interactables:
                    own_hub_active = float(interactables[own_hub_name].getValue("is_active").get())
                if opp_hub_name in interactables:
                    opp_hub_active = float(interactables[opp_hub_name].getValue("is_active").get())
            except:
                pass 
        
        obs = np.concatenate([obs, np.array([norm_fuel, own_hub_active, opp_hub_active], dtype=np.float32)])
        
        # Append opponent observations
        if self.training_config.observe_opponent_states or self.training_config.observe_opponent_capabilities:
            max_opponents = 3
            # Identify opponents keys
            if "red" in agent_id:
                opponents = [aid for aid in self._blue_agents]
            else:
                opponents = [aid for aid in self._red_agents]
            
            # Ensure consistent order by sorting
            opponents.sort()
            
            # Helper to get normalized opponent state
            def get_opp_state(opp_idx):
                if opp_idx >= len(opponents):
                    return np.zeros(5, dtype=np.float32)
                
                opp_id = opponents[opp_idx]
                opp_robot_name = self._agent_to_robot[opp_id]
                opp_robot_state = self.server.game_state.get("robots").get(opp_robot_name)
                
                if not opp_robot_state:
                    return np.zeros(5, dtype=np.float32)

                field_size = self.game.get_field_size()
                fx = float(field_size[0].to(Meter))
                fy = float(field_size[1].to(Meter))

                vals = [
                    float(opp_robot_state.x.get().to(Meter)) / fx,
                    float(opp_robot_state.y.get().to(Meter)) / fy,
                    float(opp_robot_state.heading.get().to(Degree)) / 360.0,
                    0.0, # vx placeholder
                    0.0  # vy placeholder
                ]
                
                if self.training_config.mirror_red_alliance and "red" in agent_id:
                     # Mirror Opponent Observations too
                     # Opponent for Red is Blue.
                     # Blue is at (x,y). Red needs to see it mirrored: (L-x, W-y).
                     # Heading + 180.
                     
                     norm_x = vals[0]
                     norm_y = vals[1]
                     norm_heading = vals[2] # 0-1 scalled
                     
                     vals[0] = 1.0 - norm_x
                     vals[1] = 1.0 - norm_y
                     vals[2] = (norm_heading + 0.5) % 1.0
                     
                     vals[3] = -vals[3]
                     vals[4] = -vals[4]

                return np.array(vals, dtype=np.float32)

            # Helper to get opponent capabilities
            def get_opp_caps(opp_idx):
                if opp_idx >= len(opponents):
                    return np.zeros(RobotCapabilities.vector_size(), dtype=np.float32)
                    
                opp_id = opponents[opp_idx]
                opp_robot_name = self._agent_to_robot[opp_id]
                robot = self.server.robots.get(opp_robot_name)
                
                if robot and hasattr(robot, "capabilities") and robot.capabilities:
                    return robot.capabilities.to_vector()
                return np.zeros(RobotCapabilities.vector_size(), dtype=np.float32)

            for i in range(max_opponents):
                if self.training_config.observe_opponent_states:
                    obs = np.concatenate([obs, get_opp_state(i)])
                
                if self.training_config.observe_opponent_capabilities:
                    obs = np.concatenate([obs, get_opp_caps(i)])

        # Append teammate observations
        if self.training_config.observe_teammate_states or self.training_config.observe_teammate_capabilities:
            max_teammates = 2 # Fixed for box space, handle dynamic appropriately in real impl
            
            # Identify teammates (excluding self)
            if "red" in agent_id:
                teammates = [aid for aid in self._red_agents if aid != agent_id]
            else:
                teammates = [aid for aid in self._blue_agents if aid != agent_id]
            
            teammates.sort()
            
            # Helper for teammate state
            def get_tm_state(tm_idx):
                if tm_idx >= len(teammates):
                    return np.zeros(6, dtype=np.float32)
                
                tm_id = teammates[tm_idx]
                tm_name = self._agent_to_robot[tm_id]
                tm_state = self.server.game_state.get("robots").get(tm_name)
                
                if not tm_state:
                    return np.zeros(6, dtype=np.float32)

                field_size = self.game.get_field_size()
                fx = float(field_size[0].to(Meter))
                fy = float(field_size[1].to(Meter))
                
                # Normalize
                # Calculate Teammate Fuel
                fuel_count = 0.0
                max_capacity = 1.0
                if hasattr(tm_state, "gamepieces"):
                     gamepieces = tm_state.gamepieces.get()
                     fuel_count = sum(gamepieces.values())
                     
                     # Get TM capacity
                     tm_robot = self.server.robots.get(tm_name)
                     if tm_robot and tm_robot.capabilities and tm_robot.capabilities.gamepiece_capacity:
                          max_capacity = sum(tm_robot.capabilities.gamepiece_capacity.values())
                
                # Use standard max=100 from user request or robot capacity
                # "robots can hold 100 max generally" - user might mean total gamepieces or a large number.
                # Actually, standard FRC robots hold usually < 5. "100" might be user exaggeration or "100%"?
                # "robots can hold 100 max generally" -> maybe they mean percentage?
                # But context implies quantity. If I use max(1.0, max_capacity), it is robust.
                norm_fuel = min(1.0, fuel_count / max(1.0, max_capacity))

                # Normalize State
                vals = [
                    float(tm_state.x.get().to(Meter)) / fx,
                    float(tm_state.y.get().to(Meter)) / fy,
                    float(tm_state.heading.get().to(Degree)) / 360.0,
                    0.0, 0.0,       # vx, vy
                    norm_fuel       # fuel
                ]
                
                # Apply mirroring if we are Red
                if self.training_config.mirror_red_alliance and "red" in agent_id:
                     # Teammate is also Red.
                     # We must mirror their coordinates so they appear consistent with "My" mirrored world.
                     # If I am at x'=L-x, my teammate at x2 should be x2'=L-x2.
                     
                     norm_x = vals[0]
                     norm_y = vals[1]
                     norm_heading = vals[2]
                     
                     vals[0] = 1.0 - norm_x
                     vals[1] = 1.0 - norm_y
                     vals[2] = (norm_heading + 0.5) % 1.0
                     vals[3] = -vals[3]
                     vals[4] = -vals[4]
                     # Fuel (vals[5]) does not need mirroring
                     
                return np.array(vals, dtype=np.float32)

            # Helper for teammate capabilities
            def get_tm_caps(tm_idx):
                if tm_idx >= len(teammates):
                    return np.zeros(RobotCapabilities.vector_size(), dtype=np.float32)
                
                tm_id = teammates[tm_idx]
                tm_name = self._agent_to_robot[tm_id]
                robot = self.server.robots.get(tm_name)
                
                if robot and hasattr(robot, "capabilities") and robot.capabilities:
                    return robot.capabilities.to_vector()
                return np.zeros(RobotCapabilities.vector_size(), dtype=np.float32)

            for i in range(max_teammates):
                if self.training_config.observe_teammate_states:
                    obs = np.concatenate([obs, get_tm_state(i)])
                
                if self.training_config.observe_teammate_capabilities:
                    obs = np.concatenate([obs, get_tm_caps(i)])

        return obs
    
    def render(self, mode: str = "human"):
        """Render the environment."""
        pass  # Can integrate with renderer if needed
    
    def close(self):
        """Clean up resources."""
        pass
    
    # RLlib helper methods
    def get_agent_groups(self) -> Dict[str, List[str]]:
        """Get agent groupings for RLlib."""
        return {
            "red_team": self._red_agents.copy(),
            "blue_team": self._blue_agents.copy(),
        }
    
    def get_policy_mapping_fn(self):
        """Get policy mapping function for RLlib asymmetric training."""
        red_policy = self.config.red_alliance.policy_id if self.config.red_alliance else "default"
        blue_policy = self.config.blue_alliance.policy_id if self.config.blue_alliance else "default"
        
        def policy_mapping(agent_id, episode, **kwargs):
            if agent_id.startswith("red"):
                return red_policy
            return blue_policy
        
        return policy_mapping
    def _get_action_mask(self, robot_name: str) -> np.ndarray:
        """Get action mask for a robot. 1=Valid, 0=Invalid."""
        action_map = self._action_maps.get(robot_name, [])
        n_actions = len(action_map)
        mask = np.ones(n_actions, dtype=np.float32)
        
        # Get robot state for inventory
        game_state = self.server.game_state
        robot_state = game_state.get_robot(robot_name)
        if not robot_state:
            return mask
            
        fuel_count = 0
        if hasattr(robot_state, "gamepieces"):
             gamepieces = robot_state.gamepieces.get()
             fuel_count = sum(gamepieces.values())
        
        robot_obj = self.server.robots.get(robot_name)
        max_capacity = 5
        if robot_obj and robot_obj.capabilities and robot_obj.capabilities.gamepiece_capacity:
             max_capacity = sum(robot_obj.capabilities.gamepiece_capacity.values())
             
        # Check explicit constraints
        for idx, (interactable, interaction) in enumerate(action_map):
            # action_map elements are (interactable_name, interaction_name)
            
            # Mask SCORING or SHUTTLING if empty
            if ("score" in interaction.lower() or "shuttle" in interaction.lower()) and fuel_count <= 0:
                mask[idx] = 0.0
                
            # Mask PICKUP if:
            # 1. Robot is full
            # 2. Zone is empty
            elif "pickup" in interaction.lower():
                # Check Robot Full
                if fuel_count >= max_capacity:
                    mask[idx] = 0.0
                    continue
                
                # Check Zone Empty
                # We need to map concrete interactable name to object
                # Note: `interactable` here IS the ABSTRACT name from action_map, but we need CONCRETE to look up state.
                # `action_map` stores (AbstractInteractable, AbstractInteraction)
                # But to check state, we need the concrete Interactable object.
                # We can use `self._abstract_to_concrete` mapping or search.
                # However, `_get_action_mask` iterates `action_map`.
                # We should look up the concrete one.
                
                # Retrieve concrete mapping
                concrete_intr_name = interactable # Default if no mapping
                concrete_interaction = interaction
                
                if robot_name in self._abstract_to_concrete:
                    # abstract_map and concrete_map are parallel lists in `_build_action_maps`
                    # BUT `_abstract_to_concrete` is a dict? No, in `_build_action_maps` it was made a list?
                    # Line 393: `self._abstract_to_concrete[robot_name] = list(concrete_map)`
                    # So it's a list parallel to action_map.
                    
                    concrete_list = self._abstract_to_concrete.get(robot_name, [])
                    if idx < len(concrete_list):
                        concrete_intr_name, concrete_interaction = concrete_list[idx]
                
                # Now get the interactable object
                intr_obj = self.server.match.interactables.get(concrete_intr_name)
                if intr_obj:
                    # Get fuel available from state
                    # Interactable state is usually `intr_obj.state`? 
                    # No, states are stored in GameState.
                    intr_state = game_state.get("interactables").get(concrete_intr_name)
                    if intr_state and hasattr(intr_state, "fuel_available"):
                        available = intr_state.getValue("fuel_available").get()
                        
                        # Determine amount needed
                        amount = 1 # Default
                        if "pickup_5" in interaction:
                            amount = 5
                        elif "pickup_10" in interaction:
                            amount = 10
                            
                        if available < amount:
                            mask[idx] = 0.0
                
        return mask
