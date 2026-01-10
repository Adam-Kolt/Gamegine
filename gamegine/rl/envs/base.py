"""
Base environment class for Gamegine RL environments.

Provides common functionality for observation/action space generation,
reward computation, and game state management.
"""
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple, Type, Union

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from gamegine.representation.game import Game
from gamegine.simulation.GameServer import AbstractGameServer, DiscreteGameServer
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.rl.config import EnvConfig, RobotConfig
from gamegine.utils.NCIM.Dimensions.spatial import Meter
from gamegine.utils.NCIM.Dimensions.angular import Degree


class BaseGamegineEnv(ABC):
    """Abstract base class for all Gamegine RL environments.
    
    Provides:
    - Automatic observation space generation from GameServer introspection
    - Automatic action space generation from available interactions
    - Default reward computation with invalid action penalties
    - Game state management and reset logic
    """
    
    def __init__(
        self,
        game: Game,
        server_class: Type[AbstractGameServer],
        config: EnvConfig,
    ):
        """Initialize the base environment.
        
        :param game: The Game definition (field, obstacles, interactables).
        :param server_class: The GameServer class to instantiate (Discrete or Continuous).
        :param config: Environment configuration.
        """
        self.game = game
        self.server_class = server_class
        self.config = config
        
        # Will be initialized in reset()
        self.server: Optional[AbstractGameServer] = None
        self._action_maps: Dict[str, List[Tuple[str, str]]] = {}
        self._prev_score: float = 0.0
        self._step_count: int = 0
        
        # Cache field size for observation normalization
        field_size = game.get_field_size()
        self._field_width = field_size[0].to(Meter)
        self._field_height = field_size[1].to(Meter)
        
        # Agent names from config
        self.agents = [rc.name for rc in config.robots]
        self._robot_configs: Dict[str, RobotConfig] = {rc.name: rc for rc in config.robots}
        
    def _setup_server(self) -> None:
        """Initialize the game server and register all robots."""
        self.server = self.server_class()
        self.server.load_from_game(self.game)
        
        for robot_config in self.config.robots:
            # Use config name instead of robot.name to allow agent-specific naming
            robot_name = robot_config.name
            self.server.match.add_robot(robot_name, robot_config.robot)
            self.server.init_robot(robot_name, robot_config.start_state)
            
            # Initialize robot interaction configs
            for interaction_config in robot_config.robot.interaction_configs:
                # These are already on the robot, just ensure they're registered
                pass
        
        # Build action maps for each robot
        self._build_action_maps()
        
    def _build_action_maps(self) -> None:
        """Build the mapping from action indices to (interactable, interaction) tuples."""
        for agent in self.agents:
            actions = self.server.get_actions_set(agent)
            # Add WAIT/NO_OP as action 0
            self._action_maps[agent] = [("WAIT", "NO_OP")] + list(actions)
    
    def _create_observation_space(self, agent: str) -> spaces.Dict:
        """Create observation space for a single agent.
        
        Introspects the game and robot state to build a comprehensive observation.
        """
        obs_spaces = {}
        
        # Robot pose: x, y, heading (normalized)
        obs_spaces["robot_pose"] = spaces.Box(
            low=np.array([0.0, 0.0, -1.0]),
            high=np.array([1.0, 1.0, 1.0]),
            dtype=np.float32
        )
        
        # Robot velocity: vx, vy, omega (if enabled)
        if self.config.include_velocity_obs:
            obs_spaces["robot_velocity"] = spaces.Box(
                low=np.array([-1.0, -1.0, -1.0]),
                high=np.array([1.0, 1.0, 1.0]),
                dtype=np.float32
            )
        
        # Gamepieces (if enabled)
        if self.config.include_gamepiece_obs:
            # Count unique gamepiece types from the game
            # Default to 2 types (common in FRC: coral, algae, etc.)
            obs_spaces["robot_gamepieces"] = spaces.Box(
                low=0, high=10, shape=(2,), dtype=np.int32
            )
        
        # Game state
        obs_spaces["game_time"] = spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32)
        obs_spaces["game_score"] = spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32)
        
        # Teammate observations
        teammates = [a for a in self.agents if a != agent and self._is_teammate(agent, a)]
        if teammates:
            obs_spaces["teammate_poses"] = spaces.Box(
                low=-1.0, high=1.0, shape=(len(teammates) * 3,), dtype=np.float32
            )
            if self.config.include_velocity_obs:
                obs_spaces["teammate_velocities"] = spaces.Box(
                    low=-1.0, high=1.0, shape=(len(teammates) * 3,), dtype=np.float32
                )
        
        # Opponent observations (if enabled)
        if self.config.include_opponent_obs:
            opponents = [a for a in self.agents if a != agent and not self._is_teammate(agent, a)]
            if opponents:
                obs_spaces["opponent_poses"] = spaces.Box(
                    low=-1.0, high=1.0, shape=(len(opponents) * 3,), dtype=np.float32
                )
                if self.config.include_velocity_obs:
                    obs_spaces["opponent_velocities"] = spaces.Box(
                        low=-1.0, high=1.0, shape=(len(opponents) * 3,), dtype=np.float32
                    )
        
        return spaces.Dict(obs_spaces)
    
    def _create_action_space(self, agent: str) -> spaces.Discrete:
        """Create action space for a single agent.
        
        Uses the action map built from GameServer.get_actions_set().
        """
        num_actions = len(self._action_maps.get(agent, []))
        if num_actions == 0:
            # Fallback: at least WAIT action
            num_actions = 1
        return spaces.Discrete(num_actions)
    
    def _is_teammate(self, agent1: str, agent2: str) -> bool:
        """Check if two agents are on the same team."""
        team1 = self._robot_configs[agent1].team
        team2 = self._robot_configs[agent2].team
        return team1 == team2
    
    def _get_robot_state(self, agent: str) -> RobotState:
        """Get the current RobotState for an agent."""
        return self.server.game_state.get("robots").get(agent)
    
    def _get_obs(self, agent: str) -> Dict[str, np.ndarray]:
        """Build observation dictionary for an agent."""
        robot_state = self._get_robot_state(agent)
        game_state = self.server.game_state
        
        obs = {}
        
        # Robot pose (normalized)
        x = robot_state.x.get().to(Meter) / self._field_width
        y = robot_state.y.get().to(Meter) / self._field_height
        heading = robot_state.heading.get().to(Degree) / 180.0  # Normalize to [-1, 1]
        obs["robot_pose"] = np.array([x, y, heading], dtype=np.float32)
        
        # Robot velocity (if available and enabled)
        if self.config.include_velocity_obs:
            # Default to zeros if velocity not tracked
            obs["robot_velocity"] = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        
        # Gamepieces
        if self.config.include_gamepiece_obs:
            gamepieces = robot_state.gamepieces.get()
            # Convert to array (assumes max 2 gamepiece types for now)
            gp_counts = list(gamepieces.values()) if gamepieces else [0, 0]
            while len(gp_counts) < 2:
                gp_counts.append(0)
            obs["robot_gamepieces"] = np.array(gp_counts[:2], dtype=np.int32)
        
        # Game time (normalized by total match time)
        total_time = game_state.auto_time.get() + game_state.teleop_time.get()
        current_time = game_state.current_time.get()
        obs["game_time"] = np.array([current_time / max(total_time, 1)], dtype=np.float32)
        
        # Game score (normalized, assume max ~400 points)
        obs["game_score"] = np.array([game_state.score.get() / 400.0], dtype=np.float32)
        
        # Teammate poses
        teammates = [a for a in self.agents if a != agent and self._is_teammate(agent, a)]
        if teammates:
            teammate_poses = []
            teammate_velocities = []
            for tm in teammates:
                tm_state = self._get_robot_state(tm)
                teammate_poses.extend([
                    tm_state.x.get().to(Meter) / self._field_width,
                    tm_state.y.get().to(Meter) / self._field_height,
                    tm_state.heading.get().to(Degree) / 180.0
                ])
                if self.config.include_velocity_obs:
                    teammate_velocities.extend([0.0, 0.0, 0.0])  # Placeholder
            obs["teammate_poses"] = np.array(teammate_poses, dtype=np.float32)
            if self.config.include_velocity_obs:
                obs["teammate_velocities"] = np.array(teammate_velocities, dtype=np.float32)
        
        # Opponent poses
        if self.config.include_opponent_obs:
            opponents = [a for a in self.agents if a != agent and not self._is_teammate(agent, a)]
            if opponents:
                opponent_poses = []
                opponent_velocities = []
                for op in opponents:
                    op_state = self._get_robot_state(op)
                    opponent_poses.extend([
                        op_state.x.get().to(Meter) / self._field_width,
                        op_state.y.get().to(Meter) / self._field_height,
                        op_state.heading.get().to(Degree) / 180.0
                    ])
                    if self.config.include_velocity_obs:
                        opponent_velocities.extend([0.0, 0.0, 0.0])  # Placeholder
                obs["opponent_poses"] = np.array(opponent_poses, dtype=np.float32)
                if self.config.include_velocity_obs:
                    obs["opponent_velocities"] = np.array(opponent_velocities, dtype=np.float32)
        
        return obs
    
    def _compute_reward(
        self,
        agent: str,
        action_valid: bool,
        prev_score: float,
        current_score: float,
    ) -> float:
        """Compute reward for an agent.
        
        Default: Δscore + penalty for invalid actions.
        Override via config.reward_fn for custom rewards.
        """
        if self.config.reward_fn is not None:
            # Use custom reward function
            robot_states = {a: self._get_robot_state(a) for a in self.agents}
            action_valids = {agent: action_valid}
            rewards = self.config.reward_fn(
                self.server.game_state,
                robot_states,
                action_valids
            )
            return rewards.get(agent, 0.0)
        
        # Default reward
        reward = current_score - prev_score
        
        if not action_valid:
            reward += self.config.invalid_action_penalty
        
        return reward
    
    def _is_done(self) -> bool:
        """Check if the episode is complete."""
        # Game over by time
        if self.server.match.is_game_over():
            return True
        
        # Max steps exceeded
        if self.config.max_episode_steps and self._step_count >= self.config.max_episode_steps:
            return True
        
        return False
    
    @abstractmethod
    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        """Reset the environment."""
        pass
    
    @abstractmethod
    def step(self, action):
        """Take a step in the environment."""
        pass
