"""
Continuous environment implementations for Gamegine RL.

SingleAgentContinuousEnv: gym.Env for single-agent continuous action training.
MultiAgentContinuousEnv: PettingZoo ParallelEnv for multi-agent continuous action training.
"""
from typing import Any, Dict, List, Optional, Tuple, Type

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from gamegine.representation.game import Game
from gamegine.simulation.GameServer import AbstractGameServer, ContinuousGameServer
from gamegine.rl.config import EnvConfig
from gamegine.rl.envs.base import BaseGamegineEnv
from gamegine.utils.NCIM.Dimensions.spatial import Meter
from gamegine.utils.NCIM.Dimensions.angular import Radian


class SingleAgentContinuousEnv(BaseGamegineEnv, gym.Env):
    """Single-agent continuous action environment.
    
    Uses velocity commands (vx, vy, omega) as actions.
    Physics is stepped at a fixed timestep (config.dt).
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}
    
    def __init__(
        self,
        game: Game,
        server_class: Type[AbstractGameServer] = ContinuousGameServer,
        config: EnvConfig = None,
    ):
        if config is None:
            raise ValueError("EnvConfig must be provided")
        if len(config.robots) != 1:
            raise ValueError("SingleAgentContinuousEnv requires exactly 1 robot in config")
        
        # Ensure we're using ContinuousGameServer
        if server_class == AbstractGameServer.__class__:
            server_class = ContinuousGameServer
        
        super().__init__(game, server_class, config)
        
        # Initialize server
        self._setup_server()
        
        # Set up spaces for the single agent
        self._agent = self.agents[0]
        self.observation_space = self._create_continuous_observation_space(self._agent)
        
        # Continuous action space: [vx, vy, omega] normalized to [-1, 1]
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(3,),
            dtype=np.float32
        )
        
        # Max velocities for denormalization
        self._max_linear_vel = 5.0  # m/s
        self._max_angular_vel = 2 * np.pi  # rad/s
        
    def _create_continuous_observation_space(self, agent: str) -> spaces.Dict:
        """Create observation space with velocity included."""
        base_space = self._create_observation_space(agent)
        
        # Ensure velocity is included for continuous env
        if "robot_velocity" not in base_space.spaces:
            base_space.spaces["robot_velocity"] = spaces.Box(
                low=np.array([-1.0, -1.0, -1.0]),
                high=np.array([1.0, 1.0, 1.0]),
                dtype=np.float32
            )
        
        return base_space
    
    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[Dict[str, np.ndarray], dict]:
        """Reset the environment to initial state."""
        super().reset(seed=seed)
        
        # Reinitialize server
        self._setup_server()
        self._prev_score = 0.0
        self._step_count = 0
        
        # Track velocity for continuous control
        self._current_velocity = np.zeros(3, dtype=np.float32)
        
        obs = self._get_obs(self._agent)
        info = {}
        
        return obs, info
    
    def step(
        self,
        action: np.ndarray,
    ) -> Tuple[Dict[str, np.ndarray], float, bool, bool, dict]:
        """Execute one physics step with velocity command.
        
        :param action: Normalized velocity command [vx, vy, omega] in [-1, 1].
        :returns: (observation, reward, terminated, truncated, info)
        """
        self._step_count += 1
        prev_score = self.server.game_state.score.get()
        
        # Denormalize action to actual velocities
        vx = action[0] * self._max_linear_vel
        vy = action[1] * self._max_linear_vel
        omega = action[2] * self._max_angular_vel
        
        # Apply velocity command to robot
        # Note: This requires ContinuousGameServer to support velocity-based control
        # For now, we approximate by directly updating position
        dt = self.config.dt
        robot_state = self._get_robot_state(self._agent)
        
        # Get current pose
        current_x = robot_state.x.get().to(Meter)
        current_y = robot_state.y.get().to(Meter)
        current_heading = robot_state.heading.get().to(Radian)
        
        # Simple kinematic update (world frame velocities)
        new_x = current_x + vx * dt
        new_y = current_y + vy * dt
        new_heading = current_heading + omega * dt
        
        # Clamp to field bounds
        new_x = np.clip(new_x, 0, self._field_width)
        new_y = np.clip(new_y, 0, self._field_height)
        
        # Update robot state
        from gamegine.utils.NCIM.Dimensions.spatial import Meter as M
        from gamegine.utils.NCIM.Dimensions.angular import Radian as R
        robot_state.x.set(M(new_x))
        robot_state.y.set(M(new_y))
        robot_state.heading.set(R(new_heading))
        
        # Advance game time
        self.server.update(dt)
        
        # Track velocity for observations
        self._current_velocity = np.array([
            vx / self._max_linear_vel,
            vy / self._max_linear_vel,
            omega / self._max_angular_vel
        ], dtype=np.float32)
        
        current_score = self.server.game_state.score.get()
        
        # Compute reward (continuous envs typically don't have "invalid" actions)
        reward = self._compute_reward(self._agent, True, prev_score, current_score)
        
        # Check done
        terminated = self._is_done()
        truncated = False
        
        # Get observation
        obs = self._get_obs(self._agent)
        obs["robot_velocity"] = self._current_velocity
        
        info = {
            "score": current_score,
            "time": self.server.game_state.current_time.get(),
            "velocity": self._current_velocity.copy(),
        }
        
        return obs, reward, terminated, truncated, info
    
    def render(self, mode: str = "human"):
        """Render the environment (placeholder)."""
        pass
    
    def close(self):
        """Clean up resources."""
        pass


class MultiAgentContinuousEnv(BaseGamegineEnv):
    """Multi-agent continuous action environment.
    
    All agents submit velocity commands simultaneously.
    Physics is stepped at a fixed timestep for all agents.
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}
    
    def __init__(
        self,
        game: Game,
        server_class: Type[AbstractGameServer] = ContinuousGameServer,
        config: EnvConfig = None,
    ):
        if config is None:
            raise ValueError("EnvConfig must be provided")
        if len(config.robots) < 2:
            raise ValueError("MultiAgentContinuousEnv requires at least 2 robots in config")
        
        super().__init__(game, server_class, config)
        
        # Initialize server
        self._setup_server()
        
        # PettingZoo attributes
        self.possible_agents = list(self.agents)
        self.agent_name_mapping = {a: i for i, a in enumerate(self.agents)}
        
        # Build spaces for each agent
        self.observation_spaces = {
            a: self._create_continuous_observation_space(a) for a in self.agents
        }
        self.action_spaces = {
            a: spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
            for a in self.agents
        }
        
        # Max velocities
        self._max_linear_vel = 5.0
        self._max_angular_vel = 2 * np.pi
        
        # Track velocities
        self._current_velocities: Dict[str, np.ndarray] = {}
    
    def _create_continuous_observation_space(self, agent: str) -> spaces.Dict:
        """Create observation space with velocity included."""
        base_space = self._create_observation_space(agent)
        
        if "robot_velocity" not in base_space.spaces:
            base_space.spaces["robot_velocity"] = spaces.Box(
                low=np.array([-1.0, -1.0, -1.0]),
                high=np.array([1.0, 1.0, 1.0]),
                dtype=np.float32
            )
        
        return base_space
    
    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[Dict[str, Dict], Dict[str, dict]]:
        """Reset the environment."""
        # Reinitialize server
        self._setup_server()
        self._prev_score = 0.0
        self._step_count = 0
        
        # Reset velocities
        self._current_velocities = {a: np.zeros(3, dtype=np.float32) for a in self.agents}
        
        # All agents are active
        self.agents = list(self.possible_agents)
        
        observations = {a: self._get_obs(a) for a in self.agents}
        for a in self.agents:
            observations[a]["robot_velocity"] = self._current_velocities[a]
        
        infos = {a: {} for a in self.agents}
        
        return observations, infos
    
    def step(
        self,
        actions: Dict[str, np.ndarray],
    ) -> Tuple[
        Dict[str, Dict],
        Dict[str, float],
        Dict[str, bool],
        Dict[str, bool],
        Dict[str, dict],
    ]:
        """Execute physics step for all agents simultaneously."""
        self._step_count += 1
        prev_score = self.server.game_state.score.get()
        dt = self.config.dt
        
        # Apply velocity commands to all agents
        for agent, action in actions.items():
            vx = action[0] * self._max_linear_vel
            vy = action[1] * self._max_linear_vel
            omega = action[2] * self._max_angular_vel
            
            robot_state = self._get_robot_state(agent)
            
            current_x = robot_state.x.get().to(Meter)
            current_y = robot_state.y.get().to(Meter)
            current_heading = robot_state.heading.get().to(Radian)
            
            new_x = np.clip(current_x + vx * dt, 0, self._field_width)
            new_y = np.clip(current_y + vy * dt, 0, self._field_height)
            new_heading = current_heading + omega * dt
            
            from gamegine.utils.NCIM.Dimensions.spatial import Meter as M
            from gamegine.utils.NCIM.Dimensions.angular import Radian as R
            robot_state.x.set(M(new_x))
            robot_state.y.set(M(new_y))
            robot_state.heading.set(R(new_heading))
            
            self._current_velocities[agent] = np.array([
                vx / self._max_linear_vel,
                vy / self._max_linear_vel,
                omega / self._max_angular_vel
            ], dtype=np.float32)
        
        # Advance game time
        self.server.update(dt)
        
        current_score = self.server.game_state.score.get()
        
        # Compute rewards
        rewards = {
            a: self._compute_reward(a, True, prev_score, current_score)
            for a in self.agents
        }
        
        # Check termination
        terminated = self._is_done()
        terminations = {a: terminated for a in self.agents}
        truncations = {a: False for a in self.agents}
        
        # Get observations
        observations = {a: self._get_obs(a) for a in self.agents}
        for a in self.agents:
            observations[a]["robot_velocity"] = self._current_velocities[a]
        
        # Build infos
        infos = {
            a: {
                "velocity": self._current_velocities[a].copy(),
            }
            for a in self.agents
        }
        
        return observations, rewards, terminations, truncations, infos
    
    def render(self, mode: str = "human"):
        """Render the environment (placeholder)."""
        pass
    
    def close(self):
        """Clean up resources."""
        pass
    
    # PettingZoo API compatibility
    def observation_space(self, agent: str) -> spaces.Space:
        return self.observation_spaces[agent]
    
    def action_space(self, agent: str) -> spaces.Space:
        return self.action_spaces[agent]
