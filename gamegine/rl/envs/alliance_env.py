"""
Alliance-based environment for competitive multi-agent training.

Implements a RLlib MultiAgentEnv for:
- Self-play: Both alliances are RL agents competing
- Solo training: One alliance trains, opponent is inactive
- Vs opponent: Train against a frozen/scripted policy

Agent IDs follow the pattern: "red_0", "red_1", "blue_0", "blue_1"
"""
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
        self.server = None
        self._action_maps: Dict[str, List[Tuple[str, str]]] = {}
        
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
    
    def _build_action_maps(self) -> None:
        """Build the mapping from action indices to (interactable, interaction) tuples."""
        for robot_config in self.config.robots:
            robot_name = robot_config.name
            actions = self.server.get_actions_set(robot_name)
            # Add WAIT/NO_OP as action 0
            self._action_maps[robot_name] = [("WAIT", "NO_OP")] + list(actions)
    
    def _build_observation_space(self, robot_name: str) -> spaces.Space:
        """Build observation space for an agent."""
        # Basic observation: position, heading, time, scores
        # This can be extended based on config
        obs_dim = 10  # x, y, heading, vx, vy, time, red_score, blue_score, gamepieces, ...
        return spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
    
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
        
        # Build initial observations
        observations = {}
        infos = {}
        for agent_id in self._agent_ids:
            robot_name = self._agent_to_robot[agent_id]
            observations[agent_id] = self._get_observation(agent_id)
            infos[agent_id] = {"robot_name": robot_name}
        
        return observations, infos
    
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
        """Execute actions for all agents."""
        prev_score = self.server.game_state.score.get()
        prev_red = self.server.game_state.red_score.get()
        prev_blue = self.server.game_state.blue_score.get()
        
        action_results = {}
        action_names = {}
        
        for agent_id, action_idx in actions.items():
            if agent_id not in self._agent_ids:
                continue
            
            robot_name = self._agent_to_robot[agent_id]
            
            # Get action from action map
            action_map = self._action_maps.get(robot_name, [])
            if action_idx < 0 or action_idx >= len(action_map):
                action_results[robot_name] = False
                action_names[robot_name] = "INVALID_INDEX"
                continue
            
            interactable_name, interaction_name = action_map[action_idx]
            action_names[robot_name] = f"{interactable_name}:{interaction_name}"
            
            # WAIT action
            if interactable_name == "WAIT":
                action_results[robot_name] = True
                continue
            
            # Execute action (fast mode or full trajectory)
            try:
                if self._fast_mode:
                    # Fast mode: get nav point and teleport
                    nav_point = self.server.match.get_navigation_point(
                        interactable_name, interaction_name, robot_name
                    )
                    if nav_point is not None:
                        self.server.drive_robot_fast(robot_name, nav_point[0], nav_point[1], nav_point[2])
                    # Process the action
                    action_success = self.server.process_action(
                        interactable_name, interaction_name, robot_name, None
                    )
                    action_results[robot_name] = action_success
                else:
                    # Full trajectory mode
                    result = self.server.drive_and_process_action(
                        interactable_name, 
                        interaction_name, 
                        robot_name,
                        None  # No time cutoff
                    )
                    # Handle (success, trajectory) return type
                    if isinstance(result, tuple):
                        action_results[robot_name] = result[0]
                    else:
                        action_results[robot_name] = bool(result)
            except Exception as e:
                action_results[robot_name] = False
        
        # Get current scores
        game_state = self.server.game_state
        current_red = game_state.red_score.get()
        current_blue = game_state.blue_score.get()
        
        # Compute rewards
        rewards = self._compute_rewards(current_red, current_blue, action_results)
        
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
            observations[agent_id] = self._get_observation(agent_id)
            terminations[agent_id] = game_over
            truncations[agent_id] = truncated
            infos[agent_id] = {
                "robot_name": robot_name,
                "action_name": action_names.get(robot_name, "WAIT"),
                "action_valid": action_results.get(robot_name, True),
            }
        
        # Add __all__ keys for PettingZoo compatibility
        terminations["__all__"] = game_over
        truncations["__all__"] = truncated
        
        return observations, rewards, terminations, truncations, infos
    
    def _compute_rewards(
        self,
        current_red: int,
        current_blue: int,
        action_results: Dict[str, bool],
    ) -> Dict[str, float]:
        """Compute rewards for all agents based on alliance scores."""
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
            obs[0] = float(robot_state.x.get())
            obs[1] = float(robot_state.y.get())
            obs[2] = float(robot_state.heading.get())
            # Velocities would come from trajectory if available
            obs[3] = 0.0  # vx
            obs[4] = 0.0  # vy
        
        obs[5] = float(game_state.current_time.get())
        obs[6] = float(game_state.red_score.get())
        obs[7] = float(game_state.blue_score.get())
        
        # Could add gamepiece counts, opponent positions, etc.
        
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
