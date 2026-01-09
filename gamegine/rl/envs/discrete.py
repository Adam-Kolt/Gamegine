"""
Discrete environment implementations for Gamegine RL.

SingleAgentDiscreteEnv: gym.Env for single-agent discrete action training.
MultiAgentDiscreteEnv: PettingZoo ParallelEnv for multi-agent discrete action training.
"""
from typing import Any, Dict, List, Optional, Tuple, Type

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from gamegine.representation.game import Game
from gamegine.simulation.GameServer import AbstractGameServer, DiscreteGameServer
from gamegine.rl.config import EnvConfig
from gamegine.rl.envs.base import BaseGamegineEnv
from gamegine.utils.NCIM.ncim import Second


class SingleAgentDiscreteEnv(BaseGamegineEnv, gym.Env):
    """Single-agent discrete action environment.
    
    Wraps a Gamegine Game/Server as a standard Gymnasium environment.
    Each step executes one complete action (drive + interact).
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(
        self,
        game: Game,
        server_class: Type[AbstractGameServer] = DiscreteGameServer,
        config: EnvConfig = None,
    ):
        if config is None:
            raise ValueError("EnvConfig must be provided")
        if len(config.robots) != 1:
            raise ValueError("SingleAgentDiscreteEnv requires exactly 1 robot in config")
        
        super().__init__(game, server_class, config)
        
        # Initialize server to build action maps
        self._setup_server()
        
        # Set up spaces for the single agent
        self._agent = self.agents[0]
        self.observation_space = self._create_observation_space(self._agent)
        self.action_space = self._create_action_space(self._agent)
        
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
        
        obs = self._get_obs(self._agent)
        info = {}
        
        return obs, info
    
    def step(
        self,
        action: int,
    ) -> Tuple[Dict[str, np.ndarray], float, bool, bool, dict]:
        """Execute one action and return the result.
        
        :param action: Action index from action_space.
        :returns: (observation, reward, terminated, truncated, info)
        """
        self._step_count += 1
        prev_score = self.server.game_state.score.get()
        
        # Get action tuple
        action_tuple = self._action_maps[self._agent][action]
        
        action_valid = True
        if action_tuple[0] == "WAIT":
            # NO_OP: just advance a small amount of time
            self.server.update(0.1)
        else:
            # Execute the action
            try:
                interactable, interaction = action_tuple
                result = self.server.drive_and_process_action(
                    interactable,
                    interaction,
                    self._agent,
                    self.server.game_state.current_time.get()
                )
                # Handle both old (bool) and new (tuple) return types
                if isinstance(result, tuple):
                    action_valid = result[0]
                else:
                    action_valid = result
            except Exception:
                action_valid = False
        
        current_score = self.server.game_state.score.get()
        
        # Compute reward
        reward = self._compute_reward(self._agent, action_valid, prev_score, current_score)
        
        # Check done
        terminated = self._is_done()
        truncated = False
        
        # Get observation
        obs = self._get_obs(self._agent)
        
        info = {
            "action_valid": action_valid,
            "score": current_score,
            "time": self.server.game_state.current_time.get(),
        }
        
        return obs, reward, terminated, truncated, info
    
    def render(self, mode: str = "human"):
        """Render the environment (placeholder)."""
        pass
    
    def close(self):
        """Clean up resources."""
        pass


class MultiAgentDiscreteEnv(BaseGamegineEnv):
    """Multi-agent discrete action environment using true SMDP event-stepping.
    
    Implements a PettingZoo-compatible ParallelEnv interface with proper
    Semi-Markov Decision Process semantics:
    
    1. Only FREE agents are asked for actions each step
    2. Agents commit to actions with computed durations
    3. Time advances to the NEXT event (earliest completion)
    4. Completed actions are resolved at their finish time
    5. Busy agents continue their trajectories across steps
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(
        self,
        game: Game,
        server_class: Type[AbstractGameServer] = DiscreteGameServer,
        config: EnvConfig = None,
    ):
        if config is None:
            raise ValueError("EnvConfig must be provided")
        if len(config.robots) < 2:
            raise ValueError("MultiAgentDiscreteEnv requires at least 2 robots in config")
        
        super().__init__(game, server_class, config)
        
        # SMDP state tracking
        self._busy_until: Dict[str, float] = {}  # agent -> timestamp when action completes
        self._pending_actions: Dict[str, Tuple[str, str]] = {}  # agent -> (interactable, interaction)
        self._pending_trajectories: Dict[str, Any] = {}  # agent -> trajectory object
        self._pending_nav_points: Dict[str, Any] = {}  # agent -> (x, y, theta) destination
        
        # Trajectory tracking for visualization
        self._current_trajectories: Dict[str, Any] = {}  # agent -> trajectory (for current step)
        self._trajectory_start_times: Dict[str, float] = {}  # agent -> when trajectory started
        
        # Initialize server to build action maps
        self._setup_server()
        
        # PettingZoo attributes
        self.possible_agents = list(self.agents)
        self.agent_name_mapping = {a: i for i, a in enumerate(self.agents)}
        
        # Build spaces for each agent
        self.observation_spaces = {a: self._create_observation_space(a) for a in self.agents}
        self.action_spaces = {a: self._create_action_space(a) for a in self.agents}
        
    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> Tuple[Dict[str, Dict], Dict[str, dict]]:
        """Reset the environment.
        
        :returns: (observations, infos) dicts keyed by agent name.
        """
        # Reinitialize server
        self._setup_server()
        self._prev_score = 0.0
        self._step_count = 0
        
        # Reset SMDP state - all agents start free at time 0
        current_time = self.server.game_state.current_time.get()
        self._busy_until = {a: current_time for a in self.agents}
        self._pending_actions = {}
        self._pending_trajectories = {}
        self._pending_nav_points = {}
        self._current_trajectories = {}
        self._trajectory_start_times = {}
        
        # Clear active trajectories from physics engine (for dynamic obstacle avoidance)
        if hasattr(self.server, 'physics_engine'):
            self.server.physics_engine.clear_all_active_trajectories()
        
        # All agents are active
        self.agents = list(self.possible_agents)
        
        observations = {a: self._get_obs(a) for a in self.agents}
        infos = {a: {"is_free": True} for a in self.agents}
        
        return observations, infos
    
    def step(
        self,
        actions: Dict[str, int],
    ) -> Tuple[
        Dict[str, Dict],  # observations
        Dict[str, float],  # rewards
        Dict[str, bool],   # terminations
        Dict[str, bool],   # truncations
        Dict[str, dict],   # infos
    ]:
        """Execute one SMDP step.
        
        This implementation uses drive_and_process_action which generates trajectories
        properly. Time advances based on the shortest action completion, allowing
        agents to complete at different times.
        
        :param actions: Dict mapping agent name -> action index.
        """
        self._step_count += 1
        prev_score = self.server.game_state.score.get()
        current_time = self.server.game_state.current_time.get()
        
        action_valid = {a: True for a in self.agents}
        action_names = {a: "WAIT" for a in self.agents}
        trajectories = {a: None for a in self.agents}
        action_times = {a: 0.0 for a in self.agents}
        
        # Execute each free agent's action
        for agent, action in actions.items():
            if not self._is_agent_free(agent, current_time):
                action_names[agent] = "BUSY"
                continue
            
            action_tuple = self._action_maps[agent][action]
            
            if action_tuple[0] == "WAIT":
                action_times[agent] = 0.1
                self._busy_until[agent] = current_time + 0.1
                action_names[agent] = "WAIT"
            else:
                interactable, interaction = action_tuple
                action_names[agent] = f"{interactable}:{interaction}"
                
                try:
                    time_before = self.server.game_state.current_time.get()
                    
                    # Use drive_and_process_action which generates trajectories
                    result = self.server.drive_and_process_action(
                        interactable,
                        interaction,
                        agent,
                        None  # No time cutoff
                    )
                    
                    time_after = self.server.game_state.current_time.get()
                    action_times[agent] = time_after - time_before
                    
                    # Handle (success, trajectory) return type
                    if isinstance(result, tuple):
                        action_valid[agent] = result[0]
                        trajectories[agent] = result[1] if len(result) > 1 else None
                    else:
                        action_valid[agent] = bool(result)
                        trajectories[agent] = None
                    
                    # Register trajectory for dynamic obstacle avoidance by other robots
                    if trajectories[agent] is not None and hasattr(self.server, 'physics_engine'):
                        robot = self.server.robots.get(agent)
                        if robot:
                            from gamegine.utils.NCIM.Dimensions.spatial import Meter
                            radius = robot.get_bounding_radius()
                            radius_m = radius.to(Meter) if hasattr(radius, 'to') else float(radius)
                            self.server.physics_engine.register_active_trajectory(
                                agent,
                                trajectories[agent],
                                time_before,
                                radius_m,
                            )
                    
                    # Update busy_until based on actual action time
                    self._busy_until[agent] = time_after
                    
                except Exception as e:
                    action_valid[agent] = False
                    action_times[agent] = 0.1
                    self._busy_until[agent] = current_time + 0.1
        
        new_time = self.server.game_state.current_time.get()
        current_score = self.server.game_state.score.get()
        
        # Compute rewards
        rewards = {}
        for agent in self.agents:
            rewards[agent] = self._compute_reward(
                agent, action_valid[agent], prev_score, current_score
            )
        
        # Check termination
        terminated = self._is_done()
        terminations = {a: terminated for a in self.agents}
        truncations = {a: False for a in self.agents}
        
        # Get observations
        observations = {a: self._get_obs(a) for a in self.agents}
        
        # Build infos with trajectories for visualization
        time_delta = new_time - current_time
        infos = {}
        for agent in self.agents:
            traj = trajectories[agent]
            traj_time = 0.0
            if traj is not None:
                try:
                    traj_time = traj.get_travel_time().to(Second)
                except:
                    pass
            
            infos[agent] = {
                "action_valid": action_valid[agent],
                "action_name": action_names[agent],
                "is_free": self._is_agent_free(agent, new_time),
                "trajectory": traj,
                "trajectory_time": traj_time,
                "action_time": action_times[agent],
                "step_time_delta": time_delta,
            }
        
        return observations, rewards, terminations, truncations, infos
    
    def _is_agent_free(self, agent: str, current_time: float) -> bool:
        """Check if an agent is free to take a new action."""
        return self._busy_until.get(agent, 0) <= current_time
    
    def _generate_trajectory(self, agent: str, robot_state, nav_point) -> Any:
        """Generate a trajectory from current position to nav_point."""
        try:
            from gamegine.utils.NCIM.Dimensions.spatial import Meter
            from gamegine.utils.NCIM.Dimensions.angular import Radian
            
            start_x = robot_state.x.get() if hasattr(robot_state.x, 'get') else robot_state.x
            start_y = robot_state.y.get() if hasattr(robot_state.y, 'get') else robot_state.y
            start_theta = robot_state.heading.get() if hasattr(robot_state.heading, 'get') else robot_state.heading
            
            # Use the physics engine's trajectory generator
            traversal_space = self.server.get_traversal_space(agent)
            if traversal_space is None:
                return None
            
            # Get robot for trajectory generation
            robot = self.server.robots.get(agent)
            if robot is None:
                return None
            
            # Find path
            from gamegine.analysis import pathfinding
            path = pathfinding.find_path(
                traversal_space.traversal_map,
                (start_x, start_y),
                (nav_point[0], nav_point[1]),
            )
            
            if path is None:
                return None
            
            # Generate trajectory
            trajectory = self.server.physics_engine.generate_trajectory(
                agent,
                robot,
                (start_x, start_y, start_theta),
                nav_point,
                path,
                traversal_space,
            )
            
            return trajectory
            
        except Exception as e:
            return None
    
    def _estimate_travel_time(self, agent: str, nav_point) -> float:
        """Estimate travel time without generating full trajectory."""
        robot_state = self._get_robot_state(agent)
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        
        start_x = robot_state.x.get() if hasattr(robot_state.x, 'get') else robot_state.x
        start_y = robot_state.y.get() if hasattr(robot_state.y, 'get') else robot_state.y
        
        dx = float(nav_point[0].to(Meter)) - float(start_x.to(Meter))
        dy = float(nav_point[1].to(Meter)) - float(start_y.to(Meter))
        distance = (dx**2 + dy**2)**0.5
        
        # Assume ~3 m/s average speed
        return distance / 3.0
    
    def _get_interaction_time(self, agent: str, interactable: str, interaction: str) -> float:
        """Get interaction time from robot config."""
        robot = self.server.robots.get(agent)
        if robot is None:
            return 0.5
        
        if interactable in robot.interaction_configs:
            if interaction in robot.interaction_configs[interactable]:
                config = robot.interaction_configs[interactable][interaction]
                # Get time from config (it's a callable)
                try:
                    robot_state = self._get_robot_state(agent)
                    time_val = config.time_to_interact(None, robot_state, self.server.game_state)
                    return float(time_val) if hasattr(time_val, '__float__') else 0.5
                except:
                    return 0.5
        return 0.5
    
    def _set_robot_position(self, agent: str, nav_point):
        """Set robot position to destination (discrete teleport)."""
        robot_state = self.server.game_state.get("robots").get(agent)
        if robot_state is not None:
            robot_state.x.set(nav_point[0])
            robot_state.y.set(nav_point[1])
            robot_state.heading.set(nav_point[2])
    
    def action_mask(self, agent: str) -> np.ndarray:
        """Return action mask for an agent.
        
        Masks:
        - All actions except WAIT if agent is busy
        - Occupied targets (optional, for conflict avoidance)
        """
        current_time = self.server.game_state.current_time.get()
        num_actions = len(self._action_maps[agent])
        mask = np.ones(num_actions, dtype=np.int8)
        
        if not self._is_agent_free(agent, current_time):
            # Agent is busy - only WAIT is allowed
            mask[:] = 0
            mask[0] = 1  # WAIT is action 0
        else:
            # Agent is free - allow all actions but mask WAIT
            mask[0] = 0
            
            # TODO: Mask targets already claimed by pending actions
        
        return mask
    
    def get_free_agents(self) -> List[str]:
        """Get list of agents that are free to act."""
        current_time = self.server.game_state.current_time.get()
        return [a for a in self.agents if self._is_agent_free(a, current_time)]
    
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

