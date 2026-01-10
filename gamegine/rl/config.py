"""
Configuration classes for the Gamegine RL environment factory.
"""
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Literal, Optional, Tuple, Any

from gamegine.representation.robot import SwerveRobot
from gamegine.simulation.robot import RobotState
from gamegine.simulation.game import GameState
from gamegine.first.alliance import Alliance


@dataclass
class RobotConfig:
    """Configuration for a single robot/agent in the environment.
    
    :param robot: The SwerveRobot definition (physical parameters, swerve config).
    :param start_state: Initial RobotState (position, heading, gamepieces).
    :param name: Unique identifier for this agent (e.g., "Blue1", "Red2").
    :param team: Team affiliation for multi-team games ("blue", "red").
    """
    robot: SwerveRobot
    start_state: RobotState
    name: str = "Agent"
    team: str = "blue"


# Type alias for reward functions
RewardFn = Callable[[GameState, Dict[str, RobotState], Dict[str, bool]], Dict[str, float]]
"""
Reward function signature:
    Args:
        game_state: Current GameState
        robot_states: Dict mapping robot_name -> RobotState
        action_valid: Dict mapping robot_name -> bool (was action valid?)
    Returns:
        Dict mapping robot_name -> float reward
"""


@dataclass
class AllianceConfig:
    """Configuration for an alliance in training.
    
    :param robots: List of RobotConfig for robots on this alliance.
    :param policy_id: RLlib policy ID for this alliance (for asymmetric training).
    """
    robots: List[RobotConfig]
    policy_id: str = "default"


@dataclass
class TrainingConfig:
    """Configuration for alliance-based training.
    
    :param mode: Training mode:
        - "solo": One alliance trains to maximize its score (no opponent)
        - "self_play": Both alliances are RL agents competing
        - "vs_opponent": Train against a frozen/scripted opponent policy
    :param share_reward: If True, all robots on an alliance share the same reward.
    :param opponent_policy_id: Policy ID for opponent in vs_opponent mode.
    :param competitive_reward: If True, reward = Δ(our_score - opponent_score).
    """
    mode: Literal["solo", "self_play", "vs_opponent"] = "solo"
    share_reward: bool = True
    opponent_policy_id: Optional[str] = None
    competitive_reward: bool = True  # Use score differential vs absolute score


@dataclass
class EnvConfig:
    """Configuration for the RL environment.
    
    :param mode: Environment stepping mode ("discrete" or "continuous").
    :param multi_agent: If True, creates a PettingZoo ParallelEnv; otherwise gym.Env.
    :param robots: List of RobotConfig for each agent (legacy, prefer alliances).
    :param red_alliance: Configuration for red alliance robots.
    :param blue_alliance: Configuration for blue alliance robots.
    :param training: Training configuration for alliance-based learning.
    :param reward_fn: Custom reward function (optional). Default uses Δscore + penalties.
    :param include_opponent_obs: Include opponent positions/velocities in observations.
    :param include_gamepiece_obs: Include gamepiece counts in observations.
    :param include_velocity_obs: Include robot velocities in observations.
    :param use_event_stepping: (Discrete mode) Advance time to next event horizon.
    :param dt: (Continuous mode) Physics timestep in seconds.
    :param invalid_action_penalty: Penalty for attempting invalid actions.
    :param max_episode_steps: Maximum steps before truncation (None for no limit).
    """
    mode: Literal["discrete", "continuous"] = "discrete"
    multi_agent: bool = False
    robots: List[RobotConfig] = field(default_factory=list)
    
    # Alliance-based configuration
    red_alliance: Optional[AllianceConfig] = None
    blue_alliance: Optional[AllianceConfig] = None
    training: TrainingConfig = field(default_factory=TrainingConfig)
    
    # Reward configuration
    reward_fn: Optional[RewardFn] = None
    invalid_action_penalty: float = -1.0
    
    # Observation configuration
    include_opponent_obs: bool = True
    include_gamepiece_obs: bool = True
    include_velocity_obs: bool = True
    
    # Discrete-specific (SMDP)
    use_event_stepping: bool = True
    
    # Continuous-specific
    dt: float = 0.02  # 50Hz
    
    # Episode limits
    max_episode_steps: Optional[int] = None
    
    # Training optimizations (for faster iteration, sacrifice accuracy)
    fast_mode: bool = False  # Skip trajectory generation, use teleport + time estimate
    cache_trajectories: bool = False  # Pre-compute and cache trajectories
    use_server_pool: bool = False  # Reuse servers instead of recreating
    
    def __post_init__(self):
        # If alliances are provided, populate robots list from them
        if self.red_alliance or self.blue_alliance:
            self.robots = []
            if self.red_alliance:
                self.robots.extend(self.red_alliance.robots)
            if self.blue_alliance:
                self.robots.extend(self.blue_alliance.robots)
            self.multi_agent = True  # Alliance mode implies multi-agent
        
        if not self.robots:
            raise ValueError("At least one RobotConfig must be provided in 'robots' or via alliances")
        
        # Validate unique names
        names = [r.name for r in self.robots]
        if len(names) != len(set(names)):
            raise ValueError("All robot names must be unique")
    
    def get_robots_by_alliance(self, alliance: Alliance) -> List[RobotConfig]:
        """Get all robots belonging to a specific alliance."""
        team_str = "red" if alliance == Alliance.RED else "blue"
        return [r for r in self.robots if r.team.lower() == team_str]

