"""
Configuration classes for the Gamegine RL environment factory.
"""
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Literal, Optional, Tuple, Any

from gamegine.representation.robot import SwerveRobot
from gamegine.simulation.robot import RobotState
from gamegine.simulation.game import GameState


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
class EnvConfig:
    """Configuration for the RL environment.
    
    :param mode: Environment stepping mode ("discrete" or "continuous").
    :param multi_agent: If True, creates a PettingZoo ParallelEnv; otherwise gym.Env.
    :param robots: List of RobotConfig for each agent in the environment.
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
    
    def __post_init__(self):
        if not self.robots:
            raise ValueError("At least one RobotConfig must be provided in 'robots'")
        
        # Validate unique names
        names = [r.name for r in self.robots]
        if len(names) != len(set(names)):
            raise ValueError("All robot names must be unique")
