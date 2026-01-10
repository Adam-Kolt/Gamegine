"""
Gamegine Reinforcement Learning Module.

Provides a factory for automatically generating RL environments from
Gamegine Game and GameServer definitions.

Usage:
    from gamegine.rl import make_env, EnvConfig, RobotConfig
    
    config = EnvConfig(
        mode="discrete",           # or "continuous"
        multi_agent=True,          # or False for single-agent
        robots=[
            RobotConfig(robot, start_state, name="Agent1", team="blue"),
        ],
    )
    
    env = make_env(MyGame, DiscreteGameServer, config)
    
    # For Gymnasium (single-agent)
    obs, info = env.reset()
    obs, reward, terminated, truncated, info = env.step(action)
    
    # For PettingZoo (multi-agent)
    observations, infos = env.reset()
    observations, rewards, terminations, truncations, infos = env.step(actions)

Alliance-Based Training:
    from gamegine.rl import make_self_play_env, make_alliance_env
    
    # Self-play with 3v3 alliances
    env = make_self_play_env(game, red_robots, blue_robots)
    
    # Solo training (one alliance, no opponent)
    env = make_solo_env(game, robots, alliance="blue")
"""
from gamegine.rl.config import (
    EnvConfig, 
    RobotConfig, 
    RewardFn,
    AllianceConfig,
    TrainingConfig,
)
from gamegine.rl.factory import (
    make_env, 
    make_single_agent_env, 
    make_multi_agent_env,
    make_alliance_env,
    make_self_play_env,
    make_solo_env,
    get_rllib_config,
)
from gamegine.rl.envs import (
    BaseGamegineEnv,
    SingleAgentDiscreteEnv,
    MultiAgentDiscreteEnv,
    SingleAgentContinuousEnv,
    MultiAgentContinuousEnv,
)

# Import alliance env separately to avoid circular import issues
try:
    from gamegine.rl.envs.alliance_env import AllianceEnv
except ImportError:
    AllianceEnv = None

__all__ = [
    # Factory functions
    "make_env",
    "make_single_agent_env",
    "make_multi_agent_env",
    "make_alliance_env",
    "make_self_play_env",
    "make_solo_env",
    "get_rllib_config",
    # Config classes
    "EnvConfig",
    "RobotConfig",
    "AllianceConfig",
    "TrainingConfig",
    "RewardFn",
    # Environments
    "BaseGamegineEnv",
    "SingleAgentDiscreteEnv",
    "MultiAgentDiscreteEnv",
    "SingleAgentContinuousEnv",
    "MultiAgentContinuousEnv",
    "AllianceEnv",
]

