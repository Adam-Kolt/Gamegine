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
"""
from gamegine.rl.config import EnvConfig, RobotConfig, RewardFn
from gamegine.rl.factory import make_env, make_single_agent_env, make_multi_agent_env
from gamegine.rl.envs import (
    BaseGamegineEnv,
    SingleAgentDiscreteEnv,
    MultiAgentDiscreteEnv,
    SingleAgentContinuousEnv,
    MultiAgentContinuousEnv,
)

__all__ = [
    # Factory
    "make_env",
    "make_single_agent_env",
    "make_multi_agent_env",
    # Config
    "EnvConfig",
    "RobotConfig",
    "RewardFn",
    # Environments
    "BaseGamegineEnv",
    "SingleAgentDiscreteEnv",
    "MultiAgentDiscreteEnv",
    "SingleAgentContinuousEnv",
    "MultiAgentContinuousEnv",
]
