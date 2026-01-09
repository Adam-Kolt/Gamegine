"""
Environment factory for creating Gamegine RL environments.

Usage:
    from gamegine.rl import make_env, EnvConfig, RobotConfig
    
    config = EnvConfig(
        mode="discrete",
        multi_agent=True,
        robots=[
            RobotConfig(robot1, start_state1, name="Blue1", team="blue"),
            RobotConfig(robot2, start_state2, name="Blue2", team="blue"),
        ],
    )
    
    env = make_env(game, DiscreteGameServer, config)
"""
from typing import Type, Union

import gymnasium as gym

from gamegine.representation.game import Game
from gamegine.simulation.GameServer import AbstractGameServer, DiscreteGameServer, ContinuousGameServer
from gamegine.rl.config import EnvConfig
from gamegine.rl.envs.discrete import SingleAgentDiscreteEnv, MultiAgentDiscreteEnv
from gamegine.rl.envs.continuous import SingleAgentContinuousEnv, MultiAgentContinuousEnv


def make_env(
    game: Game,
    server_class: Type[AbstractGameServer] = None,
    config: EnvConfig = None,
) -> Union[gym.Env, "ParallelEnv"]:
    """Create an RL environment from a Gamegine Game and configuration.
    
    This factory function automatically selects the appropriate environment
    implementation based on the configuration:
    
    - mode="discrete", multi_agent=False → SingleAgentDiscreteEnv (gym.Env)
    - mode="discrete", multi_agent=True → MultiAgentDiscreteEnv (PettingZoo ParallelEnv)
    - mode="continuous", multi_agent=False → SingleAgentContinuousEnv (gym.Env)
    - mode="continuous", multi_agent=True → MultiAgentContinuousEnv (PettingZoo ParallelEnv)
    
    :param game: The Game definition containing field, obstacles, interactables.
    :param server_class: The GameServer class to use (defaults based on mode).
    :param config: Environment configuration (mode, robots, rewards, etc.).
    :returns: An initialized RL environment ready for training.
    
    Example:
        >>> from gamegine.rl import make_env, EnvConfig, RobotConfig
        >>> from my_game import MyGame, SWERVE_ROBOT
        >>> 
        >>> config = EnvConfig(
        ...     mode="discrete",
        ...     robots=[RobotConfig(SWERVE_ROBOT, RobotState(), name="Agent")]
        ... )
        >>> env = make_env(MyGame, config=config)
        >>> obs, info = env.reset()
    """
    if config is None:
        raise ValueError("EnvConfig must be provided")
    
    # Auto-select server class based on mode if not specified
    if server_class is None:
        if config.mode == "discrete":
            server_class = DiscreteGameServer
        else:
            server_class = ContinuousGameServer
    
    # Select environment implementation
    if config.multi_agent:
        if config.mode == "discrete":
            return MultiAgentDiscreteEnv(game, server_class, config)
        else:
            return MultiAgentContinuousEnv(game, server_class, config)
    else:
        if config.mode == "discrete":
            return SingleAgentDiscreteEnv(game, server_class, config)
        else:
            return SingleAgentContinuousEnv(game, server_class, config)


def make_single_agent_env(
    game: Game,
    server_class: Type[AbstractGameServer] = None,
    config: EnvConfig = None,
) -> gym.Env:
    """Create a single-agent Gymnasium environment.
    
    Convenience wrapper that ensures multi_agent=False.
    """
    if config is None:
        raise ValueError("EnvConfig must be provided")
    
    config.multi_agent = False
    return make_env(game, server_class, config)


def make_multi_agent_env(
    game: Game,
    server_class: Type[AbstractGameServer] = None,
    config: EnvConfig = None,
):
    """Create a multi-agent PettingZoo environment.
    
    Convenience wrapper that ensures multi_agent=True.
    """
    if config is None:
        raise ValueError("EnvConfig must be provided")
    
    config.multi_agent = True
    return make_env(game, server_class, config)
