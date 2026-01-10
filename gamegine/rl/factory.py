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

Alliance-based training:
    from gamegine.rl import make_alliance_env, make_self_play_env
    
    # Quick self-play setup
    env = make_self_play_env(game, red_robots, blue_robots)
"""
from typing import Type, Union, List, Optional

import gymnasium as gym

from gamegine.representation.game import Game
from gamegine.simulation.GameServer import AbstractGameServer, DiscreteGameServer, ContinuousGameServer
from gamegine.rl.config import EnvConfig, RobotConfig, AllianceConfig, TrainingConfig
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
    - Has alliances configured → AllianceEnv (for competitive training)
    
    :param game: The Game definition containing field, obstacles, interactables.
    :param server_class: The GameServer class to use (defaults based on mode).
    :param config: Environment configuration (mode, robots, rewards, etc.).
    :returns: An initialized RL environment ready for training.
    """
    if config is None:
        raise ValueError("EnvConfig must be provided")
    
    # Auto-select server class based on mode if not specified
    if server_class is None:
        if config.mode == "discrete":
            server_class = DiscreteGameServer
        else:
            server_class = ContinuousGameServer
    
    # Check if alliance-based training is configured
    if config.red_alliance or config.blue_alliance:
        from gamegine.rl.envs.alliance_env import AllianceEnv
        return AllianceEnv(game, server_class, config)
    
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


def make_alliance_env(
    game: Game,
    red_robots: List[RobotConfig],
    blue_robots: List[RobotConfig] = None,
    mode: str = "solo",
    share_reward: bool = True,
    competitive_reward: bool = True,
    server_class: Type[AbstractGameServer] = None,
    fast_mode: bool = False,
    use_server_pool: bool = False,
):
    """Create an alliance-based training environment.
    
    Quick setup for competitive or solo alliance training.
    
    :param game: The Game definition.
    :param red_robots: List of RobotConfig for red alliance.
    :param blue_robots: List of RobotConfig for blue alliance (optional for solo).
    :param mode: Training mode ("solo", "self_play", "vs_opponent").
    :param share_reward: If True, teammates share the same reward.
    :param competitive_reward: If True, reward = Δ(our - opponent) instead of Δ(our).
    :param server_class: GameServer class to use.
    :param fast_mode: If True, use teleport instead of trajectory generation.
    :param use_server_pool: If True, reuse servers instead of recreating each reset.
    :returns: AllianceEnv ready for training.
    
    Example:
        >>> env = make_alliance_env(
        ...     game=Reefscape,
        ...     red_robots=[red1, red2, red3],
        ...     blue_robots=[blue1, blue2, blue3],
        ...     mode="self_play",
        ... )
    """
    from gamegine.rl.envs.alliance_env import AllianceEnv
    
    # Ensure team affiliations are set
    for r in red_robots:
        r.team = "red"
    if blue_robots:
        for r in blue_robots:
            r.team = "blue"
    
    training_config = TrainingConfig(
        mode=mode,
        share_reward=share_reward,
        competitive_reward=competitive_reward,
    )
    
    config = EnvConfig(
        mode="discrete",
        red_alliance=AllianceConfig(robots=red_robots, policy_id="red_policy"),
        blue_alliance=AllianceConfig(robots=blue_robots or [], policy_id="blue_policy") if blue_robots else None,
        training=training_config,
        fast_mode=fast_mode,
        use_server_pool=use_server_pool,
    )
    
    if server_class is None:
        server_class = DiscreteGameServer
    
    return AllianceEnv(game, server_class, config)


def make_self_play_env(
    game: Game,
    red_robots: List[RobotConfig],
    blue_robots: List[RobotConfig],
    share_reward: bool = True,
    server_class: Type[AbstractGameServer] = None,
    fast_mode: bool = False,
    use_server_pool: bool = False,
):
    """Create a self-play environment where both alliances compete.
    
    Convenience function for symmetric self-play training.
    
    :param game: The Game definition.
    :param red_robots: Robots for red alliance.
    :param blue_robots: Robots for blue alliance.
    :param share_reward: If True, teammates share rewards.
    :param server_class: GameServer class to use.
    :param fast_mode: If True, use teleport instead of trajectory generation.
    :param use_server_pool: If True, reuse servers instead of recreating each reset.
    :returns: AllianceEnv configured for self-play.
    """
    return make_alliance_env(
        game=game,
        red_robots=red_robots,
        blue_robots=blue_robots,
        mode="self_play",
        share_reward=share_reward,
        competitive_reward=True,
        server_class=server_class,
        fast_mode=fast_mode,
        use_server_pool=use_server_pool,
    )


def make_solo_env(
    game: Game,
    robots: List[RobotConfig],
    alliance: str = "blue",
    server_class: Type[AbstractGameServer] = None,
    fast_mode: bool = False,
    use_server_pool: bool = False,
):
    """Create a solo training environment (no opponent).
    
    One alliance trains to maximize its score without competition.
    
    :param game: The Game definition.
    :param robots: Robots for the training alliance.
    :param alliance: Which alliance to train ("red" or "blue").
    :param server_class: GameServer class to use.
    :param fast_mode: If True, use teleport instead of trajectory generation.
    :param use_server_pool: If True, reuse servers instead of recreating each reset.
    :returns: AllianceEnv configured for solo training.
    """
    for r in robots:
        r.team = alliance
    
    if alliance.lower() == "red":
        return make_alliance_env(
            game=game,
            red_robots=robots,
            blue_robots=None,
            mode="solo",
            competitive_reward=False,
            server_class=server_class,
            fast_mode=fast_mode,
            use_server_pool=use_server_pool,
        )
    else:
        return make_alliance_env(
            game=game,
            red_robots=[],
            blue_robots=robots,
            mode="solo",
            competitive_reward=False,
            server_class=server_class,
            fast_mode=fast_mode,
            use_server_pool=use_server_pool,
        )


def get_rllib_config(
    env_class: type,
    env_config: dict,
    algorithm: str = "PPO",
    **kwargs,
):
    """Get an RLlib Algorithm config for training.
    
    :param env_class: Environment class (e.g., AllianceEnv).
    :param env_config: Configuration dict for the environment.
    :param algorithm: RL algorithm to use ("PPO", "APPO", etc.).
    :param kwargs: Additional algorithm configuration.
    :returns: RLlib AlgorithmConfig object.
    
    Example:
        >>> from gamegine.rl import get_rllib_config, AllianceEnv
        >>> config = get_rllib_config(
        ...     env_class=AllianceEnv,
        ...     env_config={"game": my_game, "config": env_cfg},
        ...     algorithm="PPO",
        ... )
        >>> algo = config.build()
        >>> algo.train()
    """
    from gamegine.rl.rllib_utils import create_rllib_config
    return create_rllib_config(
        algorithm=algorithm,
        env_class=env_class,
        env_config=env_config,
        **kwargs,
    )

