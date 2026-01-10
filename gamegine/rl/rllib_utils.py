"""
RLlib integration utilities for Gamegine.

Provides helper functions for setting up RLlib training with alliance-based environments.
"""
from typing import Dict, List, Optional, Type, Any, Callable
import logging

from gamegine.rl.config import EnvConfig, AllianceConfig, TrainingConfig
from gamegine.first.alliance import Alliance

logger = logging.getLogger(__name__)


def get_policy_mapping_fn(config: EnvConfig) -> Callable:
    """
    Create a policy mapping function for RLlib.
    
    Maps agent IDs to policy IDs for asymmetric training where
    each alliance can have a different policy.
    
    :param config: Environment configuration with alliance settings.
    :returns: Policy mapping function for RLlib config.
    """
    red_policy = "red_policy"
    blue_policy = "blue_policy"
    
    if config.red_alliance:
        red_policy = config.red_alliance.policy_id
    if config.blue_alliance:
        blue_policy = config.blue_alliance.policy_id
    
    def policy_mapping(agent_id: str, episode=None, worker=None, **kwargs) -> str:
        if agent_id.startswith("red"):
            return red_policy
        return blue_policy
    
    return policy_mapping


def get_policies_config(
    config: EnvConfig,
    observation_space,
    action_space,
) -> Dict[str, tuple]:
    """
    Create policies configuration for RLlib MultiAgentConfig.
    
    :param config: Environment configuration.
    :param observation_space: Observation space for agents.
    :param action_space: Action space for agents.
    :returns: Dict mapping policy_id -> (policy_class, obs_space, action_space, config)
    """
    policies = {}
    
    red_policy_id = config.red_alliance.policy_id if config.red_alliance else "red_policy"
    blue_policy_id = config.blue_alliance.policy_id if config.blue_alliance else "blue_policy"
    
    # Use None for policy_class to let RLlib use default
    policies[red_policy_id] = (None, observation_space, action_space, {})
    
    # For self-play, both alliances might share a policy or have separate ones
    if config.training.mode == "self_play":
        if red_policy_id != blue_policy_id:
            policies[blue_policy_id] = (None, observation_space, action_space, {})
        # If same policy_id, they share (symmetric self-play)
    else:
        policies[blue_policy_id] = (None, observation_space, action_space, {})
    
    return policies


def get_multiagent_config(
    env_config: EnvConfig,
    observation_space,
    action_space,
) -> dict:
    """
    Generate RLlib MultiAgentConfig settings for the environment.
    
    :param env_config: Environment configuration.
    :param observation_space: Common observation space.
    :param action_space: Common action space.
    :returns: Dict with multiagent configuration for RLlib.
    """
    return {
        "policies": get_policies_config(env_config, observation_space, action_space),
        "policy_mapping_fn": get_policy_mapping_fn(env_config),
        "policies_to_train": _get_policies_to_train(env_config),
    }


def _get_policies_to_train(config: EnvConfig) -> List[str]:
    """Determine which policies should be trained based on training mode."""
    red_policy = config.red_alliance.policy_id if config.red_alliance else "red_policy"
    blue_policy = config.blue_alliance.policy_id if config.blue_alliance else "blue_policy"
    
    if config.training.mode == "solo":
        # Solo mode: train only blue alliance by default
        # User can configure which alliance to train
        return [blue_policy]
    
    elif config.training.mode == "self_play":
        # Self-play: train both alliances
        if red_policy == blue_policy:
            return [red_policy]  # Same policy, train once
        return [red_policy, blue_policy]
    
    elif config.training.mode == "vs_opponent":
        # Train blue against frozen red
        return [blue_policy]
    
    return [blue_policy]


def create_rllib_config(
    algorithm: str = "PPO",
    env_class: type = None,
    env_config: dict = None,
    multiagent_config: dict = None,
    **kwargs,
) -> Any:
    """
    Create an RLlib Algorithm configuration.
    
    :param algorithm: Algorithm name ("PPO", "APPO", "IMPALA", etc.)
    :param env_class: Environment class to use.
    :param env_config: Environment configuration dict.
    :param multiagent_config: Multi-agent configuration from get_multiagent_config.
    :param kwargs: Additional algorithm-specific configuration.
    :returns: RLlib AlgorithmConfig object.
    
    Example:
        >>> from ray.rllib.algorithms.ppo import PPOConfig
        >>> config = create_rllib_config(
        ...     algorithm="PPO",
        ...     env_class=AllianceEnv,
        ...     env_config={"game": my_game, "config": env_config},
        ...     multiagent_config=multiagent_config,
        ... )
        >>> algo = config.build()
    """
    try:
        from ray.rllib.algorithms.ppo import PPOConfig
        from ray.rllib.algorithms.appo import APPOConfig
        from ray.rllib.algorithms.impala import ImpalaConfig
    except ImportError:
        logger.warning("RLlib not installed. Install with: pip install 'ray[rllib]'")
        raise ImportError("RLlib is required for this function. Install with: pip install 'ray[rllib]'")
    
    # Select algorithm config class
    algo_configs = {
        "PPO": PPOConfig,
        "APPO": APPOConfig,
        "IMPALA": ImpalaConfig,
    }
    
    config_class = algo_configs.get(algorithm.upper())
    if config_class is None:
        raise ValueError(f"Unknown algorithm: {algorithm}. Supported: {list(algo_configs.keys())}")
    
    # Build config
    config = config_class()
    
    if env_class:
        config = config.environment(env=env_class, env_config=env_config or {})
    
    if multiagent_config:
        config = config.multi_agent(**multiagent_config)
    
    # Apply additional kwargs
    for key, value in kwargs.items():
        if hasattr(config, key):
            setattr(config, key, value)
    
    return config


def setup_self_play(
    config,
    sync_interval: int = 100,
):
    """
    Configure self-play training settings.
    
    :param config: RLlib AlgorithmConfig.
    :param sync_interval: How often to sync policies (in training iterations).
    :returns: Updated config.
    """
    # Self-play typically involves:
    # 1. Training both policies
    # 2. Periodically syncing/rotating historical policies
    
    # For now, just ensure both policies are trained
    # More sophisticated self-play (league training) can be added later
    
    return config


def register_gamegine_env(
    env_name: str = "gamegine-alliance-v0",
    env_class: type = None,
):
    """
    Register a Gamegine environment with Ray/RLlib.
    
    :param env_name: Name to register the environment under.
    :param env_class: Environment class to register.
    """
    try:
        from ray.tune.registry import register_env
        
        if env_class is None:
            from gamegine.rl.envs.alliance_env import AllianceEnv
            env_class = AllianceEnv
        
        def env_creator(config):
            return env_class(**config)
        
        register_env(env_name, env_creator)
        logger.info(f"Registered environment: {env_name}")
        
    except ImportError:
        logger.warning("Ray not installed. Cannot register environment.")
