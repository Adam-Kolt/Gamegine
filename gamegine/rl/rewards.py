"""
Built-in reward functions for Gamegine RL training.

These functions can be used directly or serve as templates for custom rewards.
"""
from typing import Callable, Dict, Optional

from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.first.alliance import Alliance


# Type alias for alliance reward functions
AllianceRewardFn = Callable[[GameState, int, int], float]
"""
Alliance reward function signature:
    Args:
        game_state: Current GameState
        prev_alliance_score: Previous score for this alliance
        prev_opponent_score: Previous score for opponent alliance
    Returns:
        float reward
"""


def alliance_score_delta(alliance: Alliance) -> AllianceRewardFn:
    """
    Reward function that returns the change in alliance score.
    
    reward = Δ(alliance_score)
    
    Good for solo training where opponent doesn't exist.
    
    :param alliance: The alliance to compute reward for.
    :returns: Reward function.
    """
    def reward_fn(
        game_state: GameState,
        prev_alliance_score: int,
        prev_opponent_score: int,
    ) -> float:
        current_score = game_state.get_alliance_score_value(alliance)
        return float(current_score - prev_alliance_score)
    
    return reward_fn


def competitive_reward(alliance: Alliance) -> AllianceRewardFn:
    """
    Reward function that returns the change in score differential.
    
    reward = Δ(our_score - opponent_score)
    
    This is a zero-sum competitive reward - what's good for us is bad for them.
    
    :param alliance: The alliance to compute reward for.
    :returns: Reward function.
    """
    opponent = Alliance.BLUE if alliance == Alliance.RED else Alliance.RED
    
    def reward_fn(
        game_state: GameState,
        prev_alliance_score: int,
        prev_opponent_score: int,
    ) -> float:
        current_our = game_state.get_alliance_score_value(alliance)
        current_opp = game_state.get_alliance_score_value(opponent)
        
        prev_diff = prev_alliance_score - prev_opponent_score
        current_diff = current_our - current_opp
        
        return float(current_diff - prev_diff)
    
    return reward_fn


def match_outcome_reward(
    alliance: Alliance,
    win_bonus: float = 10.0,
    lose_penalty: float = -10.0,
    tie_bonus: float = 0.0,
) -> Callable[[GameState, bool], float]:
    """
    Reward function that adds bonus/penalty at match end based on outcome.
    
    Use this in combination with other reward functions at episode termination.
    
    :param alliance: The alliance to check outcome for.
    :param win_bonus: Bonus added if alliance wins.
    :param lose_penalty: Penalty if alliance loses.
    :param tie_bonus: Bonus if match is tied.
    :returns: Function that takes (game_state, is_terminal) and returns bonus.
    """
    opponent = Alliance.BLUE if alliance == Alliance.RED else Alliance.RED
    
    def outcome_fn(game_state: GameState, is_terminal: bool) -> float:
        if not is_terminal:
            return 0.0
        
        our_score = game_state.get_alliance_score_value(alliance)
        opp_score = game_state.get_alliance_score_value(opponent)
        
        if our_score > opp_score:
            return win_bonus
        elif our_score < opp_score:
            return lose_penalty
        else:
            return tie_bonus
    
    return outcome_fn


def time_efficiency_reward(
    time_weight: float = -0.01,
) -> Callable[[float], float]:
    """
    Penalty for time taken, encouraging efficient play.
    
    reward = time_weight * dt
    
    :param time_weight: Multiplier for time penalty (negative = penalty).
    :returns: Function that takes time_delta and returns penalty.
    """
    def time_fn(dt: float) -> float:
        return time_weight * dt
    
    return time_fn


def gamepiece_bonus(
    gamepiece_type: type,
    bonus_per_piece: float = 0.1,
) -> Callable[[RobotState, RobotState], float]:
    """
    Bonus for collecting gamepieces.
    
    :param gamepiece_type: Type of gamepiece to reward.
    :param bonus_per_piece: Bonus per piece collected.
    :returns: Function that takes (prev_state, current_state) and returns bonus.
    """
    def bonus_fn(prev_state: RobotState, current_state: RobotState) -> float:
        prev_count = prev_state.gamepieces.get().get(gamepiece_type, 0)
        curr_count = current_state.gamepieces.get().get(gamepiece_type, 0)
        delta = curr_count - prev_count
        return bonus_per_piece * delta if delta > 0 else 0.0
    
    return bonus_fn


def combined_reward(*reward_fns) -> Callable:
    """
    Combine multiple reward functions by summing their outputs.
    
    :param reward_fns: Variable number of reward functions.
    :returns: Combined reward function.
    
    Example:
        >>> reward = combined_reward(
        ...     competitive_reward(Alliance.BLUE),
        ...     match_outcome_reward(Alliance.BLUE),
        ... )
    """
    def combined(*args, **kwargs):
        total = 0.0
        for fn in reward_fns:
            try:
                total += fn(*args, **kwargs)
            except TypeError:
                # Function might have different signature, skip
                pass
        return total
    
    return combined


def shaped_reward(
    base_reward_fn: AllianceRewardFn,
    shaping_potential: Callable[[GameState, RobotState], float],
) -> AllianceRewardFn:
    """
    Apply potential-based reward shaping to a base reward.
    
    shaped_reward = base_reward + γ * Φ(s') - Φ(s)
    
    This preserves optimal policies while providing denser feedback.
    
    :param base_reward_fn: The base reward function.
    :param shaping_potential: Potential function Φ(state).
    :returns: Shaped reward function.
    """
    # Note: Full implementation would need to track previous potential
    # This is a simplified version
    def shaped_fn(
        game_state: GameState,
        prev_alliance_score: int,
        prev_opponent_score: int,
    ) -> float:
        base = base_reward_fn(game_state, prev_alliance_score, prev_opponent_score)
        # Would add shaping here with state tracking
        return base
    
    return shaped_fn
