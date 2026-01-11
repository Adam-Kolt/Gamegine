"""Shooting accuracy model for simulating shot outcomes based on distance and parameters."""

from dataclasses import dataclass
from enum import Enum
from typing import Tuple
import random
import math

from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter


class ShotOutcome(Enum):
    """Result of a shooting attempt."""
    SUCCESS = "success"
    MISS = "miss"


@dataclass
class ShootingParameters:
    """Parameters that define shooting accuracy from a scoring location.
    
    :param base_accuracy: Probability of success at optimal range (0.0-1.0)
    :param optimal_range: Distance at which accuracy is highest
    :param accuracy_falloff_per_meter: Decrease in accuracy per meter from optimal
    :param max_range: Maximum effective shooting range (0% beyond this)
    """
    base_accuracy: float
    optimal_range: SpatialMeasurement
    accuracy_falloff_per_meter: float
    max_range: SpatialMeasurement


def calculate_shot_probability(
    shooter_pos: Tuple[SpatialMeasurement, SpatialMeasurement],
    target_pos: Tuple[SpatialMeasurement, SpatialMeasurement],
    params: ShootingParameters,
) -> float:
    """Calculate probability of a successful shot.
    
    :param shooter_pos: (x, y) position of the shooter
    :param target_pos: (x, y) position of the target
    :param params: Shooting parameters
    :returns: Probability of success (0.0-1.0)
    """
    # Calculate distance
    dx = shooter_pos[0].to(Meter) - target_pos[0].to(Meter)
    dy = shooter_pos[1].to(Meter) - target_pos[1].to(Meter)
    distance = math.sqrt(dx**2 + dy**2)
    
    max_range_m = params.max_range.to(Meter)
    if distance > max_range_m:
        return 0.0
    
    optimal_m = params.optimal_range.to(Meter)
    distance_from_optimal = abs(distance - optimal_m)
    
    # Apply falloff
    accuracy = params.base_accuracy - (distance_from_optimal * params.accuracy_falloff_per_meter)
    
    # Clamp to [0, 1]
    return max(0.0, min(1.0, accuracy))


def attempt_shot(
    shooter_pos: Tuple[SpatialMeasurement, SpatialMeasurement],
    target_pos: Tuple[SpatialMeasurement, SpatialMeasurement],
    params: ShootingParameters,
    deterministic: bool = False,
) -> ShotOutcome:
    """Perform a shot attempt.
    
    :param shooter_pos: (x, y) position of the shooter
    :param target_pos: (x, y) position of the target
    :param params: Shooting parameters
    :param deterministic: If True, shots with >50% probability always succeed
    :returns: ShotOutcome.SUCCESS or ShotOutcome.MISS
    """
    probability = calculate_shot_probability(shooter_pos, target_pos, params)
    
    if deterministic:
        # For RL evaluation: >50% = success, <=50% = miss
        return ShotOutcome.SUCCESS if probability > 0.5 else ShotOutcome.MISS
    
    # Stochastic: roll random number
    if random.random() < probability:
        return ShotOutcome.SUCCESS
    return ShotOutcome.MISS
