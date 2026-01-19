"""
Robot Capabilities Configuration for Versatile Strategy Networks.

Defines data structures for representing heterogeneous robot abilities,
enabling policy adaptation to different configurations.
"""
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, Tuple, Any

from gamegine.representation.gamepiece import Gamepiece
from gamegine.utils.NCIM.ncim import (
    Velocity, MetersPerSecond,
    Omega, RadiansPerSecond,
    TemporalMeasurement, Second,
    SpatialMeasurement, Meter,
)
from gamegine.utils.NCIM.ComplexDimensions.acceleration import Acceleration, MeterPerSecondSquared


# Default accuracy falloff: no degradation
def _no_falloff(distance: SpatialMeasurement) -> float:
    return 1.0


# Default duration modifier: no change
def _no_duration_mod(distance: SpatialMeasurement) -> float:
    return 1.0


@dataclass
class InteractionProfile:
    """
    Performance profile for a specific robot interaction/action.
    
    Models action-specific speed and accuracy, including distance-dependent curves.
    
    :param base_duration: Time to complete action at optimal range.
    :param accuracy: Base accuracy [0.0, 1.0] at optimal range.
    :param optimal_range: Best distance for this action.
    :param accuracy_falloff: Function returning accuracy multiplier given distance.
    :param duration_modifier: Function returning duration multiplier given distance.
    """
    base_duration: TemporalMeasurement = field(default_factory=lambda: Second(0.5))
    accuracy: float = 1.0
    optimal_range: SpatialMeasurement = field(default_factory=lambda: Meter(0.0))
    accuracy_falloff: Callable[[SpatialMeasurement], float] = field(default=_no_falloff)
    duration_modifier: Callable[[SpatialMeasurement], float] = field(default=_no_duration_mod)
    
    def get_effective_accuracy(self, distance: SpatialMeasurement) -> float:
        """Calculate accuracy at a given distance."""
        return self.accuracy * self.accuracy_falloff(distance)
    
    def get_effective_duration(self, distance: SpatialMeasurement) -> TemporalMeasurement:
        """Calculate action duration at a given distance."""
        modifier = self.duration_modifier(distance)
        return Second(float(self.base_duration) * modifier)


@dataclass
class RobotCapabilities:
    """
    Comprehensive capability profile for a robot.
    
    All values use Gamegine measurement types for unit safety.
    Values are normalized to [0, 1] at observation-encoding time.
    
    :param max_speed: Maximum translational velocity.
    :param max_acceleration: Maximum translational acceleration.
    :param rotational_speed: Maximum angular velocity.
    :param gamepiece_capacity: Max count per gamepiece type.
    :param interaction_profiles: Performance profiles keyed by interaction name.
    """
    # Locomotion
    max_speed: Velocity = field(default_factory=lambda: MetersPerSecond(4.0))
    max_acceleration: Acceleration = field(default_factory=lambda: MeterPerSecondSquared(3.0))
    rotational_speed: Omega = field(default_factory=lambda: RadiansPerSecond(6.28))  # ~1 rev/s
    
    # Inventory
    gamepiece_capacity: Dict[Gamepiece, int] = field(default_factory=dict)
    
    # Interactions (keyed by interaction name, e.g., "l4_A", "PickupCoral")
    interaction_profiles: Dict[str, InteractionProfile] = field(default_factory=dict)
    
    def get_interaction_profile(self, interaction_name: str) -> InteractionProfile:
        """Get profile for an interaction, or default if not specified."""
        return self.interaction_profiles.get(interaction_name, InteractionProfile())
    
    def to_vector(
        self,
        speed_limit: float = 10.0,
        accel_limit: float = 10.0,
        omega_limit: float = 12.56,
        max_capacity: int = 5,
        interaction_slots: int = 10,
    ) -> list:
        """
        Encode capabilities as a normalized float vector for observation space.
        
        :param speed_limit: Normalization constant for speed.
        :param accel_limit: Normalization constant for acceleration.
        :param omega_limit: Normalization constant for angular velocity.
        :param max_capacity: Normalization constant for gamepiece capacity.
        :param interaction_slots: Number of fixed slots for interaction profiles.
        :returns: List of floats, all in approximately [0, 1] range.
        """
        vec = []
        
        # Locomotion (3 values)
        vec.append(float(self.max_speed) / speed_limit)
        vec.append(float(self.max_acceleration) / accel_limit)
        vec.append(float(self.rotational_speed) / omega_limit)
        
        # Capacity (sum of all capacities, normalized)
        total_capacity = sum(self.gamepiece_capacity.values()) if self.gamepiece_capacity else 0
        vec.append(total_capacity / max_capacity)
        
        # Interaction profiles (fixed slots: base_duration, accuracy)
        # Pad with zeros if fewer profiles exist
        profiles = list(self.interaction_profiles.values())[:interaction_slots]
        for i in range(interaction_slots):
            if i < len(profiles):
                p = profiles[i]
                vec.append(float(p.base_duration) / 5.0)  # Normalize to ~1s reference
                vec.append(p.accuracy)
            else:
                vec.append(0.0)
                vec.append(0.0)
        
        return vec
    
    @staticmethod
    def vector_size(interaction_slots: int = 10) -> int:
        """Return the size of the capability vector."""
        return 4 + interaction_slots * 2  # 3 loco + 1 cap + 2*slots


# Normalization constants (reasonable defaults for FRC-style games)
CAPABILITY_NORMALIZATION = {
    "speed_limit": 10.0,       # m/s
    "accel_limit": 10.0,       # m/s²
    "omega_limit": 12.56,      # rad/s (~2 rev/s)
    "max_capacity": 5,         # pieces
    "interaction_slots": 10,   # fixed action slots
}
