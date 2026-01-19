"""REBUILT 2026 Shooting Locations Module

Defines fixed shooting positions on the field where robots can shoot from
with varying accuracy based on distance to the Hub.

Shooting mechanics:
- Robot drives to a ShootingLocation
- Executes 'shoot' interaction with accuracy check
- On SCORE: ball goes to Hub, +1 to Neutral Zone (recycle)
- On MISS: ball distributed to Neutral Zone (60%) or Alliance Zone (40%)
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import random

from gamegine.simulation.state import (
    StateSpace,
    ValueDecrease,
    ValueIncrease,
    ValueChange,
)
from gamegine.representation.interactable import (
    RobotInteractable,
    InteractionOption,
)
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.representation.bounds import Point
from gamegine.first.alliance import Alliance
from gamegine.utils.NCIM.ncim import (
    Inch,
    SpatialMeasurement,
    AngularMeasurement,
    Meter,
)

from examples.Rebuilt.scoring import Fuel


# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class ShootingConfig:
    """Configuration for shooting accuracy at a location."""
    base_accuracy: float = 0.85  # Base accuracy (0.0 - 1.0)
    distance_falloff: float = 0.05  # Accuracy penalty per meter beyond optimal
    optimal_distance: float = 2.0  # Distance in meters for base accuracy
    miss_to_neutral_zone_chance: float = 0.6  # 60% of misses go to neutral zone
    shoot_time: float = 1.0  # Base time to shoot (seconds)


@dataclass
class LocationConfig:
    """Configuration for a specific shooting location."""
    position: str  # "NEAR", "MID", or "FAR"
    field_half: str  # "TOP" or "BOT"
    distance_to_hub: float  # meters
    base_accuracy: float  # location-specific accuracy


# Default location configurations
DEFAULT_LOCATIONS = {
    "NEAR_TOP": LocationConfig("NEAR", "TOP", 2.0, 0.95),
    "NEAR_BOT": LocationConfig("NEAR", "BOT", 2.0, 0.95),
    "MID_TOP": LocationConfig("MID", "TOP", 4.0, 0.80),
    "MID_BOT": LocationConfig("MID", "BOT", 4.0, 0.80),
    "FAR_TOP": LocationConfig("FAR", "TOP", 6.0, 0.60),
    "FAR_BOT": LocationConfig("FAR", "BOT", 6.0, 0.60),
}


# =============================================================================
# SHOOTING LOCATION STATE
# =============================================================================

class ShootingLocationState(StateSpace):
    """State for a shooting location.
    
    Shooting locations are stateless (just navigation targets with accuracy config).
    The state is used for tracking shots taken for analytics.
    """
    
    def __init__(self):
        super().__init__()
        self.setValue("shots_taken", 0)
        self.setValue("shots_made", 0)
    
    @property
    def accuracy(self) -> float:
        """Actual accuracy based on shots."""
        taken = self.getValue("shots_taken").get()
        made = self.getValue("shots_made").get()
        return made / taken if taken > 0 else 0.0


# =============================================================================
# SHOOTING LOCATION INTERACTABLE
# =============================================================================

def _has_fuel_condition(interactableState: ShootingLocationState, robotState: RobotState, gameState: GameState) -> bool:
    """Condition: Robot has at least one FUEL to shoot."""
    inventory = robotState.gamepieces.get()
    return inventory.get(Fuel, 0) > 0 if inventory else False


def _create_shoot_action(alliance: Alliance, location_config: LocationConfig, shooting_config: ShootingConfig = None):
    """Create a shoot action for a specific location and alliance."""
    if shooting_config is None:
        shooting_config = ShootingConfig(base_accuracy=location_config.base_accuracy)
    
    def shoot_action(
        interactableState: ShootingLocationState,
        robotState: RobotState,
        gameState: GameState,
    ) -> List[ValueChange]:
        changes = []
        
        # Remove FUEL from robot inventory
        inventory = robotState.gamepieces.get().copy()
        if inventory.get(Fuel, 0) <= 0:
            return changes
        inventory[Fuel] -= 1
        changes.append(ValueChange(robotState.gamepieces, inventory))
        
        # Track shot taken
        changes.append(ValueIncrease(interactableState.getValue("shots_taken"), 1))
        
        # Calculate accuracy (could incorporate robot-specific accuracy later)
        accuracy = location_config.base_accuracy
        
        # Roll for shot success
        is_score = random.random() < accuracy
        
        if is_score:
            # Track successful shot
            changes.append(ValueIncrease(interactableState.getValue("shots_made"), 1))
            
            # Score in Hub
            try:
                hub_name = f"{alliance.name.title()} Hub"
                hub_state = gameState.get("interactables").get(hub_name)
                if hub_state is not None:
                    # Check if Hub is active
                    is_active = hub_state.getValue("is_active").get()
                    if is_active:
                        changes.extend([
                            ValueIncrease(hub_state.getValue("fuel_scored_active"), 1),
                            ValueIncrease(gameState.score, 1),
                            ValueIncrease(gameState.get_alliance_score(alliance), 1),
                        ])
                    else:
                        changes.append(ValueIncrease(hub_state.getValue("fuel_scored_inactive"), 1))
            except (KeyError, AttributeError):
                pass
            
            # Recycle ball to Neutral Zone
            try:
                neutral_zone_state = gameState.get("interactables").get("Neutral Zone")
                if neutral_zone_state is not None:
                    changes.append(ValueIncrease(neutral_zone_state.getValue("fuel_available"), 1))
            except (KeyError, AttributeError):
                pass
        else:
            # Miss - distribute ball to Neutral Zone or Alliance Zone
            goes_to_neutral = random.random() < shooting_config.miss_to_neutral_zone_chance
            
            if goes_to_neutral:
                try:
                    neutral_zone_state = gameState.get("interactables").get("Neutral Zone")
                    if neutral_zone_state is not None:
                        changes.append(ValueIncrease(neutral_zone_state.getValue("fuel_available"), 1))
                except (KeyError, AttributeError):
                    pass
            else:
                # Goes to alliance zone (not necessarily own alliance - could be either)
                # Typically goes to the alliance on the side where the shot was taken
                try:
                    az_name = f"{alliance.name.title()} Alliance Zone"
                    az_state = gameState.get("interactables").get(az_name)
                    if az_state is not None:
                        changes.append(ValueIncrease(az_state.getValue("fuel_available"), 1))
                except (KeyError, AttributeError):
                    pass
        
        return changes
    
    return shoot_action


def _create_shuttle_action(alliance: Alliance):
    """Create a shuttle action to pass balls to own Alliance Zone."""
    def shuttle_action(
        interactableState: ShootingLocationState,
        robotState: RobotState,
        gameState: GameState,
    ) -> List[ValueChange]:
        changes = []
        
        # Get all FUEL from robot inventory
        inventory = robotState.gamepieces.get().copy()
        fuel_count = inventory.get(Fuel, 0)
        
        if fuel_count <= 0:
            return changes
        
        # Remove all FUEL from robot
        inventory[Fuel] = 0
        changes.append(ValueChange(robotState.gamepieces, inventory))
        
        # Add to Alliance Zone
        try:
            az_name = f"{alliance.name.title()} Alliance Zone"
            az_state = gameState.get("interactables").get(az_name)
            if az_state is not None:
                changes.append(ValueIncrease(az_state.getValue("fuel_available"), fuel_count))
        except (KeyError, AttributeError):
            pass
        
        return changes
    
    return shuttle_action


class ShootingLocation(RobotInteractable):
    """Fixed shooting position on the field.
    
    Robots drive to this location to shoot at the Hub.
    Accuracy depends on distance and robot configuration.
    Can also shuttle balls to Alliance Zone from here.
    """
    
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        alliance: Alliance,
        location_key: str,  # e.g., "NEAR_TOP", "MID_BOT", "FAR_TOP"
        name: str = "",
        location_config: LocationConfig = None,
        shooting_config: ShootingConfig = None,
    ):
        if location_config is None:
            location_config = DEFAULT_LOCATIONS.get(location_key, DEFAULT_LOCATIONS["MID_TOP"])
        
        default_name = f"{alliance.name} {location_key.replace('_', ' ').title()}"
        super().__init__(Point(*center, Inch(0)), name or default_name, navigation_point)
        self.alliance = alliance
        self.location_key = location_key
        self.location_config = location_config
        self.shooting_config = shooting_config or ShootingConfig(base_accuracy=location_config.base_accuracy)
    
    def initializeInteractableState(self) -> ShootingLocationState:
        return ShootingLocationState()
    
    def get_interactions(self) -> List[InteractionOption]:
        return [
            InteractionOption(
                "shoot",
                f"Shoot FUEL from {self.name}",
                _has_fuel_condition,
                _create_shoot_action(self.alliance, self.location_config, self.shooting_config),
            ),
            InteractionOption(
                "shuttle",
                f"Shuttle FUEL to {self.alliance.name} Alliance Zone",
                _has_fuel_condition,
                _create_shuttle_action(self.alliance),
            ),
        ]


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def create_shooting_locations_for_alliance(
    alliance: Alliance,
    hub_center: Tuple[SpatialMeasurement, SpatialMeasurement],
    field_width: SpatialMeasurement,
) -> List[ShootingLocation]:
    """Create all 6 shooting locations for an alliance.
    
    Positions are calculated relative to the Hub center.
    """
    locations = []
    
    # Import measurements  
    from gamegine.utils.NCIM.Dimensions.spatial import Feet
    from gamegine.utils.NCIM.Dimensions.angular import Degree
    
    # Calculate positions based on alliance side
    # Blue alliance is on the left (x=0), Red on the right (x=max)
    is_blue = alliance == Alliance.BLUE
    direction = 1 if is_blue else -1  # Direction away from Hub
    
    # Offset from center of field for top/bottom positions
    top_offset = field_width * 0.25
    bot_offset = field_width * -0.25
    
    hub_x = hub_center[0]
    hub_y = hub_center[1]
    
    # Heading faces Hub
    facing_hub = Degree(0) if is_blue else Degree(180)
    near = Meter(1)
    mid = Meter(2)
    far = Meter(3)

    # Location Spects (x_offset, y_offset, distance, accuracy)
    location_specs = [
        ("NEAR_TOP", near * direction, top_offset, 2.0, 0.95),
        ("NEAR_BOT", near * direction, bot_offset, 2.0, 0.95),
        ("MID_TOP", mid * direction, top_offset, 4.0, 0.80),
        ("MID_BOT", mid * direction, bot_offset, 4.0, 0.80),
        ("FAR_TOP", far * direction, top_offset, 6.0, 0.60),
        ("FAR_BOT", far * direction, bot_offset, 6.0, 0.60),
    ]
    
    for key, x_offset, y_offset, distance, accuracy in location_specs:
        center = (hub_x + x_offset, hub_y + y_offset)
        nav_point = (hub_x + x_offset, hub_y + y_offset, facing_hub)
        
        config = LocationConfig(
            position=key.split("_")[0],
            field_half=key.split("_")[1],
            distance_to_hub=distance,
            base_accuracy=accuracy,
        )
        
        locations.append(ShootingLocation(
            center=center,
            navigation_point=nav_point,
            alliance=alliance,
            location_key=key,
            location_config=config,
        ))
    
    return locations
