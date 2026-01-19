"""REBUILT 2026 Scoring Module

Defines scoring interactables for the REBUILT FRC game:
- Hub: Central scoring tower where FUEL is shot
- Tower: Endgame climbing structure

Based on the 2026 FRC game rules.
"""

from typing import List, Tuple
from gamegine.simulation.state import (
    StateSpace,
    ValueDecrease,
    ValueIncrease,
    ValueChange,
)
from gamegine.representation.interactable import (
    RobotInteractable,
    InteractionOption,
    RobotInteractionConfig,
)
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.representation.gamepiece import Gamepiece, GamepiecePhysicalProperties
from gamegine.representation.bounds import Point, Rectangle, Cylinder, Transform3D
from gamegine.first.alliance import Alliance
from gamegine.utils.NCIM.ncim import (
    Inch,
    Degree,
    SpatialMeasurement,
    AngularMeasurement,
    Centimeter,
)
from gamegine.utils.NCIM.Dimensions.mass import Pound
import arcade


# =============================================================================
# FUEL GAMEPIECE
# =============================================================================

class Fuel(Gamepiece):
    """FUEL ball gamepiece for REBUILT.
    
    Orange balls that robots collect and shoot into the Hub.
    Diameter: ~7 inches
    """
    
    name = "Fuel"
    bounds = Cylinder(Inch(3.5), Inch(7), Transform3D())  # radius, height
    physical_properties = GamepiecePhysicalProperties(
        mass=Pound(0.3),
        friction_coefficient=0.8,
    )
    
    @classmethod
    def display(
        cls,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        render_scale: SpatialMeasurement,
    ):
        """Draw an orange circle for FUEL."""
        px = float(x.to(Inch)) * float(render_scale.to(Inch))
        py = float(y.to(Inch)) * float(render_scale.to(Inch))
        radius = 3.5 * float(render_scale.to(Inch))  # 7" diameter / 2
        
        # Orange ball with slight gradient effect
        arcade.draw_circle_filled(px, py, radius, (255, 140, 0, 255))
        arcade.draw_circle_outline(px, py, radius, (200, 100, 0, 255), 2)


# =============================================================================
# HUB STATE & INTERACTABLE
# =============================================================================

class HubState(StateSpace):
    """State for a Hub scoring location.
    
    Tracks:
    - fuel_scored_active: FUEL scored when Hub was active (counts for match points)
    - fuel_scored_inactive: FUEL scored when Hub was inactive (counts for RP only)
    - is_active: Whether the Hub is currently active this period
    """
    
    def __init__(self):
        super().__init__()
        self.setValue("fuel_scored_active", 0)
        self.setValue("fuel_scored_inactive", 0)
        self.setValue("is_active", True)  # Hubs start active in AUTO
    
    @property
    def fuel_scored_active(self):
        return self.getValue("fuel_scored_active")
    
    @property
    def fuel_scored_inactive(self):
        return self.getValue("fuel_scored_inactive")
    
    @property
    def total_fuel_scored(self) -> int:
        """Total FUEL scored (active + inactive), used for RP calculation."""
        return self.fuel_scored_active.get() + self.fuel_scored_inactive.get()
    
    @property
    def is_active(self):
        return self.getValue("is_active")
    
    def set_active(self, active: bool):
        """Set whether this Hub is active."""
        self.setValue("is_active", active)


def has_fuel_condition(interactableState: StateSpace, robotState: RobotState, gameState: GameState) -> bool:
    """Condition: Robot has at least one FUEL."""
    return robotState.gamepieces.get().get(Fuel, 0) > 0


def is_auto(gameState: GameState) -> bool:
    """Check if currently in AUTO period."""
    return gameState.current_time.get() <= gameState.auto_time.get()


class Hub(RobotInteractable):
    """Hub scoring tower for REBUILT.
    
    Robots shoot FUEL into the Hub. Points are only awarded when
    the Hub is active, but all FUEL counts toward ranking points.
    """
    
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        alliance: Alliance,
        name: str = "",
    ):
        super().__init__(Point(*center, Inch(0)), name or f"{alliance.name} Hub", navigation_point)
        self.alliance = alliance
    
    @staticmethod
    def initializeInteractableState() -> HubState:
        return HubState()
    
    @staticmethod
    def __generate_score_fuel_function(alliance: Alliance):
        """Generate the scoring function for a Hub."""
        def score_fuel(
            interactableState: HubState,
            robotState: RobotState,
            gameState: GameState,
        ) -> List[ValueChange]:
            changes = []
            
            # Track whether Hub is active
            is_active = interactableState.is_active.get()
            
            if is_active:
                # Score point and track as active
                changes.extend([
                    ValueIncrease(interactableState.fuel_scored_active, 1),
                    ValueIncrease(gameState.score, 1),
                    ValueIncrease(gameState.get_alliance_score(alliance), 1),
                ])
            else:
                # No points but still track for RP
                changes.append(
                    ValueIncrease(interactableState.fuel_scored_inactive, 1)
                )
            
            # Decrement FUEL from robot inventory
            inventory = robotState.gamepieces.get().copy()
            if inventory.get(Fuel, 0) > 0:
                inventory[Fuel] -= 1
                changes.append(ValueChange(robotState.gamepieces, inventory))
            
            # Add scored ball to Neutral Zone (recycle mechanic)
            try:
                neutral_zone_state = gameState.get("interactables").get("Neutral Zone")
                if neutral_zone_state is not None:
                    changes.append(ValueIncrease(neutral_zone_state.fuel_available, 1))
            except (KeyError, AttributeError):
                # Neutral Zone not registered yet, skip recycle
                pass
            
            return changes
        
        return score_fuel
    
    def get_interactions(self) -> List[InteractionOption]:
        return [
            InteractionOption(
                "score_fuel",
                f"Score FUEL in the {self.alliance.name} Hub",
                has_fuel_condition,
                Hub.__generate_score_fuel_function(self.alliance),
            ),
        ]


# =============================================================================
# TOWER STATE & INTERACTABLE
# =============================================================================

class TowerState(StateSpace):
    """State for a Tower climbing structure.
    
    Tracks robots at each climb level and total tower points.
    """
    
    def __init__(self):
        super().__init__()
        self.setValue("tower_score", 0)
        self.setValue("level_1_count", 0)  # Max 2 in AUTO, 3 in TELEOP
        self.setValue("level_2_count", 0)
        self.setValue("level_3_count", 0)
        # Track which robots have climbed (to prevent double-climbing)
        self.setValue("climbed_robots", [])
    
    @property
    def tower_score(self):
        return self.getValue("tower_score")
    
    @property
    def level_1_count(self):
        return self.getValue("level_1_count")
    
    @property
    def level_2_count(self):
        return self.getValue("level_2_count")
    
    @property
    def level_3_count(self):
        return self.getValue("level_3_count")
    
    def has_climbed(self, robot_name: str) -> bool:
        """Check if a robot has already climbed."""
        return robot_name in self.getValue("climbed_robots").get()


def robot_not_climbed_condition(interactableState: TowerState, robotState: RobotState, gameState: GameState) -> bool:
    """Condition: Robot has not already climbed this Tower."""
    robot_name = robotState.name if hasattr(robotState, 'name') else robotState.getValue("name").get()
    return not interactableState.has_climbed(robot_name)


def can_climb_level_2_or_3(interactableState: TowerState, robotState: RobotState, gameState: GameState) -> bool:
    """Condition: Only in TELEOP (not AUTO), and robot hasn't climbed."""
    return not is_auto(gameState) and robot_not_climbed_condition(interactableState, robotState, gameState)


class Tower(RobotInteractable):
    """Tower climbing structure for REBUILT.
    
    Robots can climb to Level 1 (AUTO or TELEOP), Level 2 or 3 (TELEOP only).
    Points:
    - Level 1: 15 pts AUTO, 10 pts TELEOP
    - Level 2: 20 pts TELEOP
    - Level 3: 30 pts TELEOP
    """
    
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        alliance: Alliance,
        name: str = "",
    ):
        super().__init__(Point(*center, Inch(0)), name or f"{alliance.name} Tower", navigation_point)
        self.alliance = alliance
    
    @staticmethod
    def initializeInteractableState() -> TowerState:
        return TowerState()
    
    @staticmethod
    def __generate_climb_function(level: int, points_auto: int, points_teleop: int, alliance: Alliance):
        """Generate a climb function for a specific level."""
        def climb(
            interactableState: TowerState,
            robotState: RobotState,
            gameState: GameState,
        ) -> List[ValueChange]:
            changes = []
            
            # Determine points based on match period
            if is_auto(gameState):
                points = points_auto
            else:
                points = points_teleop
            
            # Award points
            changes.extend([
                ValueIncrease(interactableState.tower_score, points),
                ValueIncrease(gameState.score, points),
                ValueIncrease(gameState.get_alliance_score(alliance), points),
                ValueIncrease(interactableState.getValue(f"level_{level}_count"), 1),
            ])
            
            # Mark robot as climbed
            climbed_list = interactableState.getValue("climbed_robots").get().copy()
            robot_name = robotState.name if hasattr(robotState, 'name') else robotState.getValue("name").get()
            climbed_list.append(robot_name)
            changes.append(ValueChange(interactableState.getValue("climbed_robots"), climbed_list))
            
            # Mark robot as game over (cannot take other actions)
            robotState.setValue("gameover", True)
            
            return changes
        
        return climb
    
    def get_interactions(self) -> List[InteractionOption]:
        return [
            InteractionOption(
                "climb_level_1",
                f"Climb to Level 1 on {self.alliance.name} Tower",
                robot_not_climbed_condition,
                Tower.__generate_climb_function(1, 15, 10, self.alliance),
            ),
            InteractionOption(
                "climb_level_2",
                f"Climb to Level 2 on {self.alliance.name} Tower",
                can_climb_level_2_or_3,
                Tower.__generate_climb_function(2, 0, 20, self.alliance),
            ),
            InteractionOption(
                "climb_level_3",
                f"Climb to Level 3 on {self.alliance.name} Tower",
                can_climb_level_2_or_3,
                Tower.__generate_climb_function(3, 0, 30, self.alliance),
            ),
        ]


# =============================================================================
# CONFIGURATION CLASSES
# =============================================================================

from dataclasses import dataclass


@dataclass
class DefenseConfig:
    """Configurable defense parameters."""
    time_multiplier: float = 1.5  # How much to slow down opponent pickups
    defense_duration: float = 5.0  # How long a defense action lasts
    cooldown: float = 2.0  # Time before robot can defend again


@dataclass
class RobotStorageConfig:
    """Configurable robot ball storage."""
    max_ball_capacity: int = 30  # Maximum balls robot can carry
    intake_rate: float = 1.0  # Multiplier for pickup time


# Global defense config (can be overridden)
DEFAULT_DEFENSE_CONFIG = DefenseConfig()


# =============================================================================
# ZONE STATE BASE CLASS
# =============================================================================

class ZoneState(StateSpace):
    """Base state for fuel storage zones with defense tracking."""
    
    def __init__(self, initial_fuel: int = 0):
        super().__init__()
        self.setValue("fuel_available", initial_fuel)
        self.setValue("defending_robots", {})  # robot_name -> defense_end_time
    
    @property
    def fuel_available(self):
        return self.getValue("fuel_available")
    
    def is_defended_by(self, alliance: Alliance, current_time: float) -> bool:
        """Check if zone is defended by the given alliance."""
        defenders = self.getValue("defending_robots").get()
        for robot_name, end_time in list(defenders.items()):
            if end_time > current_time:
                # Check alliance from robot name (assumes naming like "Blue1", "Red2")
                if alliance.name.lower() in robot_name.lower():
                    return True
        return False
    
    def get_defense_multiplier(self, robot_alliance: Alliance, current_time: float, config: DefenseConfig = None) -> float:
        """Get the time multiplier for a robot picking up from this zone.
        
        If the zone is defended by the opposing alliance, apply the multiplier.
        """
        if config is None:
            config = DEFAULT_DEFENSE_CONFIG
        
        defenders = self.getValue("defending_robots").get()
        for robot_name, end_time in list(defenders.items()):
            if end_time > current_time:
                # Check if defender is from opposing alliance
                defender_is_blue = "blue" in robot_name.lower()
                robot_is_blue = robot_alliance == Alliance.BLUE
                if defender_is_blue != robot_is_blue:
                    return config.time_multiplier
        return 1.0


# =============================================================================
# DEPOT (ALLIANCE FUEL PICKUP LOCATION)
# =============================================================================

class DepotState(ZoneState):
    """State for a Depot FUEL pickup location."""
    
    def __init__(self, initial_fuel: int = 24):
        super().__init__(initial_fuel)


def _create_depot_pickup_condition(amount: int):
    """Create condition for depot pickup of given amount."""
    def condition(interactableState: DepotState, robotState: RobotState, gameState: GameState) -> bool:
        available = interactableState.fuel_available.get()
        # Check robot capacity if available
        current_held = sum(robotState.gamepieces.get().values()) if robotState.gamepieces.get() else 0
        max_capacity = getattr(robotState, 'max_ball_capacity', 50)  # Default high if not set
        can_carry = max_capacity - current_held
        return available >= amount and can_carry >= amount
    return condition


def _create_depot_pickup_action(amount: int):
    """Create action for depot pickup of given amount."""
    def action(
        interactableState: DepotState,
        robotState: RobotState,
        gameState: GameState,
    ) -> List[ValueChange]:
        changes = []
        
        # Decrease Depot fuel count
        changes.append(ValueDecrease(interactableState.fuel_available, amount))
        
        # Add FUEL to robot inventory
        inventory = robotState.gamepieces.get().copy()
        inventory[Fuel] = inventory.get(Fuel, 0) + amount
        changes.append(ValueChange(robotState.gamepieces, inventory))
        
        return changes
    return action


class Depot(RobotInteractable):
    """Depot FUEL pickup location.
    
    Robots can pick up FUEL from their alliance's Depot.
    Each Depot starts with 24 FUEL.
    Supports discrete pickup amounts: 1, 5, or 10 balls.
    """
    
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        alliance: Alliance,
        name: str = "",
        initial_fuel: int = 24,
    ):
        super().__init__(Point(*center, Inch(0)), name or f"{alliance.name} Depot", navigation_point)
        self.alliance = alliance
        self._initial_fuel = initial_fuel
    
    def initializeInteractableState(self) -> DepotState:
        return DepotState(initial_fuel=self._initial_fuel)
    
    def get_interactions(self) -> List[InteractionOption]:
        return [
            InteractionOption(
                "pickup_1",
                f"Pick up 1 FUEL from {self.alliance.name} Depot",
                _create_depot_pickup_condition(1),
                _create_depot_pickup_action(1),
            ),
            InteractionOption(
                "pickup_5",
                f"Pick up 5 FUEL from {self.alliance.name} Depot",
                _create_depot_pickup_condition(5),
                _create_depot_pickup_action(5),
            ),
            InteractionOption(
                "pickup_10",
                f"Pick up 10 FUEL from {self.alliance.name} Depot",
                _create_depot_pickup_condition(10),
                _create_depot_pickup_action(10),
            ),
        ]


# =============================================================================
# NEUTRAL ZONE (SHARED FUEL PICKUP LOCATION)
# =============================================================================

class NeutralZoneState(ZoneState):
    """State for the Neutral Zone fuel storage.
    
    Starts empty, gains balls when Hub is scored (+1 per score).
    Both alliances can pick up from here.
    """
    
    def __init__(self, initial_fuel: int = 0):
        super().__init__(initial_fuel)


def _create_zone_pickup_condition(amount: int):
    """Create condition for zone pickup of given amount."""
    def condition(interactableState: ZoneState, robotState: RobotState, gameState: GameState) -> bool:
        available = interactableState.fuel_available.get()
        current_held = sum(robotState.gamepieces.get().values()) if robotState.gamepieces.get() else 0
        max_capacity = getattr(robotState, 'max_ball_capacity', 50)
        can_carry = max_capacity - current_held
        return available >= amount and can_carry >= amount
    return condition


def _create_zone_pickup_action(amount: int):
    """Create action for zone pickup of given amount."""
    def action(
        interactableState: ZoneState,
        robotState: RobotState,
        gameState: GameState,
    ) -> List[ValueChange]:
        changes = []
        
        # Decrease zone fuel count
        changes.append(ValueDecrease(interactableState.fuel_available, amount))
        
        # Add FUEL to robot inventory
        inventory = robotState.gamepieces.get().copy()
        inventory[Fuel] = inventory.get(Fuel, 0) + amount
        changes.append(ValueChange(robotState.gamepieces, inventory))
        
        return changes
    return action


def _defend_zone_condition(interactableState: ZoneState, robotState: RobotState, gameState: GameState) -> bool:
    """Condition: Robot is not already defending elsewhere."""
    # For simplicity, allow defending (could add more complex logic later)
    return True


def _create_defend_zone_action(defense_duration: float = 5.0):
    """Create action for defending a zone."""
    def action(
        interactableState: ZoneState,
        robotState: RobotState,
        gameState: GameState,
    ) -> List[ValueChange]:
        changes = []
        
        # Get robot name
        robot_name = robotState.name if hasattr(robotState, 'name') else "Unknown"
        
        # Add robot to defenders list with end time
        current_time = gameState.current_time.get()
        end_time = current_time + defense_duration
        
        defenders = interactableState.getValue("defending_robots").get().copy()
        defenders[robot_name] = end_time
        changes.append(ValueChange(interactableState.getValue("defending_robots"), defenders))
        
        return changes
    return action


class NeutralZone(RobotInteractable):
    """Neutral Zone shared fuel storage.
    
    Located in the center of the field. Both alliances can pick up from here.
    Gains +1 ball whenever a robot scores in the Hub.
    Can be defended to slow opponent pickups.
    """
    
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        name: str = "Neutral Zone",
        initial_fuel: int = 0,
        defense_config: DefenseConfig = None,
    ):
        super().__init__(Point(*center, Inch(0)), name, navigation_point)
        self._initial_fuel = initial_fuel
        self.defense_config = defense_config or DEFAULT_DEFENSE_CONFIG
    
    def initializeInteractableState(self) -> NeutralZoneState:
        return NeutralZoneState(initial_fuel=self._initial_fuel)
    
    def get_interactions(self) -> List[InteractionOption]:
        return [
            InteractionOption(
                "pickup_1",
                "Pick up 1 FUEL from Neutral Zone",
                _create_zone_pickup_condition(1),
                _create_zone_pickup_action(1),
            ),
            InteractionOption(
                "pickup_5",
                "Pick up 5 FUEL from Neutral Zone",
                _create_zone_pickup_condition(5),
                _create_zone_pickup_action(5),
            ),
            InteractionOption(
                "pickup_10",
                "Pick up 10 FUEL from Neutral Zone",
                _create_zone_pickup_condition(10),
                _create_zone_pickup_action(10),
            ),
            InteractionOption(
                "defend",
                "Defend the Neutral Zone",
                _defend_zone_condition,
                _create_defend_zone_action(self.defense_config.defense_duration),
            ),
        ]


# =============================================================================
# ALLIANCE ZONE (TEAM FUEL STORAGE)
# =============================================================================

class AllianceZoneState(ZoneState):
    """State for an Alliance Zone fuel storage.
    
    Starts empty, gains balls from:
    - Missed shots (40% chance goes here)
    - Shuttling actions from robots
    
    Both alliances can pick up (with defense penalty for opponents).
    """
    
    def __init__(self, owning_alliance: Alliance, initial_fuel: int = 0):
        super().__init__(initial_fuel)
        self.setValue("owning_alliance", owning_alliance.name)
    
    @property
    def owning_alliance(self) -> str:
        return self.getValue("owning_alliance").get()


class AllianceZone(RobotInteractable):
    """Alliance Zone team fuel storage.
    
    Located near each alliance's side of the field.
    Starts empty, filled by shuttling or missed shots.
    Own alliance picks up normally, opponents face defense penalty.
    Can be defended to slow opponent pickups.
    """
    
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        alliance: Alliance,
        name: str = "",
        initial_fuel: int = 0,
        defense_config: DefenseConfig = None,
    ):
        super().__init__(Point(*center, Inch(0)), name or f"{alliance.name} Alliance Zone", navigation_point)
        self.alliance = alliance
        self._initial_fuel = initial_fuel
        self.defense_config = defense_config or DEFAULT_DEFENSE_CONFIG
    
    def initializeInteractableState(self) -> AllianceZoneState:
        return AllianceZoneState(owning_alliance=self.alliance, initial_fuel=self._initial_fuel)
    
    def get_interactions(self) -> List[InteractionOption]:
        return [
            InteractionOption(
                "pickup_1",
                f"Pick up 1 FUEL from {self.alliance.name} Alliance Zone",
                _create_zone_pickup_condition(1),
                _create_zone_pickup_action(1),
            ),
            InteractionOption(
                "pickup_5",
                f"Pick up 5 FUEL from {self.alliance.name} Alliance Zone",
                _create_zone_pickup_condition(5),
                _create_zone_pickup_action(5),
            ),
            InteractionOption(
                "pickup_10",
                f"Pick up 10 FUEL from {self.alliance.name} Alliance Zone",
                _create_zone_pickup_condition(10),
                _create_zone_pickup_action(10),
            ),
            InteractionOption(
                "defend",
                f"Defend the {self.alliance.name} Alliance Zone",
                _defend_zone_condition,
                _create_defend_zone_action(self.defense_config.defense_duration),
            ),
        ]
