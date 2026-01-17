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
# DEPOT (FUEL PICKUP LOCATION)
# =============================================================================

class DepotState(StateSpace):
    """State for a Depot FUEL pickup location."""
    
    def __init__(self, initial_fuel: int = 24):
        super().__init__()
        self.setValue("fuel_available", initial_fuel)
    
    @property
    def fuel_available(self):
        return self.getValue("fuel_available")


def depot_has_fuel(interactableState: DepotState, robotState: RobotState, gameState: GameState) -> bool:
    """Condition: Depot has FUEL available."""
    return interactableState.fuel_available.get() > 0


class Depot(RobotInteractable):
    """Depot FUEL pickup location.
    
    Robots can pick up FUEL from their alliance's Depot.
    Each Depot starts with 24 FUEL.
    """
    
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        alliance: Alliance,
        name: str = "",
    ):
        super().__init__(Point(*center, Inch(0)), name or f"{alliance.name} Depot", navigation_point)
        self.alliance = alliance
    
    @staticmethod
    def initializeInteractableState() -> DepotState:
        return DepotState(initial_fuel=24)
    
    @staticmethod
    def pickup_fuel(
        interactableState: DepotState,
        robotState: RobotState,
        gameState: GameState,
    ) -> List[ValueChange]:
        """Pick up a FUEL from the Depot."""
        changes = []
        
        # Decrease Depot fuel count
        changes.append(ValueDecrease(interactableState.fuel_available, 1))
        
        # Add FUEL to robot inventory
        inventory = robotState.gamepieces.get().copy()
        inventory[Fuel] = inventory.get(Fuel, 0) + 1
        changes.append(ValueChange(robotState.gamepieces, inventory))
        
        return changes
    
    @staticmethod
    def get_interactions() -> List[InteractionOption]:
        return [
            InteractionOption(
                "pickup_fuel",
                "Pick up FUEL from Depot",
                depot_has_fuel,
                Depot.pickup_fuel,
            ),
        ]
