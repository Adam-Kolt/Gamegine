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
from gamegine.representation.gamepiece import Gamepiece
from gamepieces import Algae, Coral
from gamegine.first.alliance import Alliance
from gamegine.utils.NCIM.ncim import Inch, Degree, SpatialMeasurement
from gamegine.representation.bounds import Point


class CoralStation(RobotInteractable):
    def __init__(
        self,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        navigation_point: Tuple[SpatialMeasurement, SpatialMeasurement],
        name="",
    ):
        super().__init__(Point(*center, Inch(0)), name, navigation_point)

    @staticmethod
    def initializeInteractableState() -> StateSpace:
        return StateSpace()

    @staticmethod
    def max_coral_condition(
        interactableState: StateSpace, robotState: RobotState, gameState: GameState
    ):
        return robotState.gamepieces.get().get(Coral, 0) < 1

    @staticmethod
    def coral_pickup(
        interactableState: StateSpace, robotState: RobotState, gameState: GameState
    ):
        inventory = robotState.gamepieces.get().copy()
        inventory[Coral] = inventory.get(Coral, 0) + 1
        return [ValueChange(robotState.gamepieces, inventory)]

    @staticmethod
    def get_interactions() -> List["InteractionOption"]:
        return [
            InteractionOption(
                "PickupCoral",
                "Pick up a coral",
                CoralStation.max_coral_condition,
                CoralStation.coral_pickup,
            )
        ]
