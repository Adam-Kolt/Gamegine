from enum import Enum
from abc import ABC, abstractmethod
from typing import Callable, List

from gamegine.render import helpers
from gamegine.render.drawable import Drawable
from gamegine.representation.boundary.boundary import Boundary, BoundedObject
from gamegine.simulation.robot import RobotState
from gamegine.simulation.state import StateSpace, ValueChange
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement

# IN PROGRESSS


class RobotInteractable(BoundedObject, Drawable):
    def __init__(self, boundary: Boundary, name="") -> None:
        super().__init__(boundary, name)

    @staticmethod
    @abstractmethod
    def initializeInteractableState() -> StateSpace:
        pass

    @abstractmethod
    def get_interactions(self) -> List["InteractionOption"]:
        pass

    def draw(self, render_scale: SpatialMeasurement):
        self.bounds.draw(render_scale)


class InteractionOption(object):
    def __init__(
        self,
        identifier: str,
        description: str,
        condition: Callable[[StateSpace, RobotState, StateSpace], bool],
        action: Callable[[StateSpace, RobotState, StateSpace], List[ValueChange]],
    ) -> None:
        self.identifier = identifier
        self.description = description
        self.condition = condition
        self.action = action

    def ableToInteract(
        self,
        interactableState: StateSpace,
        robotState: RobotState,
        gameState: StateSpace,
    ) -> bool:
        return self.condition(interactableState, robotState, gameState)

    def interact(
        self,
        interactableState: StateSpace,
        robotState: RobotState,
        gameState: StateSpace,
    ) -> List[ValueChange]:
        return self.action(interactableState, robotState, gameState)

    def __str__(self):
        return f"{self.identifier}: {self.description}"

    def __repr__(self):
        return f"{self.identifier}: {self.description}"
