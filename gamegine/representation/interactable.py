from enum import Enum
from abc import ABC, abstractmethod
from typing import Callable, List

from gamegine.simulation.robot import RobotState
from gamegine.simulation.state import StateSpace, ValueChange

# IN PROGRESSS


class RobotInteractable(ABC):
    @abstractmethod
    @staticmethod
    def initializeInteractableState() -> StateSpace:
        pass

    @abstractmethod
    @staticmethod
    def get_interactions() -> List["InteractionOption"]:
        pass


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
