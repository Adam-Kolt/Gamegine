from dataclasses import dataclass
from enum import Enum
from abc import ABC, abstractmethod
from typing import Callable, List, Tuple

# Rendering is handled by gamegine.render.handlers, not embedded here
from gamegine.representation.bounds import BoundedObject, Boundary
from gamegine.simulation.robot import RobotState
from gamegine.simulation.state import StateSpace, ValueChange
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement


class RobotInteractable(BoundedObject):
    """Class for representing a robot-interactable object during the match, which includes scoring stations and other game elements which the robot can interact with to change the game state.

    :param boundary: The boundary of the interactable object.
    :type boundary: :class:`Boundary`
    :param name: The name of the interactable object. Defaults to an empty string.
    :type name: str, optional
    """

    def __init__(
        self,
        boundary: Boundary,
        name="",
        navigation_point: Tuple[
            SpatialMeasurement, SpatialMeasurement, AngularMeasurement
        ] = None,
    ):
        super().__init__(boundary, name)
        self.navigation_point = navigation_point

    def get_navigation_point(
        self,
    ) -> Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement]:
        return self.navigation_point

    @staticmethod
    @abstractmethod
    def initializeInteractableState() -> StateSpace:
        """Abstract method for initializing the state space of the interactable object."""
        pass

    @staticmethod
    @abstractmethod
    def get_interactions(self) -> List["InteractionOption"]:
        """Gets the available interactions for the robot with the interactable object.

        :return: A list of InteractionOption objects representing the available interactions.
        :rtype: List[:class:`InteractionOption`]"""
        pass


    @classmethod
    def get_interaction(cls, identifier: str) -> "InteractionOption":
        """Gets the interaction option with the given identifier.

        :param identifier: The unique identifier for the interaction option.
        :type identifier: str
        :return: The InteractionOption object with the given identifier.
        :rtype: :class:`InteractionOption`
        """
        for interaction in cls.get_interactions():
            if interaction.identifier == identifier:
                return interaction
        return None


class InteractionOption(object):
    """Class for representing an interaction option for a robot with an interactable object during the match.

    :param identifier: The unique identifier for the interaction option.
    :type identifier: str
    :param description: A description of the interaction option.
    :type description: str
    :param condition: A function which determines whether the interaction can be performed.
    :type condition: Callable[[StateSpace, RobotState, StateSpace], bool]
    :param action: A function which performs the interaction and returns the changes to the game state.
    :type action: Callable[[StateSpace, RobotState, StateSpace], List[ValueChange]]
    """

    def __init__(
        self,
        identifier: str,
        description: str,
        condition: Callable[[StateSpace, RobotState, StateSpace], bool],
        action: Callable[[StateSpace, RobotState, StateSpace], List[ValueChange]],
        navigation_point: Tuple[
            SpatialMeasurement, SpatialMeasurement, AngularMeasurement
        ] = None,
    ) -> None:
        self.identifier = identifier
        self.description = description
        self.condition = condition
        self.action = action
        self.navigation_point = navigation_point

    def ableToInteract(
        self,
        interactableState: StateSpace,
        robotState: RobotState,
        gameState: StateSpace,
    ) -> bool:
        """Determines whether the interaction can be performed.

        :param interactableState: The state space of the interactable object.
        :type interactableState: :class:`StateSpace`
        :param robotState: The state of the robot.
        :type robotState: :class:`RobotState`
        :param gameState: The state space of the game.
        :type gameState: :class:`StateSpace`
        :return: True if the interaction can be performed, False otherwise.
        :rtype: bool
        """
        return self.condition(interactableState, robotState, gameState)

    def interact(
        self,
        interactableState: StateSpace,
        robotState: RobotState,
        gameState: StateSpace,
    ) -> List[ValueChange]:
        """Performs the interaction and returns the changes to the game state.

        :param interactableState: The state space of the interactable object.
        :type interactableState: :class:`StateSpace`
        :param robotState: The state of the robot.
        :type robotState: :class:`RobotState`
        :param gameState: The state space of the game.
        :type gameState: :class:`StateSpace`
        :return: A list of ValueChange objects representing the changes to the game state.
        :rtype: List[:class:`ValueChange`]
        """
        return self.action(interactableState, robotState, gameState)

    def __str__(self):
        return f"{self.identifier}: {self.description}"

    def __repr__(self):
        return f"{self.identifier}: {self.description}"

    def get_navigation_point(
        self,
    ) -> Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement]:
        return self.navigation_point


@dataclass
class RobotInteractionConfig(object):
    """Class for representing the interaction configuration of a robot with an interactable object during the match."""

    interactable_name: str
    interaction_identifier: str
    able_to_interact: Callable[[StateSpace, RobotState, StateSpace], bool]
    time_to_interact: Callable[[StateSpace, RobotState, StateSpace], float]
    navigation_point: Tuple[
        SpatialMeasurement, SpatialMeasurement, AngularMeasurement
    ] = None
