from typing import Dict, List, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from gamegine.representation.zone import TraversalZone

from gamegine.representation.bounds import Rectangle
from gamegine.representation.interactable import RobotInteractable

from gamegine.utils.logging import Debug
from gamegine.utils.NCIM.ncim import Meter, SpatialMeasurement, Zero, Feet
import numpy as np
from gamegine.representation import obstacle


class Game(object):
    """A class representing an FRC Game. Serves as the base container for all other representation objects such as obstacles, interactables, and zones. Also stores any and all game logic and state variables.

    :param name: The name of the game. Defaults to "FRC Game".
    :type name: str, optional
    """

    def __init__(self, name: str = "FRC Game") -> None:
        """Constructor method for the Game class. Initializes the game with a name and default field size of 15m x 15m."""
        Debug(f"Creating new game: {name}")
        self.name = name
        self.field_size = (Meter(15.0), Meter(15.0))

        self.static_obstacles = {}
        self.interactables: Dict[str, RobotInteractable] = {}
        self.zones = {}
        self.global_states = {}
        self.field_borders = False

    def set_field_size(self, width: SpatialMeasurement, height: SpatialMeasurement):
        """Sets the field size of the game.

        :param width: The width of the field.
        :type width: :class:`SpatialMeasurement`
        :param height: The height of the field.
        :type height: :class:`SpatialMeasurement`
        """
        self.field_size = (width, height)
        if self.field_borders:
            self.__recompute_field_border_obstacles()

    def get_field_size(self) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        """Returns the field size of the game.

        :return: The field size of the game.
        :rtype: Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]
        """
        return self.field_size

    def half_field_size(self) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        """Returns the half of the field size of the game.

        :return: The half of the field size of the game.
        :rtype: Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]
        """
        return (self.field_size[0] / 2, self.field_size[1] / 2)

    def half_field_x(self) -> SpatialMeasurement:
        """Returns the half of the field size of the game along the x-axis.

        :return: The half of the field size of the game along the x-axis.
        :rtype: :class:`SpatialMeasurement`
        """
        return self.field_size[0] / 2

    def half_field_y(self):
        """Returns the half of the field size of the game along the y-axis.

        :return: The half of the field size of the game along the y-axis.
        :rtype: :class:`SpatialMeasurement`
        """
        return self.field_size[1] / 2

    def full_field_y(self) -> SpatialMeasurement:
        """Returns the size of the field along the y-axis.

        :return: The size of the field along the y-axis.
        :rtype: :class:`SpatialMeasurement`
        """
        return self.field_size[1]

    def full_field_x(self):
        """
        Returns the size of the field along the x-axis.

        :return: The size of the field along the x-axis.
        :rtype: :class:`SpatialMeasurement`
        """
        return self.field_size[0]

    def add_obstacle(self, obstacle: obstacle.Obstacle) -> "Game":
        """Adds an obstacle to the game.

        :param obstacle: The obstacle to add.
        :type obstacle: :class:`Obstacle`
        """
        if obstacle.name in self.static_obstacles:
            raise Exception(
                f"Obstacle {obstacle.name} already exists. Names must be unique."
            )  #  ¯\_(ツ)_/¯

        self.static_obstacles[obstacle.name] = obstacle
        return self

    def modify_obstacle(self, obstacle: obstacle.Obstacle) -> "Game":
        """Replaces an existing obstacle with a new one, allowing for modifications to the obstacle. If the obstacle does not exist, it will be added.

        :param obstacle: The obstacle to modify.
        :type obstacle: :class:`Obstacle`
        :return: The game object with the modified obstacle.
        :rtype: :class:`Game`
        """
        self.static_obstacles[obstacle.name] = obstacle
        return self

    def modify_obstacles(self, obstacles: List[obstacle.Obstacle]) -> "Game":
        """Replaces existing obstacles with new ones, allowing for modifications to the obstacles. If an obstacle does not exist, it will be added.

        :param obstacles: The obstacles to modify.
        :type obstacles: List[:class:`Obstacle`]
        :return: The game object with the modified obstacles.
        :rtype: :class:`Game`"""
        for obstacle in obstacles:
            self.modify_obstacle(obstacle)
        return self

    def get_obstacle(self, name: str) -> obstacle.Obstacle:
        """Returns the obstacle with the given name.

        :param name: The name of the obstacle.
        :type name: str
        :return: The obstacle with the given name.
        :rtype: :class:`Obstacle`
        """
        return self.static_obstacles[name]

    def add_obstacles(self, obstacles: List[obstacle.Obstacle]) -> "Game":
        """Adds a list of obstacles to the game.

        :param obstacles: The obstacles to add.
        :type obstacles: List[:class:`Obstacle`]
        :return: The game object with the added obstacles.
        :rtype: :class:`Game`
        """

        for obstacle in obstacles:
            self.add_obstacle(obstacle)
        return self

    def __recompute_field_border_obstacles(self) -> "Game":
        """Recomputes the field border obstacles based on the current field size."""
        thickness = Meter(0.1)  # In order to make the border a valid obstacle
        self.modify_obstacles(
            [
                obstacle.Obstacle(
                    "Field Border Top",
                    Rectangle(
                        Zero(), Zero() - thickness, self.field_size[0], thickness
                    ).get_3d(Feet(0), Feet(3)),
                ).invisible(),
                obstacle.Obstacle(
                    "Field Border Bottom",
                    Rectangle(
                        Zero(), self.field_size[1], self.field_size[0], thickness
                    ).get_3d(Feet(0), Feet(3)),
                ).invisible(),
                obstacle.Obstacle(
                    "Field Border Left",
                    Rectangle(
                        Zero() - thickness, Zero(), thickness, self.field_size[1]
                    ).get_3d(Feet(0), Feet(3)),
                ).invisible(),
                obstacle.Obstacle(
                    "Field Border Right",
                    Rectangle(
                        self.field_size[0], Zero(), thickness, self.field_size[1]
                    ).get_3d(Feet(0), Feet(3)),
                ).invisible(),
            ]
        )

    def enable_field_border_obstacles(self) -> "Game":
        """Enables the field border obstacles."""
        self.field_borders = True
        self.__recompute_field_border_obstacles()
        return self

    def get_obstacles(self) -> List[obstacle.Obstacle]:
        """Returns all obstacles in the game.

        :return: All obstacles in the game.
        :rtype: List[:class:`Obstacle`]
        """

        return self.static_obstacles.values()

    def add_interactable(self, interactable: RobotInteractable) -> "Game":
        if interactable.name in self.interactables:
            raise Exception(
                f"Interactable {interactable.name} already exists. Names must be unique."
            )

        self.interactables[interactable.name] = interactable
        return self

    def get_interactables(self) -> List[RobotInteractable]:
        return self.interactables.values()



    def add_zone(self, zone: "TraversalZone") -> "Game":
        """Adds a traversal zone to the game.
        
        :param zone: The zone to add.
        :type zone: :class:`TraversalZone`
        :return: The game object with the added zone.
        :rtype: :class:`Game`
        """
        if zone.name in self.zones:
            raise Exception(f"Zone {zone.name} already exists. Names must be unique.")

        self.zones[zone.name] = zone
        return self

    def get_zones(self) -> List["TraversalZone"]:
        """Returns all traversal zones in the game.
        
        :return: All zones in the game.
        :rtype: List[:class:`TraversalZone`]
        """
        return list(self.zones.values())
