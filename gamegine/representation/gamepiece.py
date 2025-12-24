from abc import ABC, abstractmethod
from typing import Callable
from dataclasses import dataclass
# Rendering is handled by gamegine.render.handlers, not embedded here
from gamegine.representation.bounds import Boundary3D, BoundedObject
from gamegine.simulation.gamepiece import GamepieceInstance
from gamegine.utils.NCIM.Dimensions.mass import MassMeasurement
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement


@dataclass
class GamepiecePhysicalProperties:
    """Stores the physical properties of a gamepiece, including mass and friction coefficient."""

    mass: MassMeasurement
    friction_coefficient: float


class Gamepiece(ABC):
    """Abstract class which holds structure for representing a gamepiece, consisting of a name, bounds, and physical properties."""

    name: str
    bounds: Boundary3D
    physical_properties: GamepiecePhysicalProperties

    @classmethod
    @abstractmethod
    def display(
        cls,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        render_scale: SpatialMeasurement,
    ):
        """Abstract method for displaying a gamepiece on the field screen.

        :param x: The x-coordinate of the gamepiece.
        :type x: :class:`SpatialMeasurement`
        :param y: The y-coordinate of the gamepiece.
        :type y: :class:`SpatialMeasurement`
        :param render_scale: The scale at which the gamepiece should be rendered.
        :type render_scale: :class:`SpatialMeasurement`
        """
        pass

    def __new__(
        cls, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> "GamepieceInstance":
        return GamepieceInstance(cls, x, y)
