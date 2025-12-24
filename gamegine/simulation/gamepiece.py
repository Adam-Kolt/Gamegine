from __future__ import annotations

# Rendering is handled by gamegine.render.handlers, not embedded here
from gamegine.representation.bounds import BoundedObject
from gamegine.simulation.state import StateSpace, ValueEntry
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from gamegine.representation.gamepiece import Gamepiece


class GamepieceState(StateSpace):
    """Class for representing the state space of a gamepiece, which includes the x and y coordinates of the gamepiece on the field screen.

    :param x: The x-coordinate of the gamepiece.
    :type x: :class:`SpatialMeasurement`
    :param y: The y-coordinate of the gamepiece.
    :type y: :class:`SpatialMeasurement`
    """

    def __init__(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
    ):
        super().__init__()
        self.setValue("x", x)
        self.setValue("y", y)

    @property
    def x(self) -> ValueEntry[SpatialMeasurement]:
        return self.getValue("x")

    @x.setter
    def x(self, x: SpatialMeasurement) -> GamepieceState:
        self.setValue("x", x)
        return self

    @property
    def y(self) -> ValueEntry[SpatialMeasurement]:
        return self.getValue("y")

    @y.setter
    def y(self, y: SpatialMeasurement) -> GamepieceState:
        self.setValue("y", y)
        return self


class GamepieceInstance(BoundedObject):
    def __init__(
        self, gamepiece: Gamepiece, x: SpatialMeasurement, y: SpatialMeasurement
    ):
        super().__init__(gamepiece.bounds)
        self.gamepiece = gamepiece
        self.x = x
        self.y = y


