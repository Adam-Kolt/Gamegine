from abc import ABC, abstractmethod
from typing import Callable
from dataclasses import dataclass
from gamegine.render.drawable import Drawable
from gamegine.representation.bounds import Boundary3D, BoundedObject
from gamegine.utils.NCIM.Dimensions.mass import MassMeasurement
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement


@dataclass
class GamepiecePhysicalProperties:
    mass: MassMeasurement
    friction_coefficient: float


class Gamepiece(ABC):
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
        pass

    def __new__(
        cls, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> "GamepieceInstance":
        return GamepieceInstance(cls, x, y)


class GamepieceInstance(BoundedObject, Drawable):
    def __init__(
        self, gamepiece: Gamepiece, x: SpatialMeasurement, y: SpatialMeasurement
    ):
        super().__init__(gamepiece.bounds)
        self.gamepiece = gamepiece
        self.x = x
        self.y = y

    def draw(self, render_scale: SpatialMeasurement):
        self.gamepiece.display(self.x, self.y, render_scale)
