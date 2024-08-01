from dataclasses import dataclass
from typing import List

from gamegine.representation.base import NamedObject
from gamegine.representation.bounds import Boundary3D
from dataclasses import dataclass

from gamegine.utils.unit import ComplexMeasurement, MassMeasurement, SpatialMeasurement


@dataclass
class PhysicalParameters:
    mass: MassMeasurement
    moi: ComplexMeasurement


class Robot(NamedObject):
    def __init__(
        self,
        name: str,
        structure: List[Boundary3D] = [],
        physics: PhysicalParameters = None,
    ) -> None:
        super().__init__("Robot " + name)


class SwerveRobot(Robot):
    def __init__(
        self,
        name: str,
        structure: List[Boundary3D] = [],
        physics: PhysicalParameters = None,
    ) -> None:
        super().__init__(name, structure, physics)
