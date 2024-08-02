from dataclasses import dataclass
from typing import List

from gamegine.representation.base import NamedObject
from gamegine.representation.bounds import Boundary3D
from dataclasses import dataclass

from gamegine.utils.unit import (
    MOI,
    ComplexMeasurement,
    MassMeasurement,
    Omega,
    RadiansPerSecond,
    SpatialMeasurement,
    Torque,
    Velocity,
)


@dataclass
class PhysicalParameters:
    mass: MassMeasurement
    moi: MOI


class Robot(NamedObject):
    def __init__(
        self,
        name: str,
        structure: List[Boundary3D] = [],
        physics: PhysicalParameters = None,
    ) -> None:
        super().__init__("Robot " + name)


@dataclass
class SwerveModuleCharacteristics:
    max_module_speed: Velocity
    max_module_force: Torque
    wheel_radius: SpatialMeasurement
    rotation_speed: Omega = RadiansPerSecond(6.28)


@dataclass
class SwerveDrivetrainCharacteristics:
    module: SwerveModuleCharacteristics
    wheel_base: SpatialMeasurement
    track_width: SpatialMeasurement


class SwerveRobot(Robot):
    def __init__(
        self,
        name: str,
        drivetrain: SwerveDrivetrainCharacteristics,
        structure: List[Boundary3D] = [],
        physics: PhysicalParameters = None,
    ) -> None:
        super().__init__(name, structure, physics)
