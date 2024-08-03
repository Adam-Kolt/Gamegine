from dataclasses import dataclass
from typing import List

from gamegine.representation.base import NamedObject
from gamegine.representation.bounds import Boundary3D
from dataclasses import dataclass

from gamegine.utils.unit import (
    MOI,
    ComplexMeasurement,
    Inch,
    MassMeasurement,
    MetersPerSecond,
    NewtonMeter,
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
    max_module_speed: Velocity = MetersPerSecond(6.0)
    max_module_force: Torque = NewtonMeter(7.0)
    wheel_radius: SpatialMeasurement = Inch(2)
    rotation_speed: Omega = RadiansPerSecond(6.28)


@dataclass
class SwerveDrivetrainCharacteristics:
    module: SwerveModuleCharacteristics = SwerveModuleCharacteristics()
    wheel_base: SpatialMeasurement = Inch(30)
    track_width: SpatialMeasurement = Inch(30)


class SwerveRobot(Robot):
    def __init__(
        self,
        name: str,
        drivetrain: SwerveDrivetrainCharacteristics,
        structure: List[Boundary3D] = [],
        physics: PhysicalParameters = None,
    ) -> None:
        super().__init__(name, structure, physics)
