from dataclasses import dataclass, field
from typing import Dict, List

from gamegine.first.alliance import Alliance
from gamegine.representation import gamepiece
from gamegine.representation.base import NamedObject
from gamegine.representation.bounds import Boundary, Boundary3D
from dataclasses import dataclass

from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement
from gamegine.utils.NCIM.ncim import (
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
    """Stores the physical parameters of a robot, including mass, moment of inertia, and center of mass."""

    mass: MassMeasurement
    moi: MOI


class Robot(NamedObject):
    """Represents a robot on the field.

    :param name: The name of the robot.
    :type name: str
    :param structure: The structure of the robot.
    :type structure: List[:class:`Boundary3D`]
    :param physics: The physical parameters of the robot.
    :type physics: :class:`PhysicalParameters`"""

    def __init__(
        self,
        name: str,
        structure: List[Boundary3D] = [],
        physics: PhysicalParameters = None,
    ) -> None:
        super().__init__("Robot " + name)


@dataclass
class SwerveModuleCharacteristics:
    """Stores the characteristics of a swerve module, including maximum speed, force, wheel radius, and rotation speed."""

    max_module_speed: Velocity = MetersPerSecond(6.0)
    max_module_force: Torque = NewtonMeter(7.0)
    wheel_radius: SpatialMeasurement = Inch(2)
    rotation_speed: Omega = RadiansPerSecond(6.28)


@dataclass
class SwerveDrivetrainCharacteristics:
    """Stores the characteristics of a swerve drivetrain, including module characteristics, wheel base, and track width."""

    module: SwerveModuleCharacteristics = field(
        default_factory=SwerveModuleCharacteristics
    )
    wheel_base: SpatialMeasurement = Inch(30)
    track_width: SpatialMeasurement = Inch(30)


class SwerveRobot(Robot):
    """Represents a swerve robot on the field."""

    def __init__(
        self,
        name: str,
        drivetrain: SwerveDrivetrainCharacteristics,
        structure: List[Boundary3D] = [],
        physics: PhysicalParameters = None,
    ) -> None:
        super().__init__(name, structure, physics)


@dataclass
class RobotGamestate:
    """Stores the gamestate of a robot, including position, orientation, alliance, bounds, and current gamepieces held."""

    x: SpatialMeasurement
    y: SpatialMeasurement
    theta: AngularMeasurement
    alliance: Alliance
    bounds: Boundary
    gamepieces: Dict[str, int]
