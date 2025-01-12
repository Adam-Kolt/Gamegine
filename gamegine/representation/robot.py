from dataclasses import dataclass, field
from typing import Dict, List

from gamegine.first.alliance import Alliance
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.representation import gamepiece
from gamegine.representation.base import NamedObject
from gamegine.representation.bounds import Boundary, Boundary3D
from dataclasses import dataclass

from gamegine.representation.interactable import RobotInteractionConfig
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
        self.structure = structure
        self.physics = physics
        self.interaction_configs = {}
        self.override_radius = None

    def add_interaction_config(self, config: RobotInteractionConfig) -> None:
        """Adds an interaction configuration to the robot.

        :param config: The interaction configuration to add.
        :type config: :class:`gamepiece.GamepieceConfig`"""
        if not config.interactable_name in self.interaction_configs:
            self.interaction_configs[config.interactable_name] = {}
        self.interaction_configs[config.interactable_name][
            config.interaction_identifier
        ] = config

    def get_bounding_radius(self) -> SpatialMeasurement:
        """Returns the bounding radius of the robot.

        :return: The bounding radius of the robot.
        :rtype: :class:`SpatialMeasurement`"""
        if self.override_radius is not None:
            return self.override_radius

        max_radius = Inch(0)
        for structure in self.structure:
            points = structure.discretized().get_vertices()
            for point in points:
                radius = (point[0] ** 2 + point[1] ** 2) ** 0.5
                if radius > max_radius:
                    max_radius = radius

        return max_radius

    def true_radius(self) -> SpatialMeasurement:
        """Returns the true radius of the robot.

        :return: The true radius of the robot.
        :rtype: :class:`SpatialMeasurement`"""

        max_radius = Inch(0)
        for structure in self.structure:
            points = structure.discretized().get_vertices()
            for point in points:
                radius = (point[0] ** 2 + point[1] ** 2) ** 0.5
                if radius > max_radius:
                    max_radius = radius

        return max_radius

    def get_physics(self) -> PhysicalParameters:
        """Returns the physical parameters of the robot.

        :return: The physical parameters of the robot.
        :rtype: :class:`PhysicalParameters`"""
        return self.physics

    def override_bounding_radius(self, radius: SpatialMeasurement) -> None:
        """Overrides the bounding radius of the robot.

        :param radius: The new bounding radius of the robot.
        :type radius: :class:`SpatialMeasurement`"""
        self.override_radius = radius


class SwerveRobot(Robot):
    """Represents a swerve robot on the field."""

    def __init__(
        self,
        name: str,
        drivetrain: SwerveConfig,
        structure: List[Boundary3D] = [],
        physics: PhysicalParameters = None,
    ) -> None:
        super().__init__(name, structure, physics)
        self.drivetrain = drivetrain

    def get_drivetrain(self) -> SwerveConfig:
        """Returns the characteristics of the robot's drivetrain.

        :return: The characteristics of the robot's drivetrain.
        :rtype: :class:`SwerveDrivetrainCharacteristics`"""
        return self.drivetrain

    def get_alliance(self) -> Alliance:
        """Returns the alliance of the robot.

        :return: The alliance of the robot.
        :rtype: :class:`Alliance`"""
        return self.alliance
