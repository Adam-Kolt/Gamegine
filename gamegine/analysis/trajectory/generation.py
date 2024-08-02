from abc import ABC, abstractmethod
from typing import List, Tuple
from dataclasses import dataclass
import pint

from gamegine.analysis.pathfinding import Path
from gamegine.render.drawable import Drawable
from gamegine.representation.bounds import DiscreteBoundary
from gamegine.utils.unit import (
    MOI,
    AngularMeasurement,
    ForceMeasurement,
    MassMeasurement,
    MetersPerSecond,
    Omega,
    RadiansPerSecond,
    SpatialMeasurement,
    Torque,
    Velocity,
)


class DrivetrainParameters(ABC):
    pass


@dataclass
class SwerveDrivetrainParameters:
    max_module_speed: Velocity
    max_module_force: Torque
    wheel_radius: SpatialMeasurement
    wheel_base: SpatialMeasurement
    track_width: SpatialMeasurement
    moi: MOI
    mass: MassMeasurement


@dataclass
class TrajectoryKeypoint:
    x: SpatialMeasurement = None
    y: SpatialMeasurement = None
    theta: AngularMeasurement = None
    velocity_x: Velocity = MetersPerSecond(0.0)
    velocity_y: Velocity = MetersPerSecond(0.0)
    omega: Omega = RadiansPerSecond(0.0)


class GuidedTrajectoryGenerator(ABC):
    @abstractmethod
    def calculate_trajectory(
        self,
        guide_path: Path,
        obstacles: List[DiscreteBoundary],
        start_parameters: TrajectoryKeypoint,
        end_parameters: TrajectoryKeypoint,
        drivetrain_parameters: DrivetrainParameters,
    ) -> "Trajectory":
        pass


class GuidedSwerveTrajectoryGenerator(GuidedTrajectoryGenerator):
    @abstractmethod
    def calculate_trajectory(
        self,
        guide_path: Path,
        obstacles: List[DiscreteBoundary],
        start_parameters: TrajectoryKeypoint,
        end_parameters: TrajectoryKeypoint,
        drivetrain_parameters: SwerveDrivetrainParameters,
    ) -> "SwerveTrajectory":
        pass


class Trajectory(Drawable):
    @abstractmethod
    def get_discrete_trajectory(
        self, poll_step: SpatialMeasurement
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement]]:
        pass


class SwerveTrajectory(Trajectory):
    @abstractmethod
    def get_discrete_trajectory(
        self, poll_step: SpatialMeasurement
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement]]:
        pass
