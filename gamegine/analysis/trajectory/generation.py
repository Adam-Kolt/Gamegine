from abc import ABC, abstractmethod
from typing import List, Tuple
from dataclasses import dataclass
import pint

from gamegine.analysis.pathfinding import Path
from gamegine.render.drawable import Drawable
from gamegine.representation.bounds import DiscreteBoundary
from gamegine.utils.unit import (
    AngularMeasurement,
    ForceMeasurement,
    MassMeasurement,
    SpatialMeasurement,
)


class DrivetrainParameters(ABC):
    pass


@dataclass
class SwerveDrivetrainParameters:
    max_module_speed: SpatialMeasurement
    max_module_force: ForceMeasurement
    wheel_radius: SpatialMeasurement
    wheel_base: SpatialMeasurement
    track_width: SpatialMeasurement
    moi: float
    mass: MassMeasurement

    def __init__(
        self,
        max_module_speed: SpatialMeasurement = None,
        max_module_force: ForceMeasurement = None,
        wheel_radius: SpatialMeasurement = None,
        wheel_base: SpatialMeasurement = None,
        track_width: SpatialMeasurement = None,
        moi: float = None,
        mass: MassMeasurement = None,
    ):
        self.max_module_speed = max_module_speed
        self.max_module_force = max_module_force
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.track_width = track_width
        self.moi = moi
        self.mass = mass


@dataclass
class TrajectoryKeypoint:
    x: SpatialMeasurement
    y: SpatialMeasurement
    theta: AngularMeasurement
    velocity_x: SpatialMeasurement
    velocity_y: SpatialMeasurement
    omega: AngularMeasurement

    def __init__(
        self,
        x: SpatialMeasurement = None,
        y: SpatialMeasurement = None,
        theta: AngularMeasurement = None,
        velocity_x: SpatialMeasurement = None,
        velocity_y: SpatialMeasurement = None,
        omega: AngularMeasurement = None,
    ):
        self.x = x
        self.y = y
        self.theta = theta
        self.velocity_x = velocity_x
        self.velocity_y = velocity_y
        self.omega = omega


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
