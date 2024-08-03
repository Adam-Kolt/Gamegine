from abc import ABC, abstractmethod
from typing import List, Tuple
from dataclasses import dataclass
import pint

from gamegine.analysis.pathfinding import Path
from gamegine.render import helpers
from gamegine.render.drawable import Drawable
from gamegine.render.style import Palette
from gamegine.representation.bounds import Circle, DiscreteBoundary
from gamegine.representation.robot import (
    PhysicalParameters,
    SwerveDrivetrainCharacteristics,
)
from gamegine.utils.unit import (
    MOI,
    AngularMeasurement,
    ForceMeasurement,
    Inch,
    MassMeasurement,
    MetersPerSecond,
    Omega,
    RadiansPerSecond,
    SpatialMeasurement,
    TimeMeasurement,
    Torque,
    Velocity,
)


class DrivetrainParameters(ABC):
    pass


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
        physical_parameters: PhysicalParameters,
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
        physical_parameters: PhysicalParameters,
        drivetrain_parameters: SwerveDrivetrainCharacteristics,
    ) -> "SwerveTrajectory":
        pass


@dataclass
class TrajectoryState:
    x: SpatialMeasurement
    y: SpatialMeasurement
    angle: AngularMeasurement
    vel_x: Velocity = MetersPerSecond(0.0)
    vel_y: Velocity = MetersPerSecond(0.0)
    acc_x: Velocity = MetersPerSecond(0.0)
    acc_y: Velocity = MetersPerSecond(0.0)


class Trajectory(Drawable):
    @abstractmethod
    def get_discrete_trajectory(
        self, poll_step: SpatialMeasurement
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement]]:
        pass

    @abstractmethod
    def get_length() -> SpatialMeasurement:
        pass

    @abstractmethod
    def get_travel_time() -> TimeMeasurement:
        pass

    @abstractmethod
    def get(percentage: float) -> TrajectoryState:
        pass

    @abstractmethod
    def at_time(time: TimeMeasurement) -> TrajectoryState:
        pass


class SwerveTrajectory(
    Trajectory
):  # TODO: Return SwerveTrajectoryState with additional module state info
    pass


class InterpolatedSwerveTrajectory(SwerveTrajectory):
    def __init__(self, states: List[TrajectoryState]) -> None:
        super().__init__()
        self.discrete_states = states

    def get_discrete_trajectory(
        self, poll_step: SpatialMeasurement
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement]]:
        pass

    def get_length() -> SpatialMeasurement:
        pass

    def get_travel_time() -> TimeMeasurement:
        pass

    def get(percentage: float) -> TrajectoryState:
        pass

    def at_time(time: TimeMeasurement) -> TrajectoryState:
        pass

    def draw(self, render_scale: SpatialMeasurement):
        for i in range(len(self.discrete_states) - 1):
            point = self.discrete_states[i]
            point2 = self.discrete_states[i + 1]

            helpers.draw_line(
                point.x,
                point.y,
                point2.x,
                point2.y,
                Inch(1.5),
                Palette.PINK,
                render_scale,
            )
            helpers.draw_point(point.x, point.y, Inch(1.5), Palette.PINK, render_scale)
        helpers.draw_point(point2.x, point2.y, Inch(2), Palette.PINK, render_scale)
        helpers.draw_point(
            self.discrete_states[0].x,
            self.discrete_states[0].y,
            Inch(2),
            Palette.PINK,
            render_scale,
        )
