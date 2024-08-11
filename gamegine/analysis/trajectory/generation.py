from abc import ABC, abstractmethod
from typing import List, Tuple
from dataclasses import dataclass
import pint
import pygame

from gamegine.analysis.pathfinding import Path
from gamegine.render import helpers
from gamegine.render.drawable import Drawable
from gamegine.render.style import Palette
from gamegine.representation.bounds import Circle, DiscreteBoundary
from gamegine.representation.robot import (
    PhysicalParameters,
    SwerveDrivetrainCharacteristics,
)
from gamegine.utils.logging import Debug
from gamegine.utils.NCIM.ncim import (
    MOI,
    AngularMeasurement,
    ForceMeasurement,
    Inch,
    MassMeasurement,
    MetersPerSecond,
    Omega,
    RadiansPerSecond,
    RatioOf,
    SpatialMeasurement,
    TemporalMeasurement,
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

    def get_velocity_magnitude(self) -> Velocity:
        return (self.vel_x**2 + self.vel_y**2) ** 0.5


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
    def get_travel_time() -> TemporalMeasurement:
        pass

    @abstractmethod
    def get(percentage: float) -> TrajectoryState:
        pass

    @abstractmethod
    def at_time(time: TemporalMeasurement) -> TrajectoryState:
        pass


class SwerveTrajectory(
    Trajectory
):  # TODO: Return SwerveTrajectoryState with additional module state info
    pass


class InterpolatedSwerveTrajectory(SwerveTrajectory):

    def __init__(self, states: List[TrajectoryState]) -> None:
        super().__init__()
        self.discrete_states = states
        self.__calc_maximums()

    def __calc_maximums(self):
        self.MAX_VEL = 0
        for state in self.discrete_states:
            vel = state.get_velocity_magnitude()
            if vel > self.MAX_VEL:
                self.MAX_VEL = vel

    def get_discrete_trajectory(
        self, poll_step: SpatialMeasurement
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement]]:
        pass

    def get_length() -> SpatialMeasurement:
        pass

    def get_travel_time() -> TemporalMeasurement:
        pass

    def get(percentage: float) -> TrajectoryState:
        pass

    def at_time(time: TemporalMeasurement) -> TrajectoryState:
        pass

    def draw(self, render_scale: SpatialMeasurement):
        for i in range(len(self.discrete_states) - 1):
            point = self.discrete_states[i]
            point2 = self.discrete_states[i + 1]
            vel = point.get_velocity_magnitude()
            vel_ratio = float(vel / self.MAX_VEL)
            pygame.draw.line(
                pygame.display.get_surface(),
                (int(vel_ratio * 255), 100, 0),
                (RatioOf(point.x, render_scale), RatioOf(point.y, render_scale)),
                (RatioOf(point2.x, render_scale), RatioOf(point2.y, render_scale)),
                width=int(RatioOf(Inch(1) + Inch(1) * vel_ratio, render_scale)),
            )

        helpers.draw_point(point2.x, point2.y, Inch(2), Palette.PINK, render_scale)
        helpers.draw_point(
            self.discrete_states[0].x,
            self.discrete_states[0].y,
            Inch(2),
            Palette.PINK,
            render_scale,
        )

    def get_points(self):
        return self.discrete_states
