from abc import ABC, abstractmethod
from typing import List, Tuple
from dataclasses import dataclass
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


"""
Class for storing data about individual waypoints in a trajectory, including position, velocity, and the angular counterpart of each. It describes the states the robot should reach sequentially in a trajectory.
"""


@dataclass
class TrajectoryKeypoint:
    x: SpatialMeasurement = None
    y: SpatialMeasurement = None
    theta: AngularMeasurement = None
    velocity_x: Velocity = MetersPerSecond(0.0)
    velocity_y: Velocity = MetersPerSecond(0.0)
    omega: Omega = RadiansPerSecond(0.0)


"""
Abstract class for a trajectory generator which bases output on an initial guide path, usually generated from running a pathfinding algorithm, obstacles, and start and end parameters.
"""


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


"""
Abstract class for a trajectory generator which bases output on an initial guide path, usually generated from running a pathfinding algorithm, obstacles, and start and end parameters. This class is specifically for swerve drivetrains.
"""


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


"""Class for storing data about each discrete point in a trajectory, including position, velocity, and acceleration of a robot at that point.
"""


@dataclass
class TrajectoryState:
    x: SpatialMeasurement
    y: SpatialMeasurement
    angle: AngularMeasurement
    vel_x: Velocity = MetersPerSecond(0.0)
    vel_y: Velocity = MetersPerSecond(0.0)
    acc_x: Velocity = MetersPerSecond(0.0)
    acc_y: Velocity = MetersPerSecond(0.0)

    """Returns the magnitude of the velocity vector at the current state.
    
    :return: The magnitude of the velocity vector at the current state.
    :rtype: :class:`Velocity`
    """

    def get_velocity_magnitude(self) -> Velocity:
        return (self.vel_x**2 + self.vel_y**2) ** 0.5


"""Abstract class for a trajectory, which is defined as a series of states which the robot travels through. Includes methods for getting the length and travel time of the trajectory.
"""


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


"""
Class for storing data about each discrete point in a trajectory, including position, velocity, and acceleration of a robot at that point. This class is specifically for swerve drivetrains.
"""


class SwerveTrajectory(Trajectory):
    pass


"""
Stores individual states of a swerve trajectory, including position, velocity, and acceleration of a robot at that point.
"""


class SwerveTrajectoryState(TrajectoryState):
    pass


class SwerveTrajectory(SwerveTrajectory):

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
