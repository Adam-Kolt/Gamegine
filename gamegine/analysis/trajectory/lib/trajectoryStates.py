from dataclasses import dataclass
from gamegine.utils.NCIM.ComplexDimensions.alpha import Alpha, RadiansPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.omega import Omega, RadiansPerSecond
from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond, Velocity
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement


@dataclass
class TrajectoryState:
    x: SpatialMeasurement
    y: SpatialMeasurement
    theta: AngularMeasurement
    vel_x: Velocity = MetersPerSecond(0.0)
    vel_y: Velocity = MetersPerSecond(0.0)
    acc_x: Velocity = MetersPerSecond(0.0)
    acc_y: Velocity = MetersPerSecond(0.0)
    omega: Omega = RadiansPerSecond(0.0)
    alpha: Alpha = RadiansPerSecondSquared(0.0)

    def get_velocity_magnitude(self) -> Velocity:
        return (self.vel_x**2 + self.vel_y**2) ** 0.5


@dataclass
class SwerveModuleState:
    wheel_angle: AngularMeasurement
    wheel_omega: Omega
    wheel_alpha: Alpha

    def get_wheel_velocity_magnitude(self) -> Velocity:
        return (self.wheel_omega * self.wheel_angle).abs()


@dataclass
class SwerveTrajectoryState(TrajectoryState):
    module_states: list[SwerveModuleState]

    def get_wheel_velocity_magnitude(self) -> Velocity:
        return (self.wheel_omega * self.wheel_angle).abs()
