from dataclasses import dataclass, field
from gamegine.utils.NCIM.ComplexDimensions.acceleration import Acceleration
from gamegine.utils.NCIM.ComplexDimensions.alpha import Alpha, RadiansPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.omega import Omega, RadiansPerSecond
from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond, Velocity
from gamegine.utils.NCIM.ComplexDimensions.torque import Torque, NewtonMeter
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement
from gamegine.utils.NCIM.Dimensions.temporal import Second, TemporalMeasurement


@dataclass
class TrajectoryState:
    """Stores individual points along a trajectory, including position, velocity, acceleration, and angular velocity.

    :param x: The x position of the robot.
    :type x: :class:`SpatialMeasurement`
    :param y: The y position of the robot.
    :type y: :class:`SpatialMeasurement`
    :param theta: The angle of the robot.
    :type theta: :class:`AngularMeasurement`
    :param vel_x: The x velocity of the robot.
    :type vel_x: :class:`Velocity`
    :param vel_y: The y velocity of the robot.
    :type vel_y: :class:`Velocity`
    :param acc_x: The x acceleration of the robot.
    :type acc_x: :class:`Velocity`
    :param acc_y: The y acceleration of the robot.
    :type acc_y: :class:`Velocity`
    :param omega: The angular velocity of the robot.
    :type omega: :class:`Omega`
    :param alpha: The angular acceleration of the robot.
    :type alpha: :class:`Alpha`
    """

    x: SpatialMeasurement
    y: SpatialMeasurement
    theta: AngularMeasurement
    vel_x: Velocity = MetersPerSecond(0.0)
    vel_y: Velocity = MetersPerSecond(0.0)
    acc_x: Velocity = MetersPerSecond(0.0)
    acc_y: Velocity = MetersPerSecond(0.0)
    omega: Omega = RadiansPerSecond(0.0)
    alpha: Alpha = RadiansPerSecondSquared(0.0)
    dt: TemporalMeasurement = Second(0.0)

    def get_velocity_magnitude(self) -> Velocity:
        """Returns the magnitude of the velocity vector at the current state.

        :return: The magnitude of the velocity vector at the current state.
        :rtype: :class:`Velocity`
        """
        return (self.vel_x**2 + self.vel_y**2) ** 0.5

    def get_acceleration_magnitude(self) -> Acceleration:
        """Returns the magnitude of the acceleration vector at the current state.

        :return: The magnitude of the acceleration vector at the current state.
        :rtype: :class:`Acceleration`
        """
        return (self.acc_x**2 + self.acc_y**2) ** 0.5


@dataclass
class SwerveModuleState:
    """Stores individual states of a swerve module, including wheel angle, angular velocity, and angular acceleration.

    :param wheel_angle: The angle of the wheel.
    :type wheel_angle: :class:`AngularMeasurement`
    :param wheel_omega: The angular velocity of the wheel.
    :type wheel_omega: :class:`Omega`
    :param wheel_alpha: The angular acceleration of the wheel.
    :type wheel_alpha: :class:`Alpha`
    """

    wheel_angle: AngularMeasurement
    wheel_omega: Omega
    wheel_alpha: Alpha
    motor_torque: Torque = field(default_factory=lambda: NewtonMeter(0.0))

    def get_wheel_velocity_magnitude(self) -> Velocity:
        """Returns the magnitude of the velocity vector of the wheel at the current state.

        :return: The magnitude of the velocity vector of the wheel at the current state.
        :rtype: :class:`Velocity`
        """
        return (self.wheel_omega * self.wheel_angle).abs()


@dataclass
class SwerveTrajectoryState(TrajectoryState):
    """Stores individual states of a swerve trajectory, including position, velocity, acceleration, and angular velocity of the robot, as well as the states of each swerve module.

    :param module_states: The states of each swerve module.
    :type module_states: list[:class:`SwerveModuleState`]
    """

    module_states: list[SwerveModuleState] = field(
        default_factory=lambda: [
            SwerveModuleState(0.0, 0.0, 0.0),
            SwerveModuleState(0.0, 0.0, 0.0),
            SwerveModuleState(0.0, 0.0, 0.0),
            SwerveModuleState(0.0, 0.0, 0.0),
        ]
    )

    def get_wheel_velocity_magnitude(self) -> Velocity:
        return (self.wheel_omega * self.wheel_angle).abs()
