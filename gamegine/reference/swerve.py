from dataclasses import dataclass, field
from typing import Tuple

from gamegine.reference import gearing
from gamegine.reference.gearing import GearSeries
from gamegine.reference.motors import NEO, KrakenX60, MotorConfig, PowerConfig
from gamegine.reference import motors
from gamegine.reference.wheels import TreadDB, Wheel
from gamegine.utils.NCIM.ComplexDimensions.omega import Omega
from gamegine.utils.NCIM.ComplexDimensions.torque import Torque
from gamegine.utils.NCIM.Dimensions.current import Ampere
from gamegine.utils.NCIM.Dimensions.spatial import Inch, SpatialMeasurement


@dataclass
class SwerveModule:
    """Dataclass used to store the configuration of a swerve module.

    :param drive_motor: The motor configuration for the drive motor.
    :type drive_motor: :class:`MotorConfig`
    :param drive_gear_ratio: The gear ratio for the drive motor.
    :type drive_gear_ratio: :class:`GearSeries`
    :param steer_motor: The motor configuration for the steer motor.
    :type steer_motor: :class:`MotorConfig`
    :param steer_gear_ratio: The gear ratio for the steer motor.
    :type steer_gear_ratio: :class:`GearSeries`
    :param wheel: The wheel configuration for the module.
    :type wheel: :class:`Wheel`
    """

    drive_motor: MotorConfig
    drive_gear_ratio: GearSeries
    steer_motor: MotorConfig
    steer_gear_ratio: GearSeries
    wheel: Wheel = field(
        default_factory=lambda: Wheel(Inch(4), TreadDB.vexPro_versawheel)
    )

    def get_torque(self, speed: Omega, available_voltage=None) -> Torque:
        """Calculates the torque output at the wheel at a given wheel speed.
        
        The input speed is the wheel angular velocity. We must:
        1. Convert wheel speed to motor speed (motor spins faster by gear ratio)
        2. Get motor torque at that motor speed (with current and voltage limits)
        3. Convert motor torque to wheel torque (torque is multiplied by gear ratio)
        
        :param speed: Wheel angular velocity
        :param available_voltage: Battery terminal voltage (None = use nominal 12V)
        :return: Available wheel torque at this operating point
        """
        # Wheel speed -> Motor speed (motor is faster)
        motor_speed = speed * self.drive_gear_ratio.get_ratio()
        
        # Get motor torque at motor speed with current/voltage limits
        motor_torque = self.drive_motor.get_torque_at(motor_speed, available_voltage)
        
        # Motor torque -> Wheel torque (torque is amplified through gearing)
        wheel_torque = motor_torque * self.drive_gear_ratio.get_ratio()
        
        return wheel_torque

    def get_max_torque(self) -> Torque:
        """Returns the maximum torque of the drive motor."""
        return self.drive_motor.motor.stall_torque * self.drive_gear_ratio.get_ratio()

    def get_max_speed(self) -> Omega:
        """Returns the maximum speed of the drive motor."""
        return self.drive_motor.motor.free_speed / self.drive_gear_ratio.get_ratio()


@dataclass
class SwerveConfig:
    """Dataclass used to store the configuration of a swerve drive.

    :param module: The configuration of the swerve module.
    :type module: :class:`SwerveModule`
    :param top_left_offset: The offset of the top left module from the center of the robot.
    :type top_left_offset: Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]
    :param top_right_offset: The offset of the top right module from the center of the robot.
    :type top_right_offset: Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]
    :param bottom_left_offset: The offset of the bottom left module from the center of the robot.
    :type bottom_left_offset: Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]
    :param bottom_right_offset: The offset of the bottom right module from the center of the robot.
    :type bottom_right_offset: Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement
    """

    module: SwerveModule
    top_left_offset: Tuple[SpatialMeasurement, SpatialMeasurement] = (
        Inch(-13),
        Inch(13),
    )
    top_right_offset: Tuple[SpatialMeasurement, SpatialMeasurement] = (
        Inch(13),
        Inch(13),
    )
    bottom_left_offset: Tuple[SpatialMeasurement, SpatialMeasurement] = (
        Inch(-13),
        Inch(-13),
    )
    bottom_right_offset: Tuple[SpatialMeasurement, SpatialMeasurement] = (
        Inch(13),
        Inch(-13),
    )
