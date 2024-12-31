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

    def get_torque(self, speed: Omega) -> Torque:
        """Calculates the torque output of the drive motor at a given speed."""
        return (
            self.drive_motor.motor.get_torque_at(
                speed, self.drive_motor.power.supply_current_limit
            )
            * self.drive_gear_ratio
        )

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
