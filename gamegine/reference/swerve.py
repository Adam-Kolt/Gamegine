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
    """Dataclass used to store the configuration of a swerve module."""

    drive_motor: MotorConfig
    drive_gear_ratio: GearSeries
    steer_motor: MotorConfig
    steer_gear_ratio: GearSeries
    wheel: Wheel = field(
        default_factory=lambda: Wheel(Inch(4), TreadDB.vexPro_versawheel)
    )

    def get_torque(self, speed: Omega) -> Torque:
        return (
            self.drive_motor.motor.get_torque_at(
                speed, self.drive_motor.power.supply_current_limit
            )
            * self.drive_gear_ratio
        )

    def get_max_torque(self) -> Torque:
        return self.drive_motor.motor.stall_torque * self.drive_gear_ratio.get_ratio()

    def get_max_speed(self) -> Omega:
        return self.drive_motor.motor.free_speed / self.drive_gear_ratio.get_ratio()


@dataclass
class SwerveConfig:
    """Dataclass used to store the configuration of a swerve drive."""

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
