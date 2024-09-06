from dataclasses import dataclass

from gamegine.utils.NCIM.ComplexDimensions.electricpot import Volt
from gamegine.utils.NCIM.ComplexDimensions.omega import Omega, RotationsPerSecond
from gamegine.utils.NCIM.ComplexDimensions.torque import Torque
from gamegine.utils.NCIM.Dimensions.current import Ampere, CurrentMeasurement
from gamegine.utils.NCIM.Dimensions.mass import MassMeasurement, Pound
from gamegine.utils.NCIM.basic import ComplexMeasurement


@dataclass
class MotorSpecification:
    weight: MassMeasurement
    free_speed: Omega
    stall_torque: Torque
    stall_current: CurrentMeasurement
    free_current: CurrentMeasurement
    kT: ComplexMeasurement
    kV: ComplexMeasurement


@dataclass
class PowerConfig:
    supply_current_limit: CurrentMeasurement = Ampere(40)
    stator_current_limit: CurrentMeasurement = Ampere(0)
    max_power_percentage: float = 1.0


@dataclass
class MotorConfig:
    motor: MotorSpecification
    power: PowerConfig


KrakenX60 = MotorSpecification(
    Pound(1.20),
    RotationsPerSecond(6000),
    Torque(7.09),
    Ampere(366),
    Ampere(1.5),
    Torque(0.0194) / Ampere(1),
    RotationsPerSecond(502.1) / Volt(1),
)

NEO = MotorSpecification(
    Pound(1.19),
    RotationsPerSecond(5880),
    Torque(3.28),
    Ampere(181),
    Ampere(1.3),
    Torque(0.0181) / Ampere(1),
    RotationsPerSecond(493.5) / Volt(1),
)

Falcon500 = MotorSpecification(
    Pound(1.10),
    RotationsPerSecond(6380),
    Torque(4.69),
    Ampere(257),
    Ampere(1.5),
    Torque(0.0182) / Ampere(1),
    RotationsPerSecond(534.8) / Volt(1),
)

Falcon500_FOC = MotorSpecification(
    Pound(1.10),
    RotationsPerSecond(6080),
    Torque(5.84),
    Ampere(304),
    Ampere(1.5),
    Torque(0.0192) / Ampere(1),
    RotationsPerSecond(509.2) / Volt(1),
)

KrakenX60_FOC = MotorSpecification(
    Pound(1.20),
    RotationsPerSecond(5800),
    Torque(9.37),
    Ampere(483),
    Ampere(1.5),
    Torque(0.0194) / Ampere(1),
    RotationsPerSecond(484.8) / Volt(1),
)

NEO_Vortex = MotorSpecification(
    Pound(1.28),
    RotationsPerSecond(6784),
    Torque(3.60),
    Ampere(211),
    Ampere(3.6),
    Torque(0.0171) / Ampere(1),
    RotationsPerSecond(575.1) / Volt(1),
)

NEO_550 = MotorSpecification(
    Pound(0.56),
    RotationsPerSecond(11710),
    Torque(1.08),
    Ampere(111),
    Ampere(1.1),
    Torque(0.0097) / Ampere(1),
    RotationsPerSecond(985.6) / Volt(1),
)

Pro_775 = MotorSpecification(
    Pound(1.05),
    RotationsPerSecond(18730),
    Torque(0.71),
    Ampere(134),
    Ampere(0.7),
    Torque(0.0053) / Ampere(1),
    RotationsPerSecond(1569.0) / Volt(1),
)

RedLine_775 = MotorSpecification(
    Pound(1.06),
    RotationsPerSecond(19500),
    Torque(0.64),
    Ampere(122),
    Ampere(2.6),
    Torque(0.0052) / Ampere(1),
    RotationsPerSecond(1660.4) / Volt(1),
)

CIM = MotorSpecification(
    Pound(3.07),
    RotationsPerSecond(5330),
    Torque(2.41),
    Ampere(131),
    Ampere(2.7),
    Torque(0.0184) / Ampere(1),
    RotationsPerSecond(453.5) / Volt(1),
)

MiniCIM = MotorSpecification(
    Pound(2.41),
    RotationsPerSecond(5840),
    Torque(1.41),
    Ampere(89),
    Ampere(3.0),
    Torque(0.0158) / Ampere(1),
    RotationsPerSecond(503.6) / Volt(1),
)

BAG = MotorSpecification(
    Pound(0.96),
    RotationsPerSecond(13180),
    Torque(0.43),
    Ampere(53),
    Ampere(1.8),
    Torque(0.0081) / Ampere(1),
    RotationsPerSecond(1136.9) / Volt(1),
)

AM_9015 = MotorSpecification(
    Pound(0.75),
    RotationsPerSecond(14270),
    Torque(0.36),
    Ampere(71),
    Ampere(3.7),
    Torque(0.0051) / Ampere(1),
    RotationsPerSecond(1254.5) / Volt(1),
)

BaneBots_550 = MotorSpecification(
    Pound(0.73),
    RotationsPerSecond(19300),
    Torque(0.49),
    Ampere(85),
    Ampere(1.4),
    Torque(0.0057) / Ampere(1),
    RotationsPerSecond(1635.3) / Volt(1),
)

NeveRest = MotorSpecification(
    Pound(0.84),
    RotationsPerSecond(5480),
    Torque(0.17),
    Ampere(10),
    Ampere(0.4),
    Torque(0.0177) / Ampere(1),
    RotationsPerSecond(473.8) / Volt(1),
)

Snowblower = MotorSpecification(
    Pound(1.35),
    RotationsPerSecond(100),
    Torque(7.91),
    Ampere(24),
    Ampere(5.0),
    Torque(0.3295) / Ampere(1),
    RotationsPerSecond(10.5) / Volt(1),
)

HD_Hex = MotorSpecification(
    Pound(0.77),
    RotationsPerSecond(6000),
    Torque(0.10),
    Ampere(9),
    Ampere(0.4),
    Torque(0.0124) / Ampere(1),
    RotationsPerSecond(524.7) / Volt(1),
)

Core_Hex = MotorSpecification(
    Pound(0.69),
    RotationsPerSecond(125),
    Torque(3.20),
    Ampere(4),
    Ampere(0.5),
    Torque(0.7273) / Ampere(1),
    RotationsPerSecond(11.8) / Volt(1),
)

V5_Smart_Motor_Red = MotorSpecification(
    Pound(0.34),
    RotationsPerSecond(100),
    Torque(2.10),
    Ampere(3),
    Ampere(0.9),
    Torque(0.8400) / Ampere(1),
    RotationsPerSecond(13.0) / Volt(1),
)
