from dataclasses import dataclass

from gamegine.utils.NCIM.ComplexDimensions.electricpot import ElectricPot, Volt
from gamegine.utils.NCIM.ComplexDimensions.omega import Omega, RotationsPerMinute
from gamegine.utils.NCIM.ComplexDimensions.torque import NewtonMeter, Torque
from gamegine.utils.NCIM.Dimensions.current import Ampere, CurrentMeasurement
from gamegine.utils.NCIM.Dimensions.mass import MassMeasurement, Pound
from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit


@dataclass
class MotorSpecification:
    """Stores the specifications of a motor, including weight, free speed, stall torque, stall current, free current, kT, kV, and max voltage.

    :param weight: The weight of the motor.
    :type weight: :class:`MassMeasurement`
    :param free_speed: The free speed of the motor.
    :type free_speed: :class:`Omega`
    :param stall_torque: The stall torque of the motor.
    :type stall_torque: :class:`Torque`
    :param stall_current: The stall current of the motor.
    :type stall_current: :class:`CurrentMeasurement`
    :param free_current: The free current of the motor.
    :type free_current: :class:`CurrentMeasurement`
    :param kT: The torque constant of the motor.
    :type kT: :class:`ComplexMeasurement`
    :param kV: The velocity constant of the motor.
    :type kV: :class:`ComplexMeasurement`
    :param maxVoltage: The maximum voltage of the motor.
    :type maxVoltage: :class:`ElectricPot`
    """

    weight: MassMeasurement
    free_speed: Omega
    stall_torque: Torque
    stall_current: CurrentMeasurement
    free_current: CurrentMeasurement
    kT: ComplexMeasurement
    kV: ComplexMeasurement
    maxVoltage: ElectricPot = Volt(12)

    def get_torque_at(
        self,
        speed: Omega,
        max_current: CurrentMeasurement,
        max_stator_current: CurrentMeasurement = Ampere(0),
    ) -> Torque:
        """Calculates the torque output of the motor at a given speed, with a given current limit.

        :param speed: The speed at which to calculate the torque.
        :type speed: :class:`Omega`
        :param max_current: The maximum current limit of the motor.
        :type max_current: :class:`CurrentMeasurement`
        :param max_stator_current: The maximum stator current limit of the motor. Defaults to Ampere(0).
        :type max_stator_current: :class:`CurrentMeasurement`, optional
        :return: The torque output of the motor at the given speed.
        :rtype: :class:`Torque`"""
        # Kinda a cooked implementation TODO: Verify, Fix, Refactor Plz
        if speed > self.free_speed:
            speed = self.free_speed

        if speed == 0:
            stator = self.stall_current
        else:
            voltage = speed / self.kV
            stator = self.maxVoltage / voltage * max_current

        if stator > max_stator_current and max_stator_current != Ampere(0):
            stator = max_stator_current
        max_torque_possible = self.kT * stator
        torque = self.stall_torque * (1 - speed / self.free_speed)
        if torque > max_torque_possible:
            torque = max_torque_possible

        return torque


@dataclass
class PowerConfig:
    """Stores the power configuration of a motor, including supply current limit, stator current limit, and max power percentage.

    :param supply_current_limit: The supply current limit of the motor.
    :type supply_current_limit: :class:`CurrentMeasurement`
    :param stator_current_limit: The stator current limit of the motor.
    :type stator_current_limit: :class:`CurrentMeasurement`
    :param max_power_percentage: The maximum power percentage of the motor. Defaults to 1.0.
    :type max_power_percentage: float, optional
    """

    supply_current_limit: CurrentMeasurement = Ampere(40)
    stator_current_limit: CurrentMeasurement = Ampere(0)
    max_power_percentage: float = 1.0


@dataclass
class MotorConfig:
    """Stores the configuration of a motor, including motor specification and power configuration.

    :param motor: The motor specification of the motor.
    :type motor: :class:`MotorSpecification`
    :param power: The power configuration of the motor.
    :type power: :class:`PowerConfig`
    """

    motor: MotorSpecification
    power: PowerConfig

    def get_torque_at(self, speed: Omega) -> Torque:
        """Calculates the torque output of the motor at a given speed.

        :param speed: The speed at which to calculate the torque.
        :type speed: :class:`Omega`
        :return: The torque output of the motor at the given speed.
        :rtype: :class:`Torque`"""
        return self.motor.get_torque_at(
            speed, self.power.supply_current_limit, self.power.stator_current_limit
        )

    def get_max_speed(self) -> Omega:
        """Returns the maximum speed of the motor.

        :return: The maximum speed of the motor.
        :rtype: :class:`Omega`"""
        return self.motor.free_speed

    def get_max_torque(self) -> Torque:
        """Returns the maximum torque of the motor.

        :return: The maximum torque of the motor.
        :rtype: :class:`Torque`"""
        return self.motor.stall_torque


KrakenX60 = MotorSpecification(
    Pound(1.20),
    RotationsPerMinute(6000),
    NewtonMeter(7.09),
    Ampere(366),
    Ampere(1.5),
    NewtonMeter(0.0194) / Ampere(1),
    RotationsPerMinute(502.1) / Volt(1),
)

NEO = MotorSpecification(
    Pound(1.19),
    RotationsPerMinute(5880),
    NewtonMeter(3.28),
    Ampere(181),
    Ampere(1.3),
    NewtonMeter(0.0181) / Ampere(1),
    RotationsPerMinute(493.5) / Volt(1),
)

Falcon500 = MotorSpecification(
    Pound(1.10),
    RotationsPerMinute(6380),
    NewtonMeter(4.69),
    Ampere(257),
    Ampere(1.5),
    NewtonMeter(0.0182) / Ampere(1),
    RotationsPerMinute(534.8) / Volt(1),
)

Falcon500_FOC = MotorSpecification(
    Pound(1.10),
    RotationsPerMinute(6080),
    NewtonMeter(5.84),
    Ampere(304),
    Ampere(1.5),
    NewtonMeter(0.0192) / Ampere(1),
    RotationsPerMinute(509.2) / Volt(1),
)

KrakenX60_FOC = MotorSpecification(
    Pound(1.20),
    RotationsPerMinute(5800),
    NewtonMeter(9.37),
    Ampere(483),
    Ampere(1.5),
    NewtonMeter(0.0194) / Ampere(1),
    RotationsPerMinute(484.8) / Volt(1),
)

NEO_Vortex = MotorSpecification(
    Pound(1.28),
    RotationsPerMinute(6784),
    NewtonMeter(3.60),
    Ampere(211),
    Ampere(3.6),
    NewtonMeter(0.0171) / Ampere(1),
    RotationsPerMinute(575.1) / Volt(1),
)

NEO_550 = MotorSpecification(
    Pound(0.56),
    RotationsPerMinute(11710),
    NewtonMeter(1.08),
    Ampere(111),
    Ampere(1.1),
    NewtonMeter(0.0097) / Ampere(1),
    RotationsPerMinute(985.6) / Volt(1),
)

Pro_775 = MotorSpecification(
    Pound(1.05),
    RotationsPerMinute(18730),
    NewtonMeter(0.71),
    Ampere(134),
    Ampere(0.7),
    NewtonMeter(0.0053) / Ampere(1),
    RotationsPerMinute(1569.0) / Volt(1),
)

RedLine_775 = MotorSpecification(
    Pound(1.06),
    RotationsPerMinute(19500),
    NewtonMeter(0.64),
    Ampere(122),
    Ampere(2.6),
    NewtonMeter(0.0052) / Ampere(1),
    RotationsPerMinute(1660.4) / Volt(1),
)

CIM = MotorSpecification(
    Pound(3.07),
    RotationsPerMinute(5330),
    NewtonMeter(2.41),
    Ampere(131),
    Ampere(2.7),
    NewtonMeter(0.0184) / Ampere(1),
    RotationsPerMinute(453.5) / Volt(1),
)

MiniCIM = MotorSpecification(
    Pound(2.41),
    RotationsPerMinute(5840),
    NewtonMeter(1.41),
    Ampere(89),
    Ampere(3.0),
    NewtonMeter(0.0158) / Ampere(1),
    RotationsPerMinute(503.6) / Volt(1),
)

BAG = MotorSpecification(
    Pound(0.96),
    RotationsPerMinute(13180),
    NewtonMeter(0.43),
    Ampere(53),
    Ampere(1.8),
    NewtonMeter(0.0081) / Ampere(1),
    RotationsPerMinute(1136.9) / Volt(1),
)

AM_9015 = MotorSpecification(
    Pound(0.75),
    RotationsPerMinute(14270),
    NewtonMeter(0.36),
    Ampere(71),
    Ampere(3.7),
    NewtonMeter(0.0051) / Ampere(1),
    RotationsPerMinute(1254.5) / Volt(1),
)

BaneBots_550 = MotorSpecification(
    Pound(0.73),
    RotationsPerMinute(19300),
    NewtonMeter(0.49),
    Ampere(85),
    Ampere(1.4),
    NewtonMeter(0.0057) / Ampere(1),
    RotationsPerMinute(1635.3) / Volt(1),
)

NeveRest = MotorSpecification(
    Pound(0.84),
    RotationsPerMinute(5480),
    NewtonMeter(0.17),
    Ampere(10),
    Ampere(0.4),
    NewtonMeter(0.0177) / Ampere(1),
    RotationsPerMinute(473.8) / Volt(1),
)

Snowblower = MotorSpecification(
    Pound(1.35),
    RotationsPerMinute(100),
    NewtonMeter(7.91),
    Ampere(24),
    Ampere(5.0),
    NewtonMeter(0.3295) / Ampere(1),
    RotationsPerMinute(10.5) / Volt(1),
)

HD_Hex = MotorSpecification(
    Pound(0.77),
    RotationsPerMinute(6000),
    NewtonMeter(0.10),
    Ampere(9),
    Ampere(0.4),
    NewtonMeter(0.0124) / Ampere(1),
    RotationsPerMinute(524.7) / Volt(1),
)

Core_Hex = MotorSpecification(
    Pound(0.69),
    RotationsPerMinute(125),
    NewtonMeter(3.20),
    Ampere(4),
    Ampere(0.5),
    NewtonMeter(0.7273) / Ampere(1),
    RotationsPerMinute(11.8) / Volt(1),
)

V5_Smart_Motor_Red = MotorSpecification(
    Pound(0.34),
    RotationsPerMinute(100),
    NewtonMeter(2.10),
    Ampere(3),
    Ampere(0.9),
    NewtonMeter(0.8400) / Ampere(1),
    RotationsPerMinute(13.0) / Volt(1),
)
