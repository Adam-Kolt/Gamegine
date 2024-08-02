from enum import Enum
from abc import ABC, abstractmethod
from typing import Dict

from gamegine.utils.logging import Debug, Error, Warn
import numpy as np

# Unit System V2...Still flawed, yet less so


class Unit(object):
    ROUNDING = 9

    def __init__(
        self, dimension: "Dimension", scale: float = 0, symbol="", shift: float = 0
    ) -> None:
        self.scale = scale
        self.shift = shift
        self.symbol = symbol
        self.dimension = dimension

    def to_base(self, value: float) -> float:
        return round(float(value) * self.scale + self.shift, Unit.ROUNDING)

    def from_base(self, value: float) -> float:
        return round(float(value) / self.scale - self.shift, Unit.ROUNDING)

    def get_dimension(self) -> "Dimension":
        return self.dimension

    def get_symbol(self) -> str:
        return self.symbol

    def get_scale(self):
        return self.scale


class Dimension(Enum):
    Spatial = 0
    Mass = 1
    Force = 2
    Energy = 3
    Temporal = 4


class SpatialUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Spatial, scale, display, shift)


class TimeUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Temporal, scale, display, shift)


class MassUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Mass, scale, display, shift)


class ForceUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Force, scale, display, shift)


class EnergyUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Energy, scale, display, shift)


class AngularUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Energy, scale, display, shift)


class ComplexUnit(Unit):
    ALIASES = {}  # Possible future speedup if necessary

    def __init__(self, units: Dict[Unit, int]) -> None:
        dimensionality = np.zeros(len(Dimension))
        self.units: Dict[int, Unit] = {}
        for unit, power in units.items():
            dimension = unit.get_dimension()
            index = dimension.value
            dimensionality[index] += power

            if not index in self.units:
                self.units[index] = unit
        self.dimension = dimensionality

    def __eq__(self, value: object) -> bool:
        if isinstance(value, ComplexUnit):
            return (value.dimension == self.dimension).all()
        return False

    def __merge_units(self, other: "ComplexUnit"):
        for dimension, unit in other.units.items():
            if not dimension in self.units:
                self.units[dimension] = unit

    def __mul__(self, other):
        if not isinstance(other, ComplexUnit):
            return
        self.dimension += other.dimension
        self.__merge_units(other)
        return self

    def __truediv__(self, other):
        if not isinstance(other, ComplexUnit):
            return
        self.dimension -= other.dimension
        self.__merge_units(other)
        return self

    def to_base(
        self, value: float
    ) -> float:  # TODO: Account for shift, not just scale, in units
        value = float(value)
        for i, pwr in enumerate(self.dimension):
            if pwr == 0:
                continue
            value *= self.units[i].get_scale() ** pwr
        return value

    def from_base(self, value: float) -> float:
        value = float(value)
        for i, pwr in enumerate(self.dimension):
            if pwr == 0:
                continue
            value /= self.units[i].get_scale() ** pwr
        return value

    def get_symbol(self) -> str:
        num = []
        den = []
        for i, pwr in enumerate(self.dimension):
            if pwr == 0:
                continue
            symbol = self.units[i].get_symbol()
            if pwr > 0:
                if pwr == 1:
                    num.append(symbol)
                else:
                    num.append(f"{symbol}^{pwr}")
            else:
                if pwr == -1:
                    den.append(symbol)
                else:
                    den.append(f"{symbol}^{abs(pwr)}")
        top = " * ".join(num)
        # TODO: make this cleaner
        if num == [] and den == []:
            return ""
        if num == []:
            top = "1"
        if den == []:
            return top
        bottom = " * ".join(den)
        return f"{top}/{bottom}"

    def get_dimension(self):
        return self.dimension


class AccelerationUnit(ComplexUnit):
    def __init__(self, spatial: SpatialUnit, time: TimeUnit) -> None:
        super().__init__({spatial: 1, time: -2})


class VelocityUnit(ComplexUnit):
    def __init__(self, spatial: SpatialUnit, time: TimeUnit) -> None:
        super().__init__({spatial: 1, time: -1})


class OmegaUnit(ComplexUnit):
    def __init__(self, angles: AngularUnit, time: TimeUnit) -> None:
        super().__init__({angles: 1, time: -1})


class AlphaUnit(ComplexUnit):
    def __init__(self, angles: AngularUnit, time: TimeUnit) -> None:
        super().__init__({angles: 1, time: -2})


class TorqueUnit(ComplexUnit):
    def __init__(self, spatial: SpatialUnit, force: ForceUnit) -> None:
        super().__init__({spatial: 1, force: 1})


class MOIUnit(ComplexUnit):
    def __init__(self, mass: MassUnit, spatial: SpatialUnit) -> None:
        super().__init__({spatial: 1, mass: 2})


class Units:
    class Spatial:
        Meter = SpatialUnit(1, "m")
        Centimeter = SpatialUnit(0.01, "cm")
        Feet = SpatialUnit(0.3048, "ft")
        Inch = SpatialUnit(0.0254, "in")
        Yard = SpatialUnit(0.9144, "yd")
        Kilometer = SpatialUnit(1000, "km")
        Mile = SpatialUnit(1609.34, "mi")
        NauticalMile = SpatialUnit(1852, "nmi")
        MicroMeter = SpatialUnit(0.000001, "μm")

    class Time:
        Second = TimeUnit(1, "s")
        Minute = TimeUnit(60, "min")
        Hour = TimeUnit(3600, "h")
        Day = TimeUnit(86400, "d")

    class Mass:
        Gram = MassUnit(1, "g")
        Kilogram = MassUnit(1000, "kg")
        Milligram = MassUnit(0.001, "mg")
        Pound = MassUnit(453.592, "lb")
        Ounce = MassUnit(28.3495, "oz")

    class Force:
        Newton = ForceUnit(1, "N")
        Dyne = ForceUnit(0.00001, "dyn")
        PoundForce = ForceUnit(4.44822, "lbf")
        KilogramForce = ForceUnit(9.80665, "kgf")

    class Energy:
        Joule = EnergyUnit(1, "J")
        Kilojoule = EnergyUnit(1000, "kJ")
        Calorie = EnergyUnit(4.184, "cal")
        Kilocalorie = EnergyUnit(4184, "kcal")
        WattHour = EnergyUnit(3600, "Wh")
        KilowattHour = EnergyUnit(3600000, "kWh")
        ElectronVolt = EnergyUnit(1.60218e-19, "eV")

    class Angular:
        Radian = AngularUnit(1, "rad")
        Degree = AngularUnit(0.0174533, "°")
        Grad = AngularUnit(0.01570796, "grad")


class ComplexUnits:
    class Acceleration:
        MeterPerSecondSquared = AccelerationUnit(Units.Spatial.Meter, Units.Time.Second)

    class Velocity:
        MeterPerSecond = VelocityUnit(Units.Spatial.Meter, Units.Time.Second)
        KilometerPerHour = VelocityUnit(Units.Spatial.Kilometer, Units.Time.Hour)
        MilePerHour = VelocityUnit(Units.Spatial.Mile, Units.Time.Hour)
        FootPerSecond = VelocityUnit(Units.Spatial.Feet, Units.Time.Second)

    class Torque:
        NewtonMeter = TorqueUnit(Units.Spatial.Meter, Units.Force.Newton)

    class MOI:
        KilogramMetersSquared = MOIUnit(Units.Mass.Kilogram, Units.Spatial.Meter)

    class Omega:
        RadsPerSecond = OmegaUnit(Units.Angular.Radian, Units.Time.Second)
        DegreesPerSecond = OmegaUnit(Units.Angular.Degree, Units.Time.Second)


class ComplexMeasurement(float):
    def __new__(cls, magnitude: float, unit: ComplexUnit, base_magnitude=None):
        if base_magnitude:
            return float.__new__(cls, base_magnitude)
        if isinstance(magnitude, ComplexMeasurement):
            return magnitude
        value = unit.to_base(magnitude)
        return float.__new__(cls, value)

    def __init__(
        self, magnitude: float, unit: ComplexUnit, base_magnitude=None
    ) -> None:
        super().__init__()
        self.dimension = unit.get_dimension()
        self.unit = unit

    def __deepcopy__(self, memo):
        return ComplexUnit(self, self.unit)

    def to(self, unit: ComplexUnit):
        unit.from_base(self)

    def get_unit_magnitude(self) -> float:
        return self.unit.from_base(self)

    def __str__(self) -> str:
        return f"{self.get_unit_magnitude()} {self.unit.get_symbol()}"

    def __repr__(self) -> str:
        return self.__str__()

    def get_dimension(self):
        return self.dimension

    def get_unit(self) -> ComplexUnit:
        return self.unit

    def to_complex(self):
        return self

    def __is_same_dimension(self, other: "ComplexMeasurement"):
        return other.get_unit() == self.get_unit()

    @classmethod
    def __is_a_measurement(cls, other):
        return isinstance(other, ComplexMeasurement)

    def __add__(self, other) -> float:
        if ComplexMeasurement.__is_a_measurement(other) and self.__is_same_dimension(
            other
        ):
            return ComplexMeasurement(
                0, self.unit, base_magnitude=float(self) + float(other)
            )
        else:
            Error("Cannot add measurements of different dimensions")

    def __sub__(self, other) -> float:
        if ComplexMeasurement.__is_a_measurement(other) and self.__is_same_dimension(
            other
        ):
            return ComplexMeasurement(
                0, self.unit, base_magnitude=float(self) - float(other)
            )
        else:
            Error("Cannot add measurements of different dimensions")

    def __mul__(self, other):  # Make complex units
        if isinstance(other, ComplexMeasurement):
            return ComplexMeasurement(
                0, self.unit * other.unit, base_magnitude=float(self) * float(other)
            )
        return ComplexMeasurement(
            0, other.unit, base_magnitude=float(self) * float(other)
        )

    def __truediv__(self, other):
        if isinstance(other, Measurement):
            return float(self) / float(other)
        return Measurement(0, self.unit, base_magnitude=float(self) / float(other))

    def __floordiv__(self, other):
        if isinstance(other, Measurement):
            return float(self) // float(other)
        return Measurement(0, self.unit, base_magnitude=float(self) // float(other))

    def __mod__(self, other):
        return Measurement(float(self) % float(other), self.unit)

    def __neg__(self):
        return Measurement(-float(self), self.unit)

    def __abs__(self):
        return Measurement(abs(float(self)), self.unit)

    def __pow__(self, other):  # Make into complex measurement
        if isinstance(other, Measurement):
            Error("Cannot raise measurement to power of another measurment")
        return Measurement(float(self) ** float(other), self.unit)


# TODO: Maybe later generics
class Velocity(ComplexMeasurement):
    def __new__(
        cls,
        magnitude: float,
        unit: VelocityUnit,
        base_magnitude=None,
    ):
        return super().__new__(magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: VelocityUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


class MetersPerSecond(Velocity):
    def __new__(cls, magnitude: float, base_magnitude=None):
        return super().__new__(
            magnitude, ComplexUnits.Velocity.MeterPerSecond, base_magnitude
        )


class Omega(ComplexMeasurement):
    def __new__(
        cls,
        magnitude: float,
        unit: OmegaUnit,
        base_magnitude=None,
    ):
        return super().__new__(magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: OmegaUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


class RadiansPerSecond(Omega):
    def __init__(self, magnitude: float) -> None:
        super().__init__(magnitude, ComplexUnits.Omega.RadsPerSecond)


class Acceleration(ComplexMeasurement):
    def __new__(
        cls,
        magnitude: float,
        unit: AccelerationUnit,
        base_magnitude=None,
    ):
        return super().__new__(magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: AccelerationUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


class Torque(ComplexMeasurement):
    def __new__(
        cls,
        magnitude: float,
        unit: TorqueUnit,
        base_magnitude=None,
    ):
        return super().__new__(magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: TorqueUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


class MOI(ComplexMeasurement):
    def __new__(
        cls,
        magnitude: float,
        unit: MOIUnit,
        base_magnitude=None,
    ):
        return super().__new__(magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: MOIUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


class Measurement(float):
    def __new__(cls, magnitude: float, unit: Unit, base_magnitude=None):
        if base_magnitude:
            return float.__new__(cls, base_magnitude)
        if isinstance(magnitude, Measurement):
            return magnitude
        value = unit.to_base(magnitude)
        return float.__new__(cls, value)

    def __init__(self, magnitude: float, unit: Unit, base_magnitude=None) -> None:
        super().__init__()
        self.dimension = unit.get_dimension()
        self.unit = unit

    def __deepcopy__(self, memo):
        return Measurement(0, self.unit, base_magnitude=float(self))

    def to(self, unit: Unit):
        return unit.from_base(self)

    def get_unit_magnitude(self) -> float:
        return self.unit.from_base(self)

    def __str__(self) -> str:
        return f"{self.get_unit_magnitude()} {self.unit.get_symbol()}"

    def __repr__(self) -> str:
        return self.__str__()

    def get_dimension(self) -> Dimension:
        return self.dimension

    def get_unit(self) -> Unit:
        return self.unit

    def __is_same_dimension(self, other):
        return other.get_dimension() == self.get_dimension()

    @classmethod
    def __is_a_measurement(cls, other):
        return isinstance(other, Measurement)

    def __add__(self, other) -> float:
        if isinstance(other, ComplexMeasurement):
            return self.to_complex() + other
        elif Measurement.__is_a_measurement(other) and self.__is_same_dimension(other):
            return Measurement(0, self.unit, base_magnitude=float(self) + float(other))
        else:
            Error("Cannot add measurements of different dimensions")

    def __sub__(self, other) -> float:
        if Measurement.__is_a_measurement(other) and self.__is_same_dimension(other):
            return Measurement(0, self.unit, base_magnitude=float(self) - float(other))
        else:
            Error("Cannot add measurements of different dimensions")

    def __mul__(self, other):  # Make complex units
        if isinstance(other, Measurement) or isinstance(other, ComplexMeasurement):
            return self.to_complex() * other.to_complex()
        else:
            return Measurement(0, self.unit, base_magnitude=float(self) * float(other))

    def __truediv__(self, other):
        if isinstance(other, Measurement):
            return float(self) / float(other)
        if isinstance(other, ComplexMeasurement):
            return self.to_complex() / other
        return Measurement(0, self.unit, base_magnitude=float(self) / float(other))

    def __floordiv__(self, other):
        if isinstance(other, Measurement):
            return float(self) // float(other)
        if isinstance(other, ComplexMeasurement):
            return self.to_complex() // other
        return Measurement(0, self.unit, base_magnitude=float(self) // float(other))

    def __mod__(self, other):
        return Measurement(0, self.unit, base_magnitude=float(self) % float(other))

    def __neg__(self):
        return Measurement(0, self.unit, base_magnitude=-float(self))

    def __abs__(self):
        return Measurement(0, self.unit, base_magnitude=abs(float(self)))

    def __pow__(self, other):  # Make into complex measurement
        if isinstance(other, Measurement):
            Error("Cannot raise measurement to power of another measurment")
        return self.to_complex(pwr=float(other))

    def to_complex(self, pwr: int = 1) -> ComplexMeasurement:
        return ComplexMeasurement(
            0, unit=ComplexUnit({self.get_unit(): pwr}), base_magnitude=float(self)
        )


class SpatialMeasurement(Measurement):
    def __init__(
        self, magnitude: float, unit: SpatialUnit, base_magnitude=None
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


class TimeMeasurement(Measurement):
    def __init__(self, magnitude: float, unit: TimeUnit, base_magnitude=None) -> None:
        super().__init__(magnitude, unit, base_magnitude)


class MassMeasurement(Measurement):
    def __init__(self, magnitude: float, unit: MassUnit, base_magnitude=None) -> None:
        super().__init__(magnitude, unit, base_magnitude)


class ForceMeasurement(Measurement):
    def __init__(self, magnitude: float, unit: ForceUnit, base_magnitude=None) -> None:
        super().__init__(magnitude, unit, base_magnitude)


class EnergyMeasurement(Measurement):
    def __init__(self, magnitude: float, unit: EnergyUnit, base_magnitude=None) -> None:
        super().__init__(magnitude, unit, base_magnitude)


class AngularMeasurement(Measurement):
    def __init__(
        self, magnitude: float, unit: AngularUnit, base_magnitude=None
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


# Spatial units
class Meter(SpatialMeasurement):
    def __new__(cls, magnitude: float):
        return SpatialMeasurement.__new__(cls, magnitude, Units.Spatial.Meter)

    def __init__(self, magnitude: float) -> None:
        SpatialMeasurement.__init__(self, magnitude, Units.Spatial.Meter)


class Centimeter(SpatialMeasurement):
    def __new__(cls, magnitude: float):
        return SpatialMeasurement.__new__(cls, magnitude, Units.Spatial.Centimeter)

    def __init__(self, magnitude: float) -> None:
        SpatialMeasurement.__init__(self, magnitude, Units.Spatial.Centimeter)


class Feet(SpatialMeasurement):
    def __new__(cls, magnitude: float):
        return SpatialMeasurement.__new__(cls, magnitude, Units.Spatial.Feet)

    def __init__(self, magnitude: float) -> None:
        SpatialMeasurement.__init__(self, magnitude, Units.Spatial.Feet)


class Inch(SpatialMeasurement):
    def __new__(cls, magnitude: float):
        return SpatialMeasurement.__new__(cls, magnitude, Units.Spatial.Inch)

    def __init__(self, magnitude: float) -> None:
        SpatialMeasurement.__init__(self, magnitude, Units.Spatial.Inch)


class Yard(SpatialMeasurement):
    def __new__(cls, magnitude: float):
        return SpatialMeasurement.__new__(cls, magnitude, Units.Spatial.Yard)

    def __init__(self, magnitude: float) -> None:
        SpatialMeasurement.__init__(self, magnitude, Units.Spatial.Yard)


class Kilometer(SpatialMeasurement):
    def __new__(cls, magnitude: float):
        return SpatialMeasurement.__new__(cls, magnitude, Units.Spatial.Kilometer)

    def __init__(self, magnitude: float) -> None:
        SpatialMeasurement.__init__(self, magnitude, Units.Spatial.Kilometer)


class Mile(SpatialMeasurement):
    def __new__(cls, magnitude: float):
        return SpatialMeasurement.__new__(cls, magnitude, Units.Spatial.Mile)

    def __init__(self, magnitude: float) -> None:
        SpatialMeasurement.__init__(self, magnitude, Units.Spatial.Mile)


class NauticalMile(SpatialMeasurement):
    def __new__(cls, magnitude: float):
        return SpatialMeasurement.__new__(cls, magnitude, Units.Spatial.NauticalMile)

    def __init__(self, magnitude: float) -> None:
        SpatialMeasurement.__init__(self, magnitude, Units.Spatial.NauticalMile)


class MicroMeter(SpatialMeasurement):
    def __new__(cls, magnitude: float):
        return SpatialMeasurement.__new__(cls, magnitude, Units.Spatial.MicroMeter)

    def __init__(self, magnitude: float) -> None:
        SpatialMeasurement.__init__(self, magnitude, Units.Spatial.MicroMeter)


# Time units
class Second(TimeMeasurement):
    def __new__(cls, magnitude: float):
        return TimeMeasurement.__new__(cls, magnitude, Units.Time.Second)

    def __init__(self, magnitude: float) -> None:
        TimeMeasurement.__init__(self, magnitude, Units.Time.Second)


class Minute(TimeMeasurement):
    def __new__(cls, magnitude: float):
        return TimeMeasurement.__new__(cls, magnitude, Units.Time.Minute)

    def __init__(self, magnitude: float) -> None:
        TimeMeasurement.__init__(self, magnitude, Units.Time.Minute)


class Hour(TimeMeasurement):
    def __new__(cls, magnitude: float):
        return TimeMeasurement.__new__(cls, magnitude, Units.Time.Hour)

    def __init__(self, magnitude: float) -> None:
        TimeMeasurement.__init__(self, magnitude, Units.Time.Hour)


class Day(TimeMeasurement):
    def __new__(cls, magnitude: float):
        return TimeMeasurement.__new__(cls, magnitude, Units.Time.Day)

    def __init__(self, magnitude: float) -> None:
        TimeMeasurement.__init__(self, magnitude, Units.Time.Day)


# Mass units
class Gram(MassMeasurement):
    def __new__(cls, magnitude: float):
        return MassMeasurement.__new__(cls, magnitude, Units.Mass.Gram)

    def __init__(self, magnitude: float) -> None:
        MassMeasurement.__init__(self, magnitude, Units.Mass.Gram)


class Kilogram(MassMeasurement):
    def __new__(cls, magnitude: float):
        return MassMeasurement.__new__(cls, magnitude, Units.Mass.Kilogram)

    def __init__(self, magnitude: float) -> None:
        MassMeasurement.__init__(self, magnitude, Units.Mass.Kilogram)


class Milligram(MassMeasurement):
    def __new__(cls, magnitude: float):
        return MassMeasurement.__new__(cls, magnitude, Units.Mass.Milligram)

    def __init__(self, magnitude: float) -> None:
        MassMeasurement.__init__(self, magnitude, Units.Mass.Milligram)


class Pound(MassMeasurement):
    def __new__(cls, magnitude: float):
        return MassMeasurement.__new__(cls, magnitude, Units.Mass.Pound)

    def __init__(self, magnitude: float) -> None:
        MassMeasurement.__init__(self, magnitude, Units.Mass.Pound)


class Ounce(MassMeasurement):
    def __new__(cls, magnitude: float):
        return MassMeasurement.__new__(cls, magnitude, Units.Mass.Ounce)

    def __init__(self, magnitude: float) -> None:
        MassMeasurement.__init__(self, magnitude, Units.Mass.Ounce)


# Force units
class Newton(ForceMeasurement):
    def __new__(cls, magnitude: float):
        return ForceMeasurement.__new__(cls, magnitude, Units.Force.Newton)

    def __init__(self, magnitude: float) -> None:
        ForceMeasurement.__init__(self, magnitude, Units.Force.Newton)


class Dyne(ForceMeasurement):
    def __new__(cls, magnitude: float):
        return ForceMeasurement.__new__(cls, magnitude, Units.Force.Dyne)

    def __init__(self, magnitude: float) -> None:
        ForceMeasurement.__init__(self, magnitude, Units.Force.Dyne)


class PoundForce(ForceMeasurement):
    def __new__(cls, magnitude: float):
        return ForceMeasurement.__new__(cls, magnitude, Units.Force.PoundForce)

    def __init__(self, magnitude: float) -> None:
        ForceMeasurement.__init__(self, magnitude, Units.Force.PoundForce)


class KilogramForce(ForceMeasurement):
    def __new__(cls, magnitude: float):
        return ForceMeasurement.__new__(cls, magnitude, Units.Force.KilogramForce)

    def __init__(self, magnitude: float) -> None:
        ForceMeasurement.__init__(self, magnitude, Units.Force.KilogramForce)


# Energy units
class Joule(EnergyMeasurement):
    def __new__(cls, magnitude: float):
        return EnergyMeasurement.__new__(cls, magnitude, Units.Energy.Joule)

    def __init__(self, magnitude: float) -> None:
        EnergyMeasurement.__init__(self, magnitude, Units.Energy.Joule)


class Kilojoule(EnergyMeasurement):
    def __new__(cls, magnitude: float):
        return EnergyMeasurement.__new__(cls, magnitude, Units.Energy.Kilojoule)

    def __init__(self, magnitude: float) -> None:
        EnergyMeasurement.__init__(self, magnitude, Units.Energy.Kilojoule)


class Calorie(EnergyMeasurement):
    def __new__(cls, magnitude: float):
        return EnergyMeasurement.__new__(cls, magnitude, Units.Energy.Calorie)

    def __init__(self, magnitude: float) -> None:
        EnergyMeasurement.__init__(self, magnitude, Units.Energy.Calorie)


class Kilocalorie(EnergyMeasurement):
    def __new__(cls, magnitude: float):
        return EnergyMeasurement.__new__(cls, magnitude, Units.Energy.Kilocalorie)

    def __init__(self, magnitude: float) -> None:
        EnergyMeasurement.__init__(self, magnitude, Units.Energy.Kilocalorie)


class WattHour(EnergyMeasurement):
    def __new__(cls, magnitude: float):
        return EnergyMeasurement.__new__(cls, magnitude, Units.Energy.WattHour)

    def __init__(self, magnitude: float) -> None:
        EnergyMeasurement.__init__(self, magnitude, Units.Energy.WattHour)


class KilowattHour(EnergyMeasurement):
    def __new__(cls, magnitude: float):
        return EnergyMeasurement.__new__(cls, magnitude, Units.Energy.KilowattHour)

    def __init__(self, magnitude: float) -> None:
        EnergyMeasurement.__init__(self, magnitude, Units.Energy.KilowattHour)


class ElectronVolt(EnergyMeasurement):
    def __new__(cls, magnitude: float):
        return EnergyMeasurement.__new__(cls, magnitude, Units.Energy.ElectronVolt)

    def __init__(self, magnitude: float) -> None:
        EnergyMeasurement.__init__(self, magnitude, Units.Energy.ElectronVolt)


# Angular units
class Radian(AngularMeasurement):
    def __new__(cls, magnitude: float):
        return AngularMeasurement.__new__(cls, magnitude, Units.Angular.Radian)

    def __init__(self, magnitude: float) -> None:
        AngularMeasurement.__init__(self, magnitude, Units.Angular.Radian)


class Degree(AngularMeasurement):
    def __new__(cls, magnitude: float):
        return AngularMeasurement.__new__(cls, magnitude, Units.Angular.Degree)

    def __init__(self, magnitude: float) -> None:
        AngularMeasurement.__init__(self, magnitude, Units.Angular.Degree)


class Grad(AngularMeasurement):
    def __new__(cls, magnitude: float):
        return AngularMeasurement.__new__(cls, magnitude, Units.Angular.Grad)

    def __init__(self, magnitude: float) -> None:
        AngularMeasurement.__init__(self, magnitude, Units.Angular.Grad)


def RatioOf(a, b):
    return a / b


def Zero() -> SpatialMeasurement:
    return Meter(0)
