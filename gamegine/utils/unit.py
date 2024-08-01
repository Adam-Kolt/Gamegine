from enum import Enum
from typing import List, Tuple
import warnings
from abc import ABC, abstractmethod, ABCMeta

from gamegine.utils.logging import Error

# The manifestation of insanity in a unit system, created in a naive attempt at making stuff lightweight and easier to use...after not wanting to read the documentation of the pint library
# A reminder to not do this


class ComplexMeasurement(float):
    def __new__(
        cls,
        value: float,
        numerator: List[Tuple["MeasurementUnit", int]],
        denominator: List[Tuple["MeasurementUnit", int]],
    ):
        if isinstance(value, ComplexMeasurement):
            return value

        # Convert the value to the base unit
        for unit, power in numerator:
            base_unit = unit.wth_uses_you().get_base_unit().value
            current_unit = unit.value
            value *= base_unit**power / current_unit**power

        for unit, power in denominator:
            base_unit = unit.wth_uses_you().get_base_unit().value
            current_unit = unit.value
            value *= current_unit**power / base_unit**power

    def __init__(
        self,
        value: float,
        numerator: List[Tuple["MeasurementUnit", int]],
        denominator: List[Tuple["MeasurementUnit", int]],
    ):

        self.numerator = {}
        self.denominator = {}

        for unit, power in numerator:
            base_unit = unit.wth_uses_you().get_base_unit().name
            if base_unit in self.numerator:
                self.numerator[base_unit] += power
            else:
                self.numerator[base_unit] = power

        for unit, power in denominator:
            base_unit = unit.wth_uses_you().get_base_unit().name
            if base_unit in self.denominator:
                self.denominator[base_unit] += power
            else:
                self.denominator[base_unit] = power

    def __str__(self):
        numerator = " * ".join(
            [f"{unit}^{power}" for unit, power in self.numerator.items()]
        )
        denominator = " * ".join(
            [f"{unit}^{power}" for unit, power in self.denominator.items()]
        )
        return f"{float(self)} {numerator} / {denominator}"

    def compare_units(
        self,
        numerator: List[Tuple["MeasurementUnit", int]],
        denominator: List[Tuple["MeasurementUnit", int]],
    ):
        for unit, power in numerator:
            base_unit = unit.wth_uses_you().get_base_unit().name
            if base_unit not in self.numerator or self.numerator[base_unit] != power:
                return False

        for unit, power in denominator:
            base_unit = unit.wth_uses_you().get_base_unit().name
            if (
                base_unit not in self.denominator
                or self.denominator[base_unit] != power
            ):
                return False

        return True

    def simplify_units(self):
        for unit, power in self.numerator.items():
            if unit in self.denominator:
                pwr = power
                power -= self.denominator[unit]
                self.denominator[unit] -= pwr

            if power <= 0:
                del self.numerator[unit]

        for unit, power in self.denominator.items():
            if power <= 0:
                del self.denominator

    def __add__(self, value):
        if isinstance(value, float):  # Maybe should just error here
            warnings.warn(
                "Warning! Adding a float to a complex measurement, this is not recommended, please define the unit of the float."
            )
            return ComplexMeasurement(
                float(self) + value, self.numerator, self.denominator
            )
        elif issubclass(value.__class__, Measurement) and not issubclass(
            value.__class__, ComplexMeasurement
        ):
            value = ComplexMeasurement(value, [(value.get_base_unit(), 1)], [])

        if self.compare_units(value.numerator, value.denominator):
            return ComplexMeasurement(
                float(self) + float(value), self.numerator, self.denominator
            )
        Error("Adding two complex measurements with different units is not allowed.")
        return self

    def __sub__(self, value):
        if isinstance(value, float):
            warnings.warn(
                "Warning! Subtracting a float from a complex measurement, this is not recommended, please define the unit of the float."
            )
            return ComplexMeasurement(
                float(self) - value, self.numerator, self.denominator
            )
        elif issubclass(value.__class__, Measurement) and not issubclass(
            value.__class__, ComplexMeasurement
        ):
            value = ComplexMeasurement(value, [(value.get_base_unit(), 1)], [])

        if self.compare_units(value.numerator, value.denominator):
            return ComplexMeasurement(
                float(self) - float(value), self.numerator, self.denominator
            )
        Error(
            "Subtracting two complex measurements with different units is not allowed."
        )
        return self

    def dimensionally_analyze_units(
        self,
        numerator: List[Tuple["MeasurementUnit", int]],
        denominator: List[Tuple["MeasurementUnit", int]],
    ):
        pass  # TODO: Implement

    def __mul__(self, value):
        pass


class AbstractEnumMeta(ABCMeta, Enum.__class__):
    pass


class MeasurementUnit(Enum, metaclass=AbstractEnumMeta):
    @abstractmethod
    def wth_uses_you(self) -> "Measurement":
        pass


class Measurement(float, ABC):
    BASE_UNIT: MeasurementUnit
    MAX_DECIMALS: int

    def get_base_unit(self) -> MeasurementUnit:
        return self.BASE_UNIT

    def get_max_decimals(self) -> int:
        return self.MAX_DECIMALS


class AngularUnits(MeasurementUnit):
    Radian = 1
    Degree = 0.0174533

    def wth_uses_you(self) -> "AngularMeasurement":
        return AngularMeasurement


class AngularMeasurement(Measurement):
    BASE_UNIT = AngularUnits.Degree
    MAX_DECIMALS = 9

    def __new__(cls, value: float, unit: AngularUnits):
        if isinstance(value, AngularMeasurement):
            return value
        value = round(value, cls.MAX_DECIMALS)
        if isinstance(unit, float):
            warnings.warn(
                "UNIT WARNING! Using float as unit increases the risk that you collide (while vigorously spinning) with the Martian surface, use a specified AngularUnits Enum instead."
            )
            value = value * unit / cls.BASE_UNIT.value
        else:
            value = value * unit.value / cls.BASE_UNIT.value
        value = round(value, cls.MAX_DECIMALS)
        return float.__new__(cls, value)

    def __deepcopy__(self, memo):
        return AngularMeasurement(self, self.BASE_UNIT)

    def __str__(self):
        return f"{float(self)} {self.BASE_UNIT.name}"

    def to(self, unit: AngularUnits) -> float:
        return self * self.BASE_UNIT.value / unit.value

    def __add__(self, other):
        if isinstance(other, AngularMeasurement):
            return AngularMeasurement(float(self) + float(other), self.BASE_UNIT)
        return NotImplemented

    def __sub__(self, other):
        if isinstance(other, AngularMeasurement):
            return AngularMeasurement(float(self) - float(other), self.BASE_UNIT)
        return NotImplemented

    def __mul__(self, other):
        return AngularMeasurement(float(self) * float(other), self.BASE_UNIT)

    def __truediv__(self, other):
        if isinstance(other, AngularMeasurement):
            return float(self) / float(other)
        return AngularMeasurement(float(self) / float(other), self.BASE_UNIT)

    def __floordiv__(self, other):
        if isinstance(other, AngularMeasurement):
            return float(self) // float(other)
        return AngularMeasurement(float(self) // float(other), self.BASE_UNIT)

    def __mod__(self, other):
        return AngularMeasurement(float(self) % float(other), self.BASE_UNIT)

    def __neg__(self):
        return AngularMeasurement(-float(self), self.BASE_UNIT)

    def __abs__(self):
        return AngularMeasurement(abs(float(self)), self.BASE_UNIT)

    def __pow__(self, other):
        return AngularMeasurement(float(self) ** float(other), self.BASE_UNIT)


class Degree(AngularMeasurement):
    def __new__(cls, value: float):
        return AngularMeasurement.__new__(cls, value, AngularUnits.Degree)


class Radian(AngularMeasurement):
    def __new__(cls, value: float):
        return AngularMeasurement.__new__(cls, value, AngularUnits.Radian)


class SpatialUnits(MeasurementUnit):
    Meter = 1
    Centimeter = 0.01
    Feet = 0.3048
    Inch = 0.0254
    Yard = 0.9144
    Kilometer = 1000
    Mile = 1609.34
    NauticalMile = 1852
    MicroMeter = 0.000001

    def wth_uses_you(self) -> "SpatialMeasurement":
        return SpatialMeasurement


# TODO: You know, the naming might not be 100% great, but at least it's compatible with Apple
class SpatialMeasurement(
    float
):  # A class which ensures unit-aware initialization of spacial measurements
    BASE_UNIT = (
        SpatialUnits.MicroMeter
    )  # To avoid floating point errors, we make the base unit smal, so all units are stored as integer floats
    STRING_UNIT = SpatialUnits.Meter
    MAX_DECIMALS = (
        9  # For creation and conversion into base unit, to avoid floating point errors
    )

    def __new__(cls, value: float, unit: SpatialUnits):
        if isinstance(value, SpatialMeasurement):
            return value
        value = round(value, cls.MAX_DECIMALS)
        if isinstance(unit, float):
            warnings.warn(
                "UNIT WARNING! Using float as unit increases the risk that you collide with the Martian surface, use a specified LinearUnits Enum instead."
            )
            value = value * unit / cls.BASE_UNIT.value
        else:
            value = value * unit.value / cls.BASE_UNIT.value
        value = round(value, cls.MAX_DECIMALS)
        return float.__new__(cls, value)

    def __deepcopy__(self, memo):
        return SpatialMeasurement(self, self.BASE_UNIT)

    def to(self, unit: SpatialUnits) -> float:
        return self * self.BASE_UNIT.value / unit.value

    def __str__(self):
        converted = self.to(self.STRING_UNIT)
        return f"{float(converted)} {self.STRING_UNIT.name}"

    def __repr__(self) -> str:
        converted = self.to(self.STRING_UNIT)
        return f"{float(converted)} {self.STRING_UNIT.name}"

    # Only allow addition and subtraction of SpatialMeasurements
    def __add__(self, other):
        if isinstance(other, SpatialMeasurement):
            return SpatialMeasurement(float(self) + float(other), self.BASE_UNIT)
        return NotImplemented

    def __sub__(self, other):
        if isinstance(other, SpatialMeasurement):
            return SpatialMeasurement(float(self) - float(other), self.BASE_UNIT)
        return NotImplemented

    def __mul__(self, other):
        return SpatialMeasurement(float(self) * float(other), self.BASE_UNIT)

    def __truediv__(self, other):
        if isinstance(other, SpatialMeasurement):
            return float(self) / float(other)
        return SpatialMeasurement(float(self) / float(other), self.BASE_UNIT)

    def __floordiv__(self, other):
        if isinstance(other, SpatialMeasurement):
            return float(self) // float(other)
        return SpatialMeasurement(float(self) // float(other), self.BASE_UNIT)

    def __mod__(self, other):
        return SpatialMeasurement(float(self) % float(other), self.BASE_UNIT)

    def __neg__(self):
        return SpatialMeasurement(-float(self), self.BASE_UNIT)

    def __abs__(self):
        return SpatialMeasurement(abs(float(self)), self.BASE_UNIT)

    def __pow__(self, other):
        return SpatialMeasurement(float(self) ** float(other), self.BASE_UNIT)


class Meter(SpatialMeasurement):
    def __new__(cls, value: float):
        return SpatialMeasurement.__new__(cls, value, SpatialUnits.Meter)


class Centimeter(SpatialMeasurement):
    def __new__(cls, value: float):
        return SpatialMeasurement.__new__(cls, value, SpatialUnits.Centimeter)


class Feet(SpatialMeasurement):
    def __new__(cls, value: float):
        return SpatialMeasurement.__new__(cls, value, SpatialUnits.Feet)


class Inch(SpatialMeasurement):
    def __new__(cls, value: float):
        return SpatialMeasurement.__new__(cls, value, SpatialUnits.Inch)


class Yard(SpatialMeasurement):
    def __new__(cls, value: float):
        return SpatialMeasurement.__new__(cls, value, SpatialUnits.Yard)


class Kilometer(SpatialMeasurement):
    def __new__(cls, value: float):
        return SpatialMeasurement.__new__(cls, value, SpatialUnits.Kilometer)


class Mile(SpatialMeasurement):
    def __new__(cls, value: float):
        return SpatialMeasurement.__new__(cls, value, SpatialUnits.Mile)


class NauticalMile(SpatialMeasurement):
    def __new__(cls, value: float):
        return SpatialMeasurement.__new__(cls, value, SpatialUnits.NauticalMile)


def HalfSub(value) -> SpatialMeasurement:
    return Inch(value * 6)


def FullSub(value) -> SpatialMeasurement:
    return Feet(value)


def BigMac(value) -> SpatialMeasurement:
    return Centimeter(value * 10)


def RatioOf(a: SpatialMeasurement, b: SpatialMeasurement) -> float:
    return a / b


def Zero() -> SpatialMeasurement:
    return Meter(0)


class ForceUnits(MeasurementUnit):
    Newton = 1
    Pound = 4.44822

    def wth_uses_you(self) -> "ForceMeasurement":
        return ForceMeasurement


class ForceMeasurement(Measurement):
    BASE_UNIT = ForceUnits.Newton
    MAX_DECIMALS = 9

    def __new__(cls, value: float, unit: ForceUnits):
        if isinstance(value, ForceMeasurement):
            return value
        value = round(value, cls.MAX_DECIMALS)
        if isinstance(unit, float):
            warnings.warn(
                "UNIT WARNING! Using float as unit increases the risk that you collide with the Martian surface, use a specified ForceUnits Enum instead."
            )
            value = value * unit / cls.BASE_UNIT.value
        else:
            value = value * unit.value / cls.BASE_UNIT.value
        value = round(value, cls.MAX_DECIMALS)
        return float.__new__(cls, value)

    def __deepcopy__(self, memo):
        return ForceMeasurement(self, self.BASE_UNIT)

    def __str__(self):
        return f"{float(self)} {self.BASE_UNIT.name}"

    def to(self, unit: ForceUnits) -> float:
        return self * self.BASE_UNIT.value / unit.value

    def __add__(self, other):
        if isinstance(other, ForceMeasurement):
            return ForceMeasurement(float(self) + float(other), self.BASE_UNIT)
        return NotImplemented

    def __sub__(self, other):
        if isinstance(other, ForceMeasurement):
            return ForceMeasurement(float(self) - float(other), self.BASE_UNIT)
        return NotImplemented

    def __mul__(self, other):
        return ForceMeasurement(float(self) * float(other), self.BASE_UNIT)

    def __truediv__(self, other):
        if isinstance(other, ForceMeasurement):
            return float(self) / float(other)
        return ForceMeasurement(float(self) / float(other), self.BASE_UNIT)

    def __floordiv__(self, other):
        if isinstance(other, ForceMeasurement):
            return float(self) // float(other)
        return ForceMeasurement(float(self) // float(other), self.BASE_UNIT)

    def __mod__(self, other):
        return ForceMeasurement(float(self) % float(other), self.BASE_UNIT)

    def __neg__(self):
        return ForceMeasurement(-float(self), self.BASE_UNIT)

    def __abs__(self):
        return ForceMeasurement(abs(float(self)), self.BASE_UNIT)

    def __pow__(self, other):
        return ForceMeasurement(float(self) ** float(other), self.BASE_UNIT)


class Newton(ForceMeasurement):
    def __new__(cls, value: float):
        return ForceMeasurement.__new__(cls, value, ForceUnits.Newton)


class Pound(ForceMeasurement):
    def __new__(cls, value: float):
        return ForceMeasurement.__new__(cls, value, ForceUnits.Pound)


class TimeUnits(MeasurementUnit):
    Second = 1
    Minute = 60
    Hour = 3600
    Day = 86400
    Week = 604800
    Year = 31536000

    def wth_uses_you(self) -> "TimeMeasurement":
        return TimeMeasurement


class TimeMeasurement(Measurement):
    BASE_UNIT = TimeUnits.Second
    MAX_DECIMALS = 9

    def __new__(cls, value: float, unit: TimeUnits):
        if isinstance(value, TimeMeasurement):
            return value
        value = round(value, cls.MAX_DECIMALS)
        if isinstance(unit, float):
            warnings.warn(
                "UNIT WARNING! Using float as unit increases the risk that you collide with the Martian surface, use a specified TimeUnits Enum instead."
            )
            value = value * unit / cls.BASE_UNIT.value
        else:
            value = value * unit.value / cls.BASE_UNIT.value
        value = round(value, cls.MAX_DECIMALS)
        return float.__new__(cls, value)

    def __deepcopy__(self, memo):
        return TimeMeasurement(self, self.BASE_UNIT)

    def __str__(self):
        return f"{float(self)} {self.BASE_UNIT.name}"

    def to(self, unit: TimeUnits) -> float:
        return self * self.BASE_UNIT.value / unit.value

    def __add__(self, other):
        if isinstance(other, TimeMeasurement):
            return TimeMeasurement(float(self) + float(other), self.BASE_UNIT)
        return NotImplemented

    def __sub__(self, other):
        if isinstance(other, TimeMeasurement):
            return TimeMeasurement(float(self) - float(other), self.BASE_UNIT)
        return NotImplemented

    def __mul__(self, other):
        return TimeMeasurement(float(self) * float(other), self.BASE_UNIT)

    def __truediv__(self, other):
        if isinstance(other, TimeMeasurement):
            return float(self) / float(other)
        return TimeMeasurement(float(self) / float(other), self.BASE_UNIT)

    def __floordiv__(self, other):
        if isinstance(other, TimeMeasurement):
            return float(self) // float(other)
        return TimeMeasurement(float(self) // float(other), self.BASE_UNIT)

    def __mod__(self, other):
        return TimeMeasurement(float(self) % float(other), self.BASE_UNIT)

    def __neg__(self):
        return TimeMeasurement(-float(self), self.BASE_UNIT)

    def __abs__(self):
        return TimeMeasurement(abs(float(self)), self.BASE_UNIT)

    def __pow__(self, other):
        return TimeMeasurement(float(self) ** float(other), self.BASE_UNIT)


class Second(TimeMeasurement):
    def __new__(cls, value: float):
        return TimeMeasurement.__new__(cls, value, TimeUnits.Second)


class Minute(TimeMeasurement):
    def __new__(cls, value: float):
        return TimeMeasurement.__new__(cls, value, TimeUnits.Minute)


class Hour(TimeMeasurement):
    def __new__(cls, value: float):
        return TimeMeasurement.__new__(cls, value, TimeUnits.Hour)


class Day(TimeMeasurement):
    def __new__(cls, value: float):
        return TimeMeasurement.__new__(cls, value, TimeUnits.Day)


class Week(TimeMeasurement):
    def __new__(cls, value: float):
        return TimeMeasurement.__new__(cls, value, TimeUnits.Week)


class Year(TimeMeasurement):
    def __new__(cls, value: float):
        return TimeMeasurement.__new__(cls, value, TimeUnits.Year)


class MassUnits(Enum):
    Kilogram = 1
    Pound = 0.453592
    Gram = 0.001
    Ounce = 0.0283495
    Stone = 6.35029
    Ton = 1000
    MetricTon = 1000

    def wth_uses_you(self) -> "MassMeasurement":
        return MassMeasurement


class MassMeasurement(Measurement):
    BASE_UNIT = MassUnits.Kilogram
    MAX_DECIMALS = 9

    def __new__(cls, value: float, unit: MassUnits):
        if isinstance(value, MassMeasurement):
            return value
        value = round(value, cls.MAX_DECIMALS)
        if isinstance(unit, float):
            warnings.warn(
                "UNIT WARNING! Using float as unit increases the risk that you collide with the Martian surface, use a specified MassUnits Enum instead."
            )
            value = value * unit / cls.BASE_UNIT.value
        else:
            value = value * unit.value / cls.BASE_UNIT.value
        value = round(value, cls.MAX_DECIMALS)
        return float.__new__(cls, value)

    def __deepcopy__(self, memo):
        return MassMeasurement(self, self.BASE_UNIT)

    def __str__(self):
        return f"{float(self)} {self.BASE_UNIT.name}"

    def to(self, unit: MassUnits) -> float:
        return self * self.BASE_UNIT.value / unit.value

    def __add__(self, other):
        if isinstance(other, MassMeasurement):
            return MassMeasurement(float(self) + float(other), self.BASE_UNIT)
        return NotImplemented

    def __sub__(self, other):
        if isinstance(other, MassMeasurement):
            return MassMeasurement(float(self) - float(other), self.BASE_UNIT)
        return NotImplemented

    def __mul__(self, other):
        return MassMeasurement(float(self) * float(other), self.BASE_UNIT)

    def __truediv__(self, other):
        if isinstance(other, MassMeasurement):
            return float(self) / float(other)
        return MassMeasurement(float(self) / float(other), self.BASE_UNIT)

    def __floordiv__(self, other):
        if isinstance(other, MassMeasurement):
            return float(self) // float(other)
        return MassMeasurement(float(self) // float(other), self.BASE_UNIT)

    def __mod__(self, other):
        return MassMeasurement(float(self) % float(other), self.BASE_UNIT)

    def __neg__(self):
        return MassMeasurement(-float(self), self.BASE_UNIT)

    def __abs__(self):
        return MassMeasurement(abs(float(self)), self.BASE_UNIT)

    def __pow__(self, other):
        return MassMeasurement(float(self) ** float(other), self.BASE_UNIT)
