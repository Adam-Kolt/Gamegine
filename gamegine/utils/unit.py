from enum import Enum
from typing import List, Tuple
import pint
import pint.converters
from gamegine import ureg, Q_
import warnings

# TODO: Please PLEASE clean this mess up ¯\_(ツ)_/¯

class AngularUnits(Enum):
    Radian = 1
    Degree = 0.0174533

class AngularMeasurement(float):
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


class SpatialUnits(Enum):
    Meter = 1
    Centimeter = 0.01
    Feet = 0.3048
    Inch = 0.0254
    Yard = 0.9144
    Kilometer = 1000
    Mile = 1609.34
    NauticalMile = 1852
    MicroMeter = 0.000001

# TODO: You know, the naming might not be 100% great, but at least it's compatible with Apple
class SpatialMeasurement(
    float
):  # A class which ensures unit-aware initialization of spacial measurements
    BASE_UNIT = SpatialUnits.MicroMeter # To avoid floating point errors, we make the base unit smal, so all units are stored as integer floats
    MAX_DECIMALS = 9 # For creation and conversion into base unit, to avoid floating point errors

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
        return AngularMeasurement(-float(self), self.BASE_UNIT)
    
    def __abs__(self):
        return AngularMeasurement(abs(float(self)), self.BASE_UNIT)
    
    def __pow__(self, other):
        return AngularMeasurement(float(self) ** float(other), self.BASE_UNIT)

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
  