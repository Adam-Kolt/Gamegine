from enum import Enum
from typing import List, Tuple
import pint
import pint.converters
from gamegine import ureg, Q_
import warnings

StandardUnit = ureg.meter
"""
    TODO: Maybe this
    Now that i think about it...we may want to just convert units on creation to a float of the standard unit, speeding up legit everything...
    Oh well, maybe by the time we get GTA 6.
    ¯\_(ツ)_/¯
"""


def Meter(value) -> pint.Quantity:
    return Q_(value, 'meter')

def Zero() -> pint.Quantity:
    return Q_(0, 'meter')

def Centimeter(value) -> pint.Quantity:
    return Q_(value, 'centimeter')

def Feet(value) -> pint.Quantity:
    return Q_(value, 'foot')

def Inch(value) -> pint.Quantity:
    return Q_(value, 'inch')

def HalfSub(value) -> pint.Quantity:
    return Inch(value * 6)

def FullSub(value) -> pint.Quantity:
    return Feet(value)

def BigMac(value) -> pint.Quantity:
    return Centimeter(value  * 10)

def Second(value) -> pint.Quantity:
    return Q_(value, 'second')

def Radian(value) -> pint.Quantity:
    return Q_(value, 'radian')

def Degree(value) -> pint.Quantity:
    return Q_(value, 'degree')

def GetRegistry():
    return ureg

def Quantity(value, unit):
    return Q_(value, unit)

# TODO: There may have been a slight oversight with this function. It only works for and made for length units. Maybe name it better.
# \_(ツ)_/¯

def StdMag(value: pint.Quantity):
    return value.to(StandardUnit).magnitude

def StdMagTuple(value: Tuple[pint.Quantity]):
    return tuple([StdMag(val) for val in value])

def StdMagList(value: List[pint.Quantity]):
    return [StdMag(val) for val in value]

def ToStd(value: float):
    return Q_(value, StandardUnit)


def List2Std(value: List[float]):
    return [Q_(val, StandardUnit) for val in value]

def Tuple2Std(value: Tuple[float]):
    return tuple([Q_(val, StandardUnit) for val in value])

def RatioOf(quant1, quant2):
    return (quant1/quant2).to_base_units().magnitude



### Awaiting Substitution into Old Unit System
### Eventually... ¯\_(ツ)_/¯

class LinearUnits(Enum):
    Meter = 1
    Centimeter = 0.01
    Feet = 0.3048
    Inch = 0.0254
    Yard = 0.9144
    Kilometer = 1000
    Mile = 1609.34
    NauticalMile = 1852 # Future proofing for FRC in 2077


class LinearMeasurement(float): # A class which ensures unit-aware initialization of linear measurements
    def __new__(cls, value: float, unit: LinearUnits):
        if isinstance(value, LinearMeasurement):
            return value
        if isinstance(unit, float):
            warnings.warn("UNIT WARNING! Using float as unit increases the risk that you collide with the Martian service, use a specified LinearUnits Enum instead.")
            return float.__new__(cls, value * unit)
        return float.__new__(cls, value * unit.value)
    
    def to(self, unit: LinearUnits) -> float:
        return self / unit.value

        
