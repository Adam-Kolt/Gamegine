from typing import List, Tuple
import pint
from gamegine import ureg, Q_

StandardUnit = ureg.meter

def Meter(value) -> pint.Quantity:
    return Q_(value, 'meter')

def Centimeter(value) -> pint.Quantity:
    return Q_(value, 'centimeter')

def Feet(value) -> pint.Quantity:
    return Q_(value, 'foot')

def Inch(value) -> pint.Quantity:
    return Q_(value, 'inch')

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