import pint
from gamegine import ureg, Q_




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
        
def RatioOf(quant1, quant2):
    return (quant1/quant2).to_base_units().magnitude