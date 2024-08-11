from gamegine.utils.NCIM.dimension import Dimension
from gamegine.utils.NCIM.basic import Measurement, Unit


class MassUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Mass, scale, display, shift)

    def __call__(self, magnitude: float):
        return MassMeasurement(magnitude, self)


class MassMeasurement(Measurement):
    def __init__(self, magnitude: float, unit: MassUnit, base_magnitude=None) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Gram = MassUnit(1, "g")
Kilogram = MassUnit(1000, "kg")
Milligram = MassUnit(0.001, "mg")
Pound = MassUnit(453.592, "lb")
Ounce = MassUnit(28.3495, "oz")
