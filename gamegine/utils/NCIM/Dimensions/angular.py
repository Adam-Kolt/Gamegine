from gamegine.utils.NCIM.basic import Measurement, Unit
from gamegine.utils.NCIM.dimension import Dimension


class AngularUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Angular, scale, display, shift)

    def __call__(self, magnitude: float):
        return AngularMeasurement(magnitude, self)


class AngularMeasurement(Measurement):
    def __init__(
        self, magnitude: float, unit: AngularUnit, base_magnitude=None
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Radian = AngularUnit(1, "rad")
Degree = AngularUnit(0.0174533, "°")
Grad = AngularUnit(0.01570796, "grad")
