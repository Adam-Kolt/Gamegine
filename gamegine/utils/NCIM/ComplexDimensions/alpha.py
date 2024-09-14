from gamegine.utils.NCIM.Dimensions.angular import AngularUnit, Degree, Radian
from gamegine.utils.NCIM.Dimensions.temporal import Second, TemporalUnit
from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit


class AlphaUnit(ComplexUnit):
    def __init__(self, angles: AngularUnit, time: TemporalUnit) -> None:
        super().__init__({angles: 1, time: -2})

    def __call__(self, magnitude: float):
        return Alpha(magnitude, self)


class Alpha(ComplexMeasurement):
    def __new__(cls, magnitude: float, unit: AlphaUnit, base_magnitude=None):
        return ComplexMeasurement.__new__(cls, magnitude, unit, base_magnitude)

    def __init__(self, magnitude: float, unit: AlphaUnit, base_magnitude=None) -> None:
        super().__init__(magnitude, unit, base_magnitude)


RadiansPerSecondSquared = AlphaUnit(Radian, Second)
DegreesPerSecondSquared = AlphaUnit(Degree, Second)
