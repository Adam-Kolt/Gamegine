from gamegine.utils.NCIM.Dimensions.angular import AngularUnit, Degree, Radian
from gamegine.utils.NCIM.Dimensions.temporal import Second, TemporalUnit
from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit


class OmegaUnit(ComplexUnit):
    def __init__(self, angles: AngularUnit, time: TemporalUnit) -> None:
        super().__init__({angles: 1, time: -1})

    def __call__(self, magnitude: float):
        return Omega(magnitude, self)


class Omega(ComplexMeasurement):
    def __new__(
        cls,
        magnitude: float,
        unit: OmegaUnit,
        base_magnitude=None,
    ):
        return ComplexMeasurement.__new__(cls, magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: OmegaUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


RadiansPerSecond = OmegaUnit(Radian, Second)
DegreesPerSecond = OmegaUnit(Degree, Second)
