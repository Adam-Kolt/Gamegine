from gamegine.utils.NCIM.Dimensions.spatial import Kilometer, Meter, SpatialUnit
from gamegine.utils.NCIM.Dimensions.temporal import Hour, Second, TemporalUnit
from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit


class AccelerationUnit(ComplexUnit):
    def __init__(self, spatial: SpatialUnit, time: TemporalUnit) -> None:
        super().__init__({spatial: 1, time: -2})

    def __call__(self, magnitude: float):
        return Acceleration(magnitude, self)


class Acceleration(ComplexMeasurement):
    def __new__(
        cls,
        magnitude: float,
        unit: AccelerationUnit,
        base_magnitude=None,
    ):
        return ComplexMeasurement.__new__(magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: AccelerationUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


MeterPerSecondSquared = AccelerationUnit(Meter, Second)
KilometerPerHourSquared = AccelerationUnit(Kilometer, Hour)
