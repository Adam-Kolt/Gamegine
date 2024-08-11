from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit
from gamegine.utils.NCIM.Dimensions.spatial import (
    Feet,
    Kilometer,
    Meter,
    Mile,
    SpatialUnit,
)
from gamegine.utils.NCIM.Dimensions.temporal import Hour, Second, TemporalUnit


class VelocityUnit(ComplexUnit):
    def __init__(self, spatial: SpatialUnit, time: TemporalUnit) -> None:
        super().__init__({spatial: 1, time: -1})

    def __call__(self, magnitude: float):
        return Velocity(magnitude, self)


class Velocity(ComplexMeasurement):
    def __new__(
        cls,
        magnitude: float,
        unit: VelocityUnit,
        base_magnitude=None,
    ):
        return ComplexMeasurement.__new__(cls, magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: VelocityUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


MetersPerSecond = VelocityUnit(Meter, Second)
KilometersPerHour = VelocityUnit(Kilometer, Hour)
MilesPerHour = VelocityUnit(Mile, Hour)
FeetPerSecond = VelocityUnit(Feet, Second)
