from gamegine.utils.NCIM.Dimensions.mass import Kilogram, MassUnit
from gamegine.utils.NCIM.Dimensions.spatial import Meter, SpatialUnit
from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit


class MOIUnit(ComplexUnit):
    def __init__(self, mass: MassUnit, spatial: SpatialUnit) -> None:
        super().__init__({spatial: 1, mass: 2})

    def __call__(self, magnitude: float):
        return MOI(magnitude, self)


class MOI(ComplexMeasurement):
    def __new__(
        cls,
        magnitude: float,
        unit: MOIUnit,
        base_magnitude=None,
    ):
        return ComplexMeasurement.__new__(cls, magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: MOIUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


KilogramMetersSquared = MOIUnit(Kilogram, Meter)
