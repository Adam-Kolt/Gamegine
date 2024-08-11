from gamegine.utils.NCIM.Dimensions.force import ForceUnit, Newton
from gamegine.utils.NCIM.Dimensions.spatial import Meter, SpatialUnit
from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit


class TorqueUnit(ComplexUnit):
    def __init__(self, spatial: SpatialUnit, force: ForceUnit) -> None:
        super().__init__({spatial: 1, force: 1})

    def __call__(self, magnitude: float):
        return Torque(magnitude, self)


class Torque(ComplexMeasurement):
    def __new__(
        cls,
        magnitude: float,
        unit: TorqueUnit,
        base_magnitude=None,
    ):
        return ComplexMeasurement.__new__(cls, magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: TorqueUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


NewtonMeter = TorqueUnit(Meter, Newton)
