from gamegine.utils.NCIM.basic import Measurement, Unit
from gamegine.utils.NCIM.dimension import Dimension


class ForceUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Force, scale, display, shift)

    def __call__(self, magnitude: float):
        return ForceMeasurement(magnitude, self)


class ForceMeasurement(Measurement):
    def __init__(self, magnitude: float, unit: ForceUnit, base_magnitude=None) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Newton = ForceUnit(1, "N")
Dyne = ForceUnit(0.00001, "dyn")
PoundForce = ForceUnit(4.44822, "lbf")
KilogramForce = ForceUnit(9.80665, "kgf")
