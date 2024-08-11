from gamegine.utils.NCIM.basic import Measurement, Unit
from gamegine.utils.NCIM.dimension import Dimension


class SpatialUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Spatial, scale, display, shift)

    def __call__(self, magnitude: float):
        return SpatialMeasurement(magnitude, self)


class SpatialMeasurement(Measurement):
    def __init__(
        self, magnitude: float, unit: SpatialUnit, base_magnitude=None
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Meter = SpatialUnit(10000, "m")
Centimeter = SpatialUnit(100, "cm")
Feet = SpatialUnit(3048, "ft")
Inch = SpatialUnit(254, "in")
Yard = SpatialUnit(9144, "yd")
Kilometer = SpatialUnit(10000000, "km")
Mile = SpatialUnit(16093400, "mi")
NauticalMile = SpatialUnit(18520000, "nmi")
MicroMeter = SpatialUnit(0.01, "μm")
