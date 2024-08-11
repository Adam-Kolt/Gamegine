from gamegine.utils.NCIM.basic import Measurement, Unit
from gamegine.utils.NCIM.dimension import Dimension


class TemporalUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Temporal, scale, display, shift)

    def __call__(self, magnitude: float):
        return TemporalMeasurement(magnitude, self)


class TemporalMeasurement(Measurement):
    def __init__(
        self, magnitude: float, unit: TemporalUnit, base_magnitude=None
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Second = TemporalUnit(1, "s")
Minute = TemporalUnit(60, "min")
Hour = TemporalUnit(3600, "h")
Day = TemporalUnit(86400, "d")
Week = TemporalUnit(604800, "wk")
Month = TemporalUnit(2628000, "mo")
Year = TemporalUnit(31536000, "yr")
