from gamegine.utils.NCIM.core import Measurement, Unit, Dimension
import numpy as np

class TemporalUnit(Unit):
    """
    A class representing a temporal unit of measurement.
    """
    def __init__(self, scale: float = 1.0, display: str = "", shift: float = 0.0) -> None:
        dims = np.zeros(len(Dimension))
        dims[Dimension.Temporal.value] = 1
        super().__init__(dims, scale, display, shift)

    def __call__(self, magnitude: float):
        return TemporalMeasurement(magnitude, self)

class TemporalMeasurement(Measurement):
    """
    A class representing a temporal measurement.
    """
    def __init__(self, magnitude: float, unit: TemporalUnit, base_magnitude=None):
        if base_magnitude is not None:
             magnitude = unit.from_base(base_magnitude)
        super().__init__(magnitude, unit)

Second = TemporalUnit(1.0, "s")
Minute = TemporalUnit(60.0, "min")
Hour = TemporalUnit(3600.0, "h")
Day = TemporalUnit(86400.0, "d")
Week = TemporalUnit(604800.0, "wk")
Month = TemporalUnit(2628000.0, "mo")
Year = TemporalUnit(31536000.0, "yr")
