from gamegine.utils.NCIM.core import Measurement, Unit, Dimension
import numpy as np

class SpatialUnit(Unit):
    """
    A class representing a spatial unit of measurement.
    """
    def __init__(self, scale: float = 1.0, display: str = "", shift: float = 0.0) -> None:
        dims = np.zeros(len(Dimension))
        dims[Dimension.Spatial.value] = 1
        super().__init__(dims, scale, display, shift)

    def __call__(self, magnitude: float):
        return SpatialMeasurement(magnitude, self)

class SpatialMeasurement(Measurement):
    """
    A class representing a spatial measurement.
    """
    def __init__(self, magnitude: float, unit: SpatialUnit, base_magnitude=None):
        # Handle base_magnitude for compatibility if needed, but core.Measuremont doesn't strictly use it in __init__ 
        # the same way. The new core expects (value, unit). 
        # If base_magnitude is provided, we should convert it.
        if base_magnitude is not None:
             # Convert base to unit
             # val = (base - shift) / scale
             magnitude = unit.from_base(base_magnitude)
        super().__init__(magnitude, unit)

Meter = SpatialUnit(1.0, "m") # Base unit
Centimeter = SpatialUnit(0.01, "cm")
Feet = SpatialUnit(0.3048, "ft")
Inch = SpatialUnit(0.0254, "in")
Yard = SpatialUnit(0.9144, "yd")
Kilometer = SpatialUnit(1000.0, "km")
Mile = SpatialUnit(1609.34, "mi")
NauticalMile = SpatialUnit(1852.0, "nmi")
MicroMeter = SpatialUnit(1e-6, "μm")
