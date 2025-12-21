from gamegine.utils.NCIM.core import Measurement, Unit, Dimension
from gamegine.utils.NCIM.Dimensions.spatial import SpatialUnit, Meter, Kilometer, Mile, Feet
from gamegine.utils.NCIM.Dimensions.temporal import TemporalUnit, Second, Hour
import numpy as np

class VelocityUnit(Unit):
    """
    A class representing a velocity unit (Length/Time).
    """
    def __init__(self, spatial: SpatialUnit, time: TemporalUnit) -> None:
        # Construct dimensions: Spatial^1 * Temporal^-1
        # Implementation: we can just divide the units
        # But we need to call super().__init__ properly to set up THIS unit
        
        # Calculate resulting scale and symbol
        res_scale = spatial.scale / time.scale
        res_symbol = f"{spatial.symbol}/{time.symbol}"
        
        dims = np.zeros(len(Dimension))
        dims[Dimension.Spatial.value] = 1
        dims[Dimension.Temporal.value] = -1
        
        super().__init__(dims, res_scale, res_symbol)

    def __call__(self, magnitude: float):
        return Velocity(magnitude, self)

class Velocity(Measurement):
    """
    A class representing a velocity measurement.
    """
    def __init__(self, magnitude: float, unit: VelocityUnit, base_magnitude=None):
        if base_magnitude is not None:
             magnitude = unit.from_base(base_magnitude)
        super().__init__(magnitude, unit)

MetersPerSecond = VelocityUnit(Meter, Second)
KilometersPerHour = VelocityUnit(Kilometer, Hour)
MilesPerHour = VelocityUnit(Mile, Hour)
FeetPerSecond = VelocityUnit(Feet, Second)
