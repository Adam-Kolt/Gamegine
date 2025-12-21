from typing import Generic, TypeVar, Union, Optional, Dict, Tuple
from enum import Enum
import numpy as np
from abc import ABC

# Forward declarations
class Dimension(Enum):
    """Enum for the different dimensions of the NCIM system"""
    Spatial = 0
    Mass = 1
    Force = 2
    Energy = 3
    Temporal = 4
    Angular = 5
    ElectricCurrent = 6

T = TypeVar('T', bound=float)

class Unit:
    """
    Represents a unit of measurement.
    Can be a simple unit (e.g. Meter) or complex (e.g. Meter/Second).
    """
    __slots__ = ('_dimensions', '_scale', '_offset', '_symbol', '_is_simple')

    def __init__(self, 
                 dimensions: np.ndarray, 
                 scale: float = 1.0, 
                 symbol: str = "", 
                 offset: float = 0.0):
        self._dimensions = dimensions # Array of powers for each Dimension enum
        self._scale = scale
        self._symbol = symbol
        self._offset = offset
        # Optimization flag: true if this is a simple, single dimension unit with no complex mapping needed
        self._is_simple = np.count_nonzero(dimensions) == 1 and np.sum(dimensions) == 1

    @property
    def dimensions(self) -> np.ndarray:
        return self._dimensions

    @property
    def dimension(self) -> np.ndarray:
        # Backward compatibility alias
        return self._dimensions

    @property
    def scale(self) -> float:
        return self._scale
    
    @property
    def offset(self) -> float:
        return self._offset

    @property
    def symbol(self) -> str:
        return self._symbol

    def to_base(self, value: float) -> float:
        return (value * self._scale) + self._offset

    def from_base(self, value: float) -> float:
        return (value - self._offset) / self._scale

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Unit):
            return NotImplemented
        return (np.array_equal(self._dimensions, other._dimensions) and 
                self._scale == other._scale and 
                self._offset == other._offset)

    def __hash__(self) -> int:
        # Use a tuple of the dimensions array (read-only view or bytes) + scale + offset
        return hash((self._dimensions.tobytes(), self._scale, self._offset))


    def __str__(self) -> str:
        return self._symbol

    def __repr__(self) -> str:
        return f"Unit({self._symbol}, scale={self._scale})"
    
    def __pow__(self, power: int) -> 'Unit':
        # Create a new unit raised to a power
        # Note: Scale is raised to power, offset is... complicated for non-linear units, 
        # usually ignored for complex units or 0.
        return Unit(
            self._dimensions * power,
            self._scale ** power,
            f"{self._symbol}^{power}" if self._symbol else "",
            0 # Offset usually doesn't make sense to power
        )
    
    def __mul__(self, other: 'Unit') -> 'Unit':
        return Unit(
            self._dimensions + other._dimensions,
            self._scale * other._scale,
            f"{self._symbol}*{other._symbol}", 
            0
        )
        
    def __truediv__(self, other: 'Unit') -> 'Unit':
        return Unit(
            self._dimensions - other._dimensions,
            self._scale / other._scale,
            f"{self._symbol}/{other._symbol}",
            0
        )

class Measurement(float):
    """
    Unified class for all measurements.
    Inherits from float to allow direct usage in many places, 
    but carries Unit information.
    """
    __slots__ = ('_unit')

    def __new__(cls, value: float, unit: Unit, base_magnitude=None, *args, **kwargs):
        if base_magnitude is not None:
             # Already base
             val_base = base_magnitude
        else:
             # Convert to base
             val_base = unit.to_base(value)
        return float.__new__(cls, val_base)

    def __init__(self, value: float, unit: Unit, *args, **kwargs):
        # float is immutable, so value is passed to __new__
        self._unit = unit

    @property
    def unit(self) -> Unit:
        return self._unit

    def to(self, target_unit: Unit) -> 'Measurement':
        """Convert this measurement to another unit."""
        if not np.array_equal(self._unit.dimensions, target_unit.dimensions):
             # Compatibility: Allow if dimensions are effectively same (all zeros vs empty? no, arrays check handles it)
             # But what about compatible units like Joule vs N*m?
             # dimensions array equality check is correct for that.
             raise ValueError(f"Incompatible dimensions: {self._unit} vs {target_unit}")
        
        # Self is already base value
        base_val = float(self)
        # We return a new Measurement. 
        # Measurement(val, unit) -> converts val to base.
        # So we need to convert base_val to target_unit magnitude first, 
        # so that when passed to __new__, it gets converted BACK to base (which equals base_val).
        # OR we pass base_magnitude=base_val.
        return Measurement(0.0, target_unit, base_magnitude=base_val)

    # --- Arithmetic Operations ---

    def __add__(self, other: object) -> 'Measurement':
        if isinstance(other, Measurement):
            # FAST PATH: Same unit
            if self._unit is other._unit: 
                # Values are base values. Add them directly.
                return Measurement(0, self._unit, base_magnitude=float(self) + float(other))
            
            # Compatible dimensions check
            if np.array_equal(self._unit.dimensions, other._unit.dimensions):
                # Values are base values. Add them directly.
                return Measurement(0, self._unit, base_magnitude=float(self) + float(other))
            
            raise ValueError(f"Cannot add measurements of different dimensions: {self._unit} vs {other._unit}")
        return NotImplemented

    def __radd__(self, other: object) -> 'Measurement':
        return self.__add__(other)

    def __sub__(self, other: object) -> 'Measurement':
        if isinstance(other, Measurement):
            # FAST PATH
            if self._unit is other._unit:
                return Measurement(0, self._unit, base_magnitude=float(self) - float(other))

            if np.array_equal(self._unit.dimensions, other._unit.dimensions):
                 return Measurement(0, self._unit, base_magnitude=float(self) - float(other))
            
            raise ValueError(f"Cannot subtract measurements of different dimensions")
        return NotImplemented

    def __rsub__(self, other: object) -> 'Measurement':
        # other - self
        if isinstance(other, Measurement):
            return other.__sub__(self)
        return NotImplemented

    def __mul__(self, other: object) -> 'Measurement':
        if isinstance(other, Measurement):
            # Multiply values (base) -> new base value?
            # m1 (base) * m2 (base).
            # Unit1 * Unit2 -> new unit.
            # Example: 2m * 3s -> 6 m*s.
            # m1 store 2. m2 store 3. float(m1)*float(m2) = 6.
            # New unit: m*s. Scale = 1*1=1.
            # Measurement(new_unit, base_val=6).
            # If 2ft * 3s.
            # m1 = 0.6096. m2 = 3.
            # float*float = 1.8288.
            # New unit: ft*s. Scale = 0.3048 * 1 = 0.3048.
            # Measurement(val=?, ft*s). 
            # If we pass base_magnitude=1.8288.
            # from_base(1.8288) = 1.8288 / 0.3048 = 6.0.
            # Correct. 2ft * 3s = 6 ft*s.
            
            new_unit = self._unit * other._unit
            return Measurement(0, new_unit, base_magnitude=float(self) * float(other))
        elif isinstance(other, (float, int)):
            return Measurement(0, self._unit, base_magnitude=float(self) * other)
        return NotImplemented

    def __rmul__(self, other: object) -> 'Measurement':
        return self.__mul__(other)

    def __truediv__(self, other: object) -> 'Measurement':
        if isinstance(other, Measurement):
            new_unit = self._unit / other._unit
            return Measurement(0, new_unit, base_magnitude=float(self) / float(other))
        elif isinstance(other, (float, int)):
            return Measurement(0, self._unit, base_magnitude=float(self) / other)
        return NotImplemented

    def __rtruediv__(self, other: object) -> 'Measurement':
        if isinstance(other, (float, int)):
            # scalar / Measurement -> Inverse unit
            # Create a '1' unit (dimensionless)
            # Actually, we can just invert the unit directly conceptually
            # but we need a '1' unit to divide FROM.
            # Simplified: Inverse the measurement
            # Unit^-1
            
            # scalar / Measurement
            # 1 / m. 
            # float(m) = 10 (if 10m).
            # 1/10 = 0.1 m^-1.
            
            dims = np.zeros(len(Dimension))
            scalar_unit = Unit(dims, 1.0, "")
            
            new_unit = scalar_unit / self._unit
            return Measurement(0, new_unit, base_magnitude=other / float(self))
            
        return NotImplemented

    def __str__(self) -> str:
        # Show magnitude in current unit
        val = self._unit.from_base(float(self))
        return f"{val:.4f} {self._unit.symbol}"

    def __repr__(self) -> str:
        # repr should show creation args (magnitude, unit)
        val = self._unit.from_base(float(self))
        return f"Measurement({val}, {self._unit})"

