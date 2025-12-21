from gamegine.utils.NCIM.core import Measurement, Unit as CoreUnit, Dimension
import numpy as np
from typing import Dict, Union

# Unit shim to handle legacy initialization (dimension Enum vs array)
class Unit(CoreUnit):
    def __init__(self, dimension_or_array, scale: float = 0, symbol: str = "", shift: float = 0.0) -> None:
        """
        Legacy shim for Unit initialization.
        Old signature: (dimension: Dimension, scale, symbol, shift)
        New signature: (dimensions: np.ndarray, scale, symbol, offset)
        """
        if isinstance(dimension_or_array, Dimension):
            # Convert single dimension enum to array
            dims = np.zeros(len(Dimension))
            dims[dimension_or_array.value] = 1
            # Note: Old 'shift' corresponds to new 'offset'
            # Old default scale was 0? Wait, typically 1 or specific.
            # In old basic.py: scale: float = 0 (default).
            super().__init__(dims, scale, symbol, shift)
        else:
            # Assume it's the new signature (numpy array)
            super().__init__(dimension_or_array, scale, symbol, shift)

# ComplexUnit shim to handle legacy dictionary initialization
class ComplexUnit(Unit):
    def __init__(self, units_table: Dict[CoreUnit, int]) -> None:
        """
        Legacy shim for ComplexUnit(units_table).
        Converts the dict {Unit: power} into dimensions/scale/symbol.
        """
        dims = np.zeros(len(Dimension))
        scale = 1.0
        numerator_symbols = []
        denominator_symbols = []
        
        for unit, power in units_table.items():
            # Add dimensions
            # unit might be a legacy shim Unit or CoreUnit
            dims += unit.dimensions * power
            # Multiply scale
            scale *= (unit.scale ** power)
            
            # Handle symbol approximation
            sym = unit.symbol
            if power > 0:
                if power == 1:
                    numerator_symbols.append(sym)
                else:
                    numerator_symbols.append(f"{sym}^{power}")
            elif power < 0:
                p = -power
                if p == 1:
                    denominator_symbols.append(sym)
                else:
                    denominator_symbols.append(f"{sym}^{p}")
        
        # Construct symbol string
        if not numerator_symbols:
            num_str = "1"
        else:
            num_str = "*".join(numerator_symbols)
            
        if not denominator_symbols:
            res_symbol = num_str
        else:
            den_str = "*".join(denominator_symbols)
            res_symbol = f"{num_str}/{den_str}"

        super().__init__(dims, scale, res_symbol)

# Alias Measurement as ComplexMeasurement as they are now unified
ComplexMeasurement = Measurement

# Alias ComplexDimension as it's just a numpy array in the new system
# If legacy code expects it to be a class, we might need a dummy class inheriting from np.ndarray
# or just alias it to np.ndarray if used primarily for type hinting or simple ops.
# For now, let's alias to np.ndarray, effectively.
ComplexDimension = np.ndarray

def RatioOf(a, b):
    return a / b
