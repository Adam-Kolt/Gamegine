from copy import deepcopy
from enum import Enum
from abc import ABC, abstractmethod
from typing import Dict

from gamegine.utils.NCIM.dimension import Dimension
from gamegine.utils.logging import Debug, Error, Warn
import numpy as np


class Unit(object):
    ROUNDING = 9

    def __init__(
        self, dimension: "Dimension", scale: float = 0, symbol="", shift: float = 0
    ) -> None:
        self.scale = scale
        self.shift = shift
        self.symbol = symbol
        self.dimension = dimension

    def to_base(self, value: float) -> float:
        return round(float(value) * self.scale + self.shift, Unit.ROUNDING)

    def from_base(self, value: float) -> float:
        return round(float(value) / self.scale - self.shift, Unit.ROUNDING)

    def get_dimension(self) -> "Dimension":
        return self.dimension

    def get_symbol(self) -> str:
        return self.symbol

    def get_scale(self):
        return self.scale


class Measurement(float):
    def __new__(cls, magnitude: float, unit: Unit, base_magnitude=None):
        if base_magnitude:
            return float.__new__(cls, base_magnitude)
        if isinstance(magnitude, Measurement):
            return magnitude
        value = unit.to_base(magnitude)
        return float.__new__(cls, value)

    def __init__(self, magnitude: float, unit: Unit, base_magnitude=None) -> None:
        super().__init__()
        self.dimension = unit.get_dimension()
        self.unit = unit

    def __deepcopy__(self, memo):
        return Measurement(0, self.unit, base_magnitude=float(self))

    def to(self, unit: Unit):
        return unit.from_base(self)

    def get_unit_magnitude(self) -> float:
        return self.unit.from_base(self)

    def __str__(self) -> str:
        return f"{self.get_unit_magnitude()} {self.unit.get_symbol()}"

    def __repr__(self) -> str:
        return self.__str__()

    def get_dimension(self) -> Dimension:
        return self.dimension

    def get_unit(self) -> Unit:
        return self.unit

    def __is_same_dimension(self, other):
        return other.get_dimension() == self.get_dimension()

    @classmethod
    def __is_a_measurement(cls, other):
        return isinstance(other, Measurement)

    def __add__(self, other) -> float:
        if isinstance(other, ComplexMeasurement):
            return self.to_complex() + other
        elif Measurement.__is_a_measurement(other) and self.__is_same_dimension(other):
            return Measurement(0, self.unit, base_magnitude=float(self) + float(other))
        else:
            Error("Cannot add measurements of different dimensions")

    def __sub__(self, other) -> float:
        if Measurement.__is_a_measurement(other) and self.__is_same_dimension(other):
            return Measurement(0, self.unit, base_magnitude=float(self) - float(other))
        else:
            Error("Cannot add measurements of different dimensions")

    def __mul__(self, other):  # Make complex units
        if isinstance(other, Measurement) or isinstance(other, ComplexMeasurement):
            return self.to_complex() * other.to_complex()
        else:
            return Measurement(0, self.unit, base_magnitude=float(self) * float(other))

    def __truediv__(self, other):
        if isinstance(other, Measurement):
            return float(self) / float(other)
        if isinstance(other, ComplexMeasurement):
            return self.to_complex() / other
        return Measurement(0, self.unit, base_magnitude=float(self) / float(other))

    def __floordiv__(self, other):
        if isinstance(other, Measurement):
            return float(self) // float(other)
        if isinstance(other, ComplexMeasurement):
            return self.to_complex() // other
        return Measurement(0, self.unit, base_magnitude=float(self) // float(other))

    def __mod__(self, other):
        return Measurement(0, self.unit, base_magnitude=float(self) % float(other))

    def __neg__(self):
        return Measurement(0, self.unit, base_magnitude=-float(self))

    def __abs__(self):
        return Measurement(0, self.unit, base_magnitude=abs(float(self)))

    def __pow__(self, other):  # Make into complex measurement
        if isinstance(other, Measurement):
            Error("Cannot raise measurement to power of another measurment")
        return self.to_complex(pwr=float(other))

    def to_complex(self, pwr: int = 1) -> "ComplexMeasurement":
        return ComplexMeasurement(
            0,
            unit=ComplexUnit({self.get_unit(): pwr}),
            base_magnitude=float(self) ** pwr,
        )


class ComplexDimension(np.ndarray):
    def __new__(cls, dimension: np.ndarray):
        obj = np.asarray(dimension).view(cls)
        return obj

    def __str__(self):
        return f"{self.get_symbol()}"

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other: np.ndarray) -> bool:
        return super().__eq__(other).all()

    def __ne__(self, other: np.ndarray) -> bool:
        return not (self == other).all()

    def __mul__(self, other: np.ndarray) -> np.ndarray:
        return super().__add__(other)

    def __div__(self, other: np.ndarray) -> np.ndarray:
        return super().__sub__(other)

    def __pow__(self, other) -> np.ndarray:
        return super().__pow__(other)

    def __add__(self, other: np.ndarray) -> np.ndarray:
        raise NotImplementedError("Cannot add dimensions")

    def __sub__(self, other: np.ndarray) -> np.ndarray:
        raise NotImplementedError("Cannot subtract dimensions")

    @classmethod
    def zeroes(cls, length: int):
        return cls(np.zeros(length))

    def from_base(self, value: float, map: "ComplexDimensionMap") -> np.ndarray:
        for i, pwr in enumerate(self):
            if pwr == 0:
                continue
            value /= map[i].get_scale() ** pwr
        return value


# Stores the mapping of dimensions to singular units
class ComplexDimensionMap(dict):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, key: Dimension) -> Unit:
        return super().__getitem__(key)

    def __setitem__(self, key: Dimension, value: Unit) -> None:
        super().__setitem__(key, value)

    def __delitem__(self, key: Dimension) -> None:
        super().__delitem__(key)

    def __iter__(self):
        return super().__iter__()

    def __len__(self):
        return super().__len__()

    def __str__(self):
        return super().__str__()

    def __repr__(self):
        return super().__repr__()

    def merge_with(self, other: "ComplexDimensionMap") -> None:
        for key, value in other.items():
            if not key in self:
                self[key] = value

    def get_symbol(self, dimension: ComplexDimension) -> str:
        num = []
        den = []
        for i, pwr in enumerate(dimension):
            if pwr == 0:
                continue
            symbol = self.units[i].get_symbol()
            if pwr > 0:
                if pwr == 1:
                    num.append(symbol)
                else:
                    num.append(f"{symbol}^{pwr}")
            else:
                if pwr == -1:
                    den.append(symbol)
                else:
                    den.append(f"{symbol}^{abs(pwr)}")
        top = " * ".join(num)
        # TODO: make this cleaner
        if num == [] and den == []:
            return ""
        if num == []:
            top = "1"
        if den == []:
            return top
        bottom = " * ".join(den)
        return f"{top}/{bottom}"


class ComplexUnit(Unit):
    ALIASES = {}  # Possible future speedup if necessary

    def __init__(
        self,
        units: Dict[Unit, int],
    ) -> None:

        dimensionality = ComplexDimension.zeroes(len(Dimension))
        units: Dict[int, Unit] = {}
        for unit, power in units.items():
            dimension = unit.get_dimension()
            index = dimension.value
            dimensionality[index] += power

            if not index in self.units:
                units[index] = unit

        self.units = ComplexDimensionMap(units)
        self.dimension = dimensionality

    def __eq__(self, value: object) -> bool:
        if isinstance(value, ComplexUnit):
            return (value.dimension == self.dimension).all()
        return False

    def to_base(
        self, value: float
    ) -> float:  # TODO: Account for shift, not just scale, in units
        value = float(value)
        for i, pwr in enumerate(self.dimension):
            if pwr == 0:
                continue
            value *= self.units[i].get_scale() ** pwr
        return value

    def from_base(self, value: float) -> float:
        value = float(value)
        for i, pwr in enumerate(self.dimension):
            if pwr == 0:
                continue
            value /= self.units[i].get_scale() ** pwr
        return value

    def get_dimension(self) -> ComplexDimension:
        return self.dimension.copy()

    def get_dimension_map(self) -> ComplexDimensionMap:
        return ComplexDimensionMap(self.units.copy())


class ComplexMeasurement(float):
    def __new__(
        cls,
        magnitude: float,
        unit: ComplexUnit,
        base_magnitude=None,
        dimension=None,
        dimension_map=None,
    ):
        if base_magnitude:
            return float.__new__(cls, base_magnitude)
        if isinstance(magnitude, ComplexMeasurement):
            return magnitude
        value = unit.to_base(magnitude)
        return float.__new__(cls, value)

    def __init__(
        self,
        magnitude: float,
        unit: ComplexUnit,
        base_magnitude=None,
        dimension=None,
        dimension_map=None,
    ) -> None:
        super().__init__()
        if not dimension is None and not dimension_map is None:
            self.dimension = dimension
            self.dimension_map = dimension_map
        else:
            self.dimension = unit.get_dimension()
            self.dimension_map = unit.get_dimension_map()

    def __deepcopy__(self, memo):
        return ComplexMeasurement(0, self.unit, float(self))

    def to(self, unit: ComplexUnit):
        return unit.from_base(self)

    def get_unit_magnitude(self) -> float:
        return self.dimension.from_base(float(self), self.dimension_map)

    def __str__(self) -> str:
        Debug(f"Dimension: {self.dimension_map}")
        return f"{self.get_unit_magnitude()} {self.dimension_map.get_symbol(self.dimension)}"

    def __repr__(self) -> str:
        return self.__str__()

    def get_dimension(self):
        return self.dimension

    def to_complex(self):
        return self

    def __is_same_dimension(self, other: "ComplexMeasurement"):
        return other.get_dimension() == self.get_dimension()

    @classmethod
    def __is_a_measurement(cls, other):
        return isinstance(other, ComplexMeasurement)

    def __add__(self, other) -> float:
        if ComplexMeasurement.__is_a_measurement(other) and self.__is_same_dimension(
            other
        ):
            return ComplexMeasurement(
                0,
                None,
                base_magnitude=float(self) + float(other),
                dimension=self.dimension,
                dimension_map=self.dimension_map,
            )
        else:
            Error("Cannot add measurements of different dimensions")

    def __sub__(self, other) -> float:
        if ComplexMeasurement.__is_a_measurement(other) and self.__is_same_dimension(
            other
        ):
            return ComplexMeasurement(
                0,
                None,
                base_magnitude=float(self) - float(other),
                dimension=self.dimension,
                dimension_map=self.dimension_map,
            )
        else:
            Error("Cannot add measurements of different dimensions")

    def __mul__(self, other):  # Make complex units
        if isinstance(other, Measurement):
            other = other.to_complex()
        if isinstance(other, ComplexMeasurement):
            if self.__is_same_dimension(other):
                return float(self) * float(other)
            else:
                return ComplexMeasurement(
                    0,
                    None,
                    base_magnitude=float(self) * float(other),
                    dimension=self.dimension * other.dimension,
                    dimension_map=self.dimension_map,
                )
        return ComplexMeasurement(
            0,
            None,
            base_magnitude=float(self) * float(other),
            dimension=self.dimension,
            dimension_map=self.dimension_map,
        )

    def __truediv__(self, other):
        if isinstance(other, Measurement):
            other = other.to_complex()
        if isinstance(other, ComplexMeasurement):
            if self.__is_same_dimension(other):
                return float(self) / float(other)
            else:
                return ComplexMeasurement(
                    0,
                    None,
                    base_magnitude=float(self) / float(other),
                    dimension=self.dimension / other.dimension,
                    dimension_map=self.dimension_map,
                )
        return ComplexMeasurement(
            0,
            None,
            base_magnitude=float(self) / float(other),
            dimension=self.dimension,
            dimension_map=self.dimension_map,
        )

    def __floordiv__(self, other):
        if isinstance(other, Measurement):
            other = other.to_complex()
        if isinstance(other, ComplexMeasurement):
            if self.__is_same_dimension(other):
                return float(self) // float(other)
            else:
                return ComplexMeasurement(
                    0,
                    None,
                    base_magnitude=float(self) // float(other),
                    dimension=self.dimension / other.dimension,
                    dimension_map=self.dimension_map,
                )
        return ComplexMeasurement(
            0,
            None,
            base_magnitude=float(self) // float(other),
            dimension=self.dimension,
            dimension_map=self.dimension_map,
        )

    def __mod__(self, other):
        return ComplexMeasurement(
            0, None, float(self) % float(other), self.dimension, self.dimension_map
        )

    def __neg__(self):
        return ComplexMeasurement(
            0, None, -float(self), self.dimension, self.dimension_map
        )

    def __abs__(self):
        return ComplexMeasurement(
            0, None, abs(float(self)), self.dimension, self.dimension_map
        )

    def __pow__(self, other):  # Make into complex measurement
        if isinstance(other, ComplexMeasurement):
            Error("Cannot raise measurement to power of another measurment")
        return ComplexMeasurement(
            0,
            None,
            base_magnitude=float(self) ** float(other),
            dimension=self.dimension ** float(other),
            dimension_map=self.dimension_map,
        )
