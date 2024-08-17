from copy import deepcopy
from enum import Enum
from abc import ABC, abstractmethod
from typing import Dict

from gamegine.utils.NCIM.dimension import Dimension
from gamegine.utils.logging import Debug, Error, Warn
import numpy as np


class Unit(object):
    """A class representing a simple unit, defined as being made up of a single dimension with a power of 1, such as meters or seconds. Contains information about scale and shift in relation to a base unit in the same dimension to allow for conversions.

    :param dimension: The dimension of the unit.
    :type dimension: :class:`Dimension`
    :param scale: The scale of the unit in relation to an arbitrary base unit in the same dimension.
    :type scale: float, optional
    :param symbol: The symbol of the unit.
    :type symbol: str, optional
    :param shift: The shift of the unit in relation to the base unit in the same dimension.
    :type shift: float, optional
    """

    ROUNDING = 9

    def __init__(
        self, dimension: "Dimension", scale: float = 0, symbol="", shift: float = 0
    ) -> None:
        self.scale = scale
        self.shift = shift
        self.symbol = symbol
        self.dimension = dimension

    def to_base(self, value: float) -> float:
        """Converts a value from the unit to the base unit in the same dimension. Additionally, the value is rounded in order to prevent floating point errors.

        :param value: The value to convert.
        :type value: float
        :return: The converted value.
        :rtype: float
        """
        return round(float(value) * self.scale + self.shift, Unit.ROUNDING)

    def from_base(self, value: float) -> float:
        """Converts a value from the base unit in the same dimension to the unit. Additionally, the value is rounded in order to prevent floating point errors.

        :param value: The value to convert.
        :type value: float
        :return: The converted value.
        :rtype: float
        """

        return round(float(value) / self.scale - self.shift, Unit.ROUNDING)

    def get_dimension(self) -> "Dimension":
        """Returns the dimension of the unit.

        :return: The dimension of the unit.
        :rtype: :class:`Dimension`
        """
        return self.dimension

    def get_symbol(self) -> str:
        """Returns the symbol of the unit. Used when printing measurements.

        :return: The symbol of the unit.
        :rtype: str
        """
        return self.symbol

    def get_scale(self):
        """Returns the scale of the unit.

        :return: The scale of the unit.
        :rtype: float
        """
        return self.scale


class Measurement(float):
    """A class representing a simple measurement, defined as a value with a unit. Contains information about the dimension and unit of the measurement. Represent a value in a single dimension, such as 5 meters or 10 seconds.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`Unit`
    :param base_magnitude: The magnitude of the measurement in the base unit of the same dimension. Used for internal conversions.
    :type base_magnitude: float, optional
    """

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

    def to(self, unit: Unit) -> float:
        """Converts the measurement to a different unit.

        :param unit: The unit to convert to.
        :type unit: :class:`Unit`
        :return: The converted measurement.
        :rtype: float
        """

        return unit.from_base(self)

    def get_unit_magnitude(self) -> float:
        """Returns the magnitude of the measurement in the unit.

        :return: The magnitude of the measurement in the unit.
        :rtype: float
        """

        return self.unit.from_base(self)

    def __str__(self) -> str:
        return f"{self.get_unit_magnitude()} {self.unit.get_symbol()}"

    def __repr__(self) -> str:
        return self.__str__()

    def get_dimension(self) -> Dimension:
        """Returns the dimension of the measurement.

        :return: The dimension of the measurement.
        :rtype: :class:`Dimension`
        """
        return self.dimension

    def get_unit(self) -> Unit:
        """Returns the unit of the measurement.

        :return: The unit of the measurement.
        :rtype: :class:`Unit`
        """

        return self.unit

    def __is_same_dimension(self, other):
        """Checks if another measurement is of the same dimension as this measurement.

        :param other: The other measurement to compare.
        :type other: :class:`Measurement`
        :return: True if the measurements are of the same dimension, False otherwise.
        :rtype: bool
        """

        return other.get_dimension() == self.get_dimension()

    @classmethod
    def __is_a_measurement(cls, other):
        """Checks if an object is a measurement.

        :param other: The object to check.
        :type other: object
        :return: True if the object is a measurement, False otherwise.
        :rtype: bool
        """
        return isinstance(other, Measurement)

    def __add__(self, other) -> float:
        """Adds two measurements together. If the measurements are of the same dimension, the result is a new measurement. If the measurements are of different dimensions, an error is raised. If the other measurement is a complex measurement, the measurment is attempted to be converted to a complex measurement and added.

        :param other: The other measurement to add.
        :type other: :class:`Measurement`
        :return: The sum of the measurements.
        :rtype: float
        """

        if isinstance(other, ComplexMeasurement):
            return self.to_complex() + other
        elif Measurement.__is_a_measurement(other) and self.__is_same_dimension(other):
            return Measurement(0, self.unit, base_magnitude=float(self) + float(other))
        else:
            Error("Cannot add measurements of different dimensions")

    def __sub__(self, other) -> float:
        """Subtracts two measurements. If the measurements are of the same dimension, the result is a new measurement. If the measurements are of different dimensions, an error is raised. If the other measurement is a complex measurement, the measurment is attempted to be converted to a complex measurement and subtracted.

        :param other: The other measurement to subtract.
        :type other: :class:`Measurement`
        :return: The difference of the measurements.
        :rtype: float
        """
        if Measurement.__is_a_measurement(other) and self.__is_same_dimension(other):
            return Measurement(0, self.unit, base_magnitude=float(self) - float(other))
        else:
            Error("Cannot add measurements of different dimensions")

    def __mul__(self, other):  # Make complex units
        """Multiplies two measurements. If the other measurement is a complex measurement, the measurment is attempted to be converted to a complex measurement and multiplied. If the measurements are of the same dimension, the result is a new measurement. If the measurements are of different dimensions, the result is a new measurement with the dimensions multiplied.

        :param other: The other measurement to multiply.
        :type other: :class:`Measurement`
        :return: The product of the measurements.
        :rtype: float
        """
        if isinstance(other, Measurement) or isinstance(other, ComplexMeasurement):
            return self.to_complex() * other.to_complex()
        else:
            return Measurement(0, self.unit, base_magnitude=float(self) * float(other))

    def __truediv__(self, other):
        """Divides two measurements. If the other measurement is a complex measurement, the measurment is attempted to be converted to a complex measurement and divided. If the measurements are of the same dimension, the result is a new measurement. If the measurements are of different dimensions, the result is a new measurement with the dimensions divided.

        :param other: The other measurement to divide.
        :type other: :class:`Measurement`
        :return: The quotient of the measurements.
        :rtype: float
        """

        if isinstance(other, Measurement):
            return float(self) / float(other)
        if isinstance(other, ComplexMeasurement):
            return self.to_complex() / other
        return Measurement(0, self.unit, base_magnitude=float(self) / float(other))

    def __floordiv__(self, other):
        """Floors the division of two measurements. If the other measurement is a complex measurement, the measurment is attempted to be converted to a complex measurement and divided. If the measurements are of the same dimension, the result is a new measurement. If the measurements are of different dimensions, the result is a new measurement with the dimensions divided.

        :param other: The other measurement to divide.
        :type other: :class:`Measurement`
        :return: The floored quotient of the measurements.
        :rtype: float
        """

        if isinstance(other, Measurement):
            return float(self) // float(other)
        if isinstance(other, ComplexMeasurement):
            return self.to_complex() // other
        return Measurement(0, self.unit, base_magnitude=float(self) // float(other))

    def __mod__(self, other):
        """Returns measurement % other"""
        return Measurement(0, self.unit, base_magnitude=float(self) % float(other))

    def __neg__(self):
        """Negates the measurement."""
        return Measurement(0, self.unit, base_magnitude=-float(self))

    def __abs__(self):
        """Returns the absolute value of the measurement."""
        return Measurement(0, self.unit, base_magnitude=abs(float(self)))

    def __pow__(self, other) -> "ComplexMeasurement":
        """Raises the measurement to the power of another value. If the other value is a complex measurement, an error is raised. If the measurements are of the same dimension, the result is a new measurement. If the measurements are of different dimensions, the result is a new measurement with the dimensions raised to the power of the other value.

        :param other: The other value to raise the measurement to.
        :type other: float
        :return: The measurement raised to the power of the other value.
        :rtype: :class:`ComplexMeasurement`
        """
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
    """A class representing a complex dimension. Stores information about dimensionality of a unit or measurement in the form of an array of powers of dimensions. Wraps a numpy array to allow for easy manipulation and comparison of dimensions.

    :param dimension: The dimension of the unit or measurement.
    :type dimension: np.ndarray
    """

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
        """Adds the values of the dimensions together pairwise, which is the equivalent of performing dimensional analysis when multiplying two measurements.

        :param other: The other dimension to multiply with.
        :type other: np.ndarray
        :return: The multiplied dimensions.
        :rtype: :class:`ComplexDimension`
        """
        return ComplexDimension(super().__add__(other))

    def __div__(self, other: np.ndarray) -> np.ndarray:
        """Subtracts the values of the dimensions together pairwise, which is the equivalent of performing dimensional analysis when dividing two measurements.

        :param other: The other dimension to divide with.
        :type other: np.ndarray
        :return: The divided dimensions.
        :rtype: :class:`ComplexDimension`
        """

        return ComplexDimension(super().__sub__(other))

    def __pow__(self, other) -> np.ndarray:
        """Raises the dimensions to the power of another value.

        :param other: The value to raise the dimensions to.
        :type other: float
        :return: The dimensions raised to the power of the other value.
        :rtype: :class:`ComplexDimension`
        """
        return ComplexDimension(super().__mul__(other))

    def __add__(self, other: np.ndarray) -> np.ndarray:
        raise NotImplementedError("Cannot add dimensions")

    def __sub__(self, other: np.ndarray) -> np.ndarray:
        raise NotImplementedError("Cannot subtract dimensions")

    @classmethod
    def zeroes(cls, length: int):
        """Returns a dimension with all values set to zero.

        :param length: The length of the dimension.
        :type length: int
        :return: The zeroed dimension.
        :rtype: :class:`ComplexDimension`
        """

        return cls(np.zeros(length))

    def from_base(self, value: float, map: "ComplexDimensionMap") -> np.ndarray:
        """Converts a value from the base unit in the same dimension to the unit.

        :param value: The value to convert.
        :type value: float
        :param map: The mapping of dimensions to units.
        :type map: :class:`ComplexDimensionMap`
        :return: The converted value.
        :rtype: float
        """

        for i, pwr in enumerate(self):
            if pwr == 0:
                continue
            value /= map[i].get_scale() ** pwr
        return value


# Stores the mapping of dimensions to singular units
class ComplexDimensionMap(dict):
    """A class representing a mapping of dimensions to singular units. Contains information which maps dimensions to units, allowing for maintaining provided units in the creation of :class:`ComplexUnit` and :class:`ComplexMeasurement` objects.

    :param units: The mapping of dimensions to units.
    :type units: Dict[:class:`Unit`, int]
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getitem__(self, key: Dimension) -> Unit:
        """Returns the unit associated with a dimension.

        :param key: The dimension to get the unit for.
        :type key: :class:`Dimension`
        :return: The unit associated with the dimension.
        :rtype: :class:`Unit`
        """
        return super().__getitem__(key)

    def __setitem__(self, key: Dimension, value: Unit) -> None:
        """Sets the unit associated with a dimension.

        :param key: The dimension to set the unit for.
        :type key: :class:`Dimension`
        :param value: The unit to associate with the dimension.
        :type value: :class:`Unit`
        """
        super().__setitem__(key, value)

    def __delitem__(self, key: Dimension) -> None:
        """Deletes the unit associated with a dimension.

        :param key: The dimension to delete the unit for.
        :type key: :class:`Dimension`
        """
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
        """Merges the mapping with another mapping. If a dimension is not present in the current mapping, it is added. This is used when performing operations between different :class:`ComplexMeasurement` objects.

        :param other: The other mapping to merge with.
        :type other: :class:`ComplexDimensionMap`
        """
        for key, value in other.items():
            if not key in self:
                self[key] = value

    def get_symbol(self, dimension: ComplexDimension) -> str:
        """Returns the symbol of the dimension, based on the degrees of dimensions in the provided :class:`ComplexDimension`. For example, meters per second squared would be represented with a symbol of m/s^2.

        :param dimension: The dimension to get the symbol for.
        :type dimension: :class:`ComplexDimension`
        :return: The symbol of the dimension.
        :rtype: str
        """

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
    """A class representing a complex unit, defined as being made up of multiple dimensions with arbitrary powers, such as meters per second squared. Contains information about scale and shift in relation to a base unit in the same dimension to allow for conversions.  Used during the initial creation of a :class:`ComplexMeasurement` and for converting out of a base :class:`ComplexMeasurement`.

    :param units: The mapping of dimensions to powers.
    :type units: Dict[:class:`Unit`, int]
    """

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
        """Checks if another object is equal to this complex unit. If the other object is a complex unit, the dimensions are compared, returning True if they are the same."""
        if isinstance(value, ComplexUnit):
            return (value.dimension == self.dimension).all()
        return False

    def to_base(
        self, value: float
    ) -> float:  # TODO: Account for shift, not just scale, in units
        """Converts a value from the :class:`ComplexUnit` to the base unit in the same dimension. Additionally, the value is rounded in order to prevent floating point errors.

        :param value: The value to convert.
        :type value: float
        :return: The converted value.
        :rtype: float
        """
        value = float(value)
        for i, pwr in enumerate(self.dimension):
            if pwr == 0:
                continue
            value *= self.units[i].get_scale() ** pwr
        return value

    def from_base(self, value: float) -> float:
        """Converts from the base value to the :class:`ComplexUnit`. Additionally, the value is rounded in order to prevent floating point errors."""
        value = float(value)
        for i, pwr in enumerate(self.dimension):
            if pwr == 0:
                continue
            value /= self.units[i].get_scale() ** pwr
        return value

    def get_dimension(self) -> ComplexDimension:
        return self.dimension.copy()

    def get_dimension_map(self) -> ComplexDimensionMap:
        """Returns copy of the mapping of dimensions to units.

        :return: The mapping of dimensions to units.
        :rtype: :class:`ComplexDimensionMap`
        """
        return ComplexDimensionMap(self.units.copy())


class ComplexMeasurement(float):
    """A class representing a complex measurement, which is defined with a magnitude and a complex unit. Contains information about the dimension and unit of the measurement. Represents a value in multiple dimensions, such as 5 meters per second squared or 10 Nm.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`ComplexUnit`
    :param base_magnitude: The magnitude of the measurement in the base unit of the same dimension. Used for internal creations.
    :type base_magnitude: float, optional
    :param dimension: The dimension of the measurement, used for internal creations.
    :type dimension: :class:`ComplexDimension`, optional
    :param dimension_map: The mapping of dimensions to units, used for internal creations.
    :type dimension_map: :class:`ComplexDimensionMap`, optional
    """

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

    def to(self, unit: ComplexUnit) -> float:
        """Converts the measurement to a different unit.

        :param unit: The unit to convert to.
        :type unit: :class:`ComplexUnit`
        :return: The converted measurement.
        :rtype: float
        """
        return unit.from_base(self)

    def get_unit_magnitude(self) -> float:
        """Returns the magnitude of the measurement in the unit.

        :return: The magnitude of the measurement in the unit.
        :rtype: float
        """
        return self.dimension.from_base(float(self), self.dimension_map)

    def __str__(self) -> str:
        Debug(f"Dimension: {self.dimension_map}")
        return f"{self.get_unit_magnitude()} {self.dimension_map.get_symbol(self.dimension)}"

    def __repr__(self) -> str:
        return self.__str__()

    def get_dimension(self):
        """Returns the dimension of the measurement.

        :return: The dimension of the measurement.
        :rtype: :class:`ComplexDimension`
        """
        return self.dimension

    def to_complex(self):
        """Returns the complex measurement object, used for the purposes of interoperability between :class:`Measurement` and :class:`ComplexMeasurement` objects."""
        return self

    def __is_same_dimension(self, other: "ComplexMeasurement"):
        """Checks if another measurement is of the same dimension as this measurement.

        :param other: The other measurement to compare.
        :type other: :class:`ComplexMeasurement`
        :return: True if the measurements are of the same dimension, False otherwise.
        :rtype: bool
        """
        return other.get_dimension() == self.get_dimension()

    @classmethod
    def __is_a_measurement(cls, other):
        """Checks if an object is a :class:`ComplexMeasurement`.

        :param other: The object to check.
        :type other: object
        :return: True if the object is a measurement, False otherwise.
        :rtype: bool
        """
        return isinstance(other, ComplexMeasurement)

    def __add__(self, other) -> float:
        """Adds two measurements together. If the measurements are of the same dimension, the result is a new measurement. If the measurements are of different dimensions, an error is raised.

        :param other: The other measurement to add.
        :type other: :class:`ComplexMeasurement`
        :return: The sum of the measurements.
        :rtype: float
        """
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
        """Subtracts two measurements. If the measurements are of the same dimension, the result is a new measurement. If the measurements are of different dimensions, an error is raised.

        :param other: The other measurement to subtract.
        :type other: :class:`ComplexMeasurement`
        :return: The difference of the measurements.
        :rtype: float
        """
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
        """Multiplies two measurements. If the measurements are of the same dimension, the result is a new measurement. If the measurements are of different dimensions, the result is a new measurement with the dimensions multiplied through dimensional analysis.

        :param other: The other measurement to multiply.
        :type other: :class:`ComplexMeasurement`
        :return: The product of the measurements.
        :rtype: float
        """
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
        """Divides two measurements. If the measurements are of the same dimension, the result is a new measurement. If the measurements are of different dimensions, the result is a new measurement with the dimensions divided through dimensional analysis.

        :param other: The other measurement to divide.
        :type other: :class:`ComplexMeasurement`
        :return: The quotient of the measurements.
        :rtype: float
        """
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
        """Floors the division of two measurements. If the measurements are of the same dimension, the result is a new measurement. If the measurements are of different dimensions, the result is a new measurement with the dimensions divided through dimensional analysis.

        :param other: The other measurement to divide.
        :type other: :class:`ComplexMeasurement`
        :return: The floored quotient of the measurements.
        :rtype: float"""
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
        """Raises the measurement to the power of another value, multiplying the dimensions by the power. If the other value is a complex measurement, an error is raised.

        :param other: The value to raise the measurement to.
        :type other: float
        :return: The measurement raised to the power of the other value.
        :rtype: :class:`ComplexMeasurement`
        """

        if isinstance(other, ComplexMeasurement):
            Error("Cannot raise measurement to power of another measurment")
        return ComplexMeasurement(
            0,
            None,
            base_magnitude=float(self) ** float(other),
            dimension=self.dimension ** float(other),
            dimension_map=self.dimension_map,
        )
