from dataclasses import dataclass
from typing import Generic, TypeVar, Union

import numpy as np

from gamegine.utils.NCIM.basic import ComplexMeasurement, Measurement


T = TypeVar("T", bound=Union[Measurement, ComplexMeasurement])


@dataclass
class Vector2D(Generic[T]):
    """Class representing a 2D vector."""

    x: T
    y: T

    def __add__(self, other: "Vector2D") -> "Vector2D":
        """Adds two 2D vectors."""
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector2D") -> "Vector2D":
        """Subtracts two 2D vectors."""
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: T) -> "Vector2D":
        """Multiplies a 2D vector by a scalar."""
        return Vector2D(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar: T) -> "Vector2D":
        """Divides a 2D vector by a scalar."""
        return Vector2D(self.x / scalar, self.y / scalar)

    def __neg__(self) -> "Vector2D":
        """Negates a 2D vector."""
        return Vector2D(-self.x, -self.y)

    def __eq__(self, other: "Vector2D") -> bool:
        """Compares two 2D vectors for equality."""
        return self.x == other.x and self.y == other.y

    def __ne__(self, other: "Vector2D") -> bool:
        """Compares two 2D vectors for inequality."""
        return not self == other

    def __str__(self) -> str:
        """Returns a string representation of the 2D vector."""
        return f"({self.x}, {self.y})"

    def __repr__(self) -> str:
        """Returns a string representation of the 2D vector."""
        return str(self)

    def to(self, unit: T) -> "Vector2D":
        """Converts the 2D vector to the specified unit."""
        return (self.x.to(unit), self.y.to(unit))

    def to_np(self, unit: T) -> "Vector2D":
        """Converts the 2D vector to a numpy array."""
        return np.array([self.x.to(unit), self.y.to(unit)])

    def magnitude(self) -> T:
        """Returns the magnitude of the 2D vector."""
        return (self.x**2 + self.y**2) ** 0.5

    def reflect_x(self) -> "Vector2D":
        """Reflects the 2D vector over the x-axis."""
        return Vector2D(self.x, -self.y)

    def reflect_y(self) -> "Vector2D":
        """Reflects the 2D vector over the y-axis."""
        return Vector2D(-self.x, self.y)


@dataclass(frozen=True)
class Vector3D(Vector2D[T]):
    """Class representing a 3D vector."""

    z: T

    def __add__(self, other: "Vector3D") -> "Vector3D":
        """Adds two 3D vectors."""
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3D") -> "Vector3D":
        """Subtracts two 3D vectors."""
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: T) -> "Vector3D":
        """Multiplies a 3D vector by a scalar."""
        return Vector3D(self.x * scalar, self.y * scalar, self.z * scalar)

    def __truediv__(self, scalar: T) -> "Vector3D":
        """Divides a 3D vector by a scalar."""
        return Vector3D(self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self) -> "Vector3D":
        """Negates a 3D vector."""
        return Vector3D(-self.x, -self.y, -self.z)

    def __eq__(self, other: "Vector3D") -> bool:
        """Compares two 3D vectors for equality."""
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __ne__(self, other: "Vector3D") -> bool:
        """Compares two 3D vectors for inequality."""
        return not self == other

    def __str__(self) -> str:
        """Returns a string representation of the 3D vector."""
        return f"({self.x}, {self.y}, {self.z})"

    def __repr__(self) -> str:
        """Returns a string representation of the 3D vector."""
        return str(self)

    def to(self, unit: T) -> "Vector3D":
        """Converts the 3D vector to the specified unit."""
        return (self.x.to(unit), self.y.to(unit), self.z.to(unit))

    def __getitem__(self, key: int) -> T:
        """Returns the value at the specified index."""
        if key == 0:
            return self.x
        elif key == 1:
            return self.y
        elif key == 2:
            return self.z
        else:
            raise IndexError("Index out of range")

    def __setitem__(self, key: int, value: T):
        """Sets the value at the specified index."""
        if key == 0:
            self.x = value
        elif key == 1:
            self.y = value
        elif key == 2:
            self.z = value
        else:
            raise IndexError("Index out of range")

    def to_np(self, unit: T) -> "Vector3D":
        """Converts the 3D vector to a numpy array."""
        return np.array([self.x.to(unit), self.y.to(unit), self.z.to(unit)])

    def magnitude(self) -> T:
        """Returns the magnitude of the 3D vector."""
        return (self.x**2 + self.y**2 + self.z**2) ** 0.5
