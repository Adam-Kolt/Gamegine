from dataclasses import dataclass
from typing import Tuple

import numpy as np
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement
from gamegine.utils.NCIM.Dimensions.spatial import Feet, Meter
from typing import List
import math
from gamegine.utils.NCIM.Dimensions.angular import Radian
from gamegine.utils.NCIM.vector import Vector2D, Vector3D
import quaternion


class Translation3D(Vector3D[SpatialMeasurement]):
    """Class representing a 3D translation."""

    def apply(
        self,
        points: List[Vector3D[SpatialMeasurement]],
    ) -> List[Vector3D[SpatialMeasurement]]:
        """Applies the translation to a list of 3D points."""
        return [point + self for point in points]


@dataclass
class Scale3D:
    """Class representing a 3D scale."""

    x: float
    y: float
    z: float

    def apply(
        self,
        points: List[Vector3D[SpatialMeasurement]],
    ) -> List[Vector3D[SpatialMeasurement]]:
        """Applies the scale to a list of 3D points."""
        return [
            Vector3D(point.x * self.x, point.y * self.y, point.z * self.z)
            for point in points
        ]


class Rotation3D:
    """Class representing a 3D rotation."""

    def __init__(self, quaternion):
        self.quaternion = quaternion

    @classmethod
    def fromEuler(yaw, pitch, roll):
        """Creates a 3D rotation from Euler angles."""
        return Rotation3D(quaternion.from_euler_angles(yaw, pitch, roll))

    @classmethod
    def fromQuaternionComponents(w, x, y, z):
        """Creates a 3D rotation from quaternion components."""
        return Rotation3D(quaternion.from_float_array([w, x, y, z]))

    @property
    def euler(self) -> Vector3D[AngularMeasurement]:
        """Returns the Euler angles of the rotation."""
        return Vector3D(
            *[Radian(angle) for angle in quaternion.as_euler_angles(self.quaternion)]
        )

    @property
    def yaw(self) -> AngularMeasurement:
        """Returns the yaw angle of the rotation."""
        return self.euler.z

    @property
    def pitch(self) -> AngularMeasurement:
        """Returns the pitch angle of the rotation."""
        return self.euler.x

    @property
    def roll(self) -> AngularMeasurement:
        """Returns the roll angle of the rotation."""
        return self.euler.y

    def __add__(self, other):
        """Adds two 3D rotations."""
        return Rotation3D(self.quaternion * other.quaternion)

    def __sub__(self, other):
        """Subtracts two 3D rotations."""
        return Rotation3D(self.quaternion * other.quaternion.inverse())

    def apply(
        self,
        points: List[Vector3D[SpatialMeasurement]],
        center: Vector3D[SpatialMeasurement] = Vector3D(Meter(0), Meter(0), Meter(0)),
    ) -> List[Vector3D[SpatialMeasurement]]:
        """Applies the rotation to a list of 3D points."""
        transformed_points = []
        for point in points:
            p = np.array([component.to(Meter) for component in point])
            # Apply centering
            p = p - np.array([component.to(Meter) for component in center])
            # Apply rotation
            p = quaternion.rotate_vectors(self.quaternion, p)
            # Revert centering
            p = p + np.array([component.to(Meter) for component in center])
            transformed_points.append(Vector3D(Meter(p[0]), Meter(p[1]), Meter(p[2])))
        return transformed_points


class Transform3D:
    """Class representing a 3D transformation including position, rotation, and scale."""

    def __init__(
        self,
        position: Translation3D = Translation3D(Meter(0), Meter(0), Meter(0)),
        rotation: Rotation3D = Rotation3D.fromEuler(0, 0, 0),
        scale: Scale3D = Scale3D(1, 1, 1),
    ):
        self.position = position
        self.rotation = rotation
        self.scale = scale

    def getRotation(self):
        return self.rotation

    def setRotation(self, rotation):
        self.rotation = rotation

    def getPosition(self):
        return self.position

    def setPosition(self, position):
        self.position = position

    def apply(
        self,
        points: List[Vector3D[SpatialMeasurement]],
        center: Vector3D[SpatialMeasurement] = Vector3D(Meter(0), Meter(0), Meter(0)),
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement, SpatialMeasurement]]:
        """Applies the transformation to a list of 3D points."""

        scaled = self.scale.apply(points)
        rotated = self.rotation.apply(scaled, center)
        translated = self.position.apply(rotated)

        return translated

    def translate(self, translation: Translation3D):
        self.position += translation


class Translation2D(Vector2D[SpatialMeasurement]):
    """Class representing a 2D translation."""

    def apply(
        self,
        points: List[Vector2D[SpatialMeasurement]],
    ) -> List[Vector2D[SpatialMeasurement]]:
        """Applies the translation to a list of 2D points."""
        return [point + self for point in points]


class Rotation2D:
    """Class representing a 2D rotation."""

    def __init__(self, angle: AngularMeasurement = Radian(0)):
        self.angle = angle

    def apply(
        self,
        points: List[Vector2D[SpatialMeasurement]],
        center: Vector2D[SpatialMeasurement] = Vector2D(Meter(0), Meter(0)),
    ) -> List[Vector2D[SpatialMeasurement]]:
        """Applies the rotation to a list of 2D points."""
        transformed_points = []
        for point in points:
            p = np.array([component.to(Meter) for component in point])
            # Apply centering
            p = p - np.array([component.to(Meter) for component in center])
            # Apply rotation
            p = np.dot(
                np.array(
                    [
                        [
                            math.cos(self.angle.to(Radian)),
                            -math.sin(self.angle.to(Radian)),
                        ],
                        [
                            math.sin(self.angle.to(Radian)),
                            math.cos(self.angle.to(Radian)),
                        ],
                    ]
                ),
                p,
            )
            # Revert centering
            p = p + np.array([component.to(Meter) for component in center])
            transformed_points.append(Vector2D(Meter(p[0]), Meter(p[1])))
        return transformed_points


@dataclass
class Scale2D:
    """Class representing a 2D scale."""

    x: float
    y: float

    def apply(
        self,
        points: List[Vector2D[SpatialMeasurement]],
    ) -> List[Vector2D[SpatialMeasurement]]:
        """Applies the scale to a list of 2D points."""
        return [Vector2D(point.x * self.x, point.y * self.y) for point in points]


class Transform2D:
    """Class representing a 2D transformation including position, rotation, and scale."""

    def __init__(
        self,
        position: Translation2D = Translation2D(Meter(0), Meter(0)),
        rotation: Rotation2D = Rotation2D(),
        scale: Scale2D = Scale2D(1, 1),
    ):
        self.position = position
        self.rotation = rotation
        self.scale = scale

    def apply(
        self,
        points: List[Vector2D[SpatialMeasurement]],
        center: Vector2D[SpatialMeasurement] = Vector2D(Meter(0), Meter(0)),
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        """Applies the transformation to a list of 2D points."""

        scaled = self.scale.apply(points)
        rotated = self.rotation.apply(scaled, center)
        translated = self.position.apply(rotated)

        return translated

    def toTransform3D(self):
        return Transform3D(
            position=Translation3D(self.position.x, self.position.y, Meter(0)),
            rotation=Rotation3D.fromEuler(0, 0, self.rotation.angle),
            scale=Scale3D(self.scale.x, self.scale.y, 1),
        )

    def getRotation(self):
        return self.rotation

    def setRotation(self, rotation):
        self.rotation = rotation

    def getPosition(self):
        return self.position

    def setPosition(self, position):
        self.position = position

    def getScale(self):
        return self.scale

    def setScale(self, scale):
        self.scale = scale

    def translate(self, translation: Translation2D):
        self.position += translation
