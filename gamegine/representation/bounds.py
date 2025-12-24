"""2D boundary primitives used to describe the field and robot geometry."""

from typing import Callable, List, Tuple
from abc import ABC, abstractmethod
import copy


import math

# Rendering is handled by gamegine.render.handlers, not embedded in domain objects
from gamegine.representation.base import NamedObject
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # type-only imports to keep runtime free of pybullet deps
    from gamegine.simulation.environment.shape import BulletShape  # noqa: F401
from gamegine.utils import logging
from gamegine.utils.NCIM.Dimensions.spatial import Feet
from gamegine.utils.logging import Debug
from gamegine.utils.matematika import ReflectValue1D, RotateAboutOrigin
import shapely.geometry as sg

from gamegine.utils.NCIM.ncim import (
    AngularMeasurement,
    Inch,
    RatioOf,
    SpatialMeasurement,
    Radian,
    Meter,
)


class Boundary(ABC):
    """Abstract base type for 2D shapes that support geometric transforms."""

    @abstractmethod
    def translate(self, x: SpatialMeasurement, y: SpatialMeasurement) -> "Boundary":
        """Return a copy shifted by the supplied offsets."""
        pass

    @abstractmethod
    def scale(self, factor: SpatialMeasurement) -> "Boundary":
        """Return a copy scaled uniformly by ``factor``."""
        pass

    @abstractmethod
    def reflect_x(self, axis: SpatialMeasurement) -> "Boundary":
        """Return a copy mirrored across the provided vertical axis."""
        pass

    @abstractmethod
    def reflect_y(self, axis: SpatialMeasurement) -> "Boundary":
        """Return a copy mirrored across the provided horizontal axis."""
        pass

    @abstractmethod
    def get_3d(
        self, z_start: SpatialMeasurement = 0, z_end: SpatialMeasurement = 0
    ) -> "Boundary3D":
        pass

    @abstractmethod
    def discretized(self, curve_segments: int = 5) -> "DiscreteBoundary":
        """Return a polygonal approximation of the boundary."""
        pass


class DiscreteBoundary(Boundary):
    """Base class for representing discrete boundaries which can be marked by a series of points, extending the :class:`Boundary` class. Discrete boundaries can be used to represent the shape of an object in 2D space. Can be used in all the manner of the base class, but also provides methods for checking if the boundary intersects a line, rectangle, or point, and for getting the bounded rectangle of the boundary."""

    @abstractmethod
    def get_vertices(self) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        """Expose the ordered vertex list describing the polygon."""
        pass

    def discretized(self, curve_segments: int = 5) -> "DiscreteBoundary":
        """Discrete boundaries are already polygonal, so return ``self``."""
        return self

    def __recompute_plain_points(self):
        """Refresh the cached list of raw vertices (used for shapely queries)."""
        self.plain_points = [point for point in self.get_vertices()]

    def intersects_line(
        self,
        x1: SpatialMeasurement,
        y1: SpatialMeasurement,
        x2: SpatialMeasurement,
        y2: SpatialMeasurement,
    ) -> bool:
        """Return ``True`` when the polyline intersects the given segment."""
        self.__recompute_plain_points()
        return sg.Polygon(self.plain_points).intersects(
            sg.LineString([(float(x1), float(y1)), (float(x2), float(y2))])
        )

    def intersects_rectangle(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        max_x: SpatialMeasurement,
        max_y: SpatialMeasurement,
    ) -> bool:
        """Return ``True`` if any portion of the boundary overlaps the rectangle."""
        self.__recompute_plain_points()
        return sg.Polygon(self.plain_points).intersects(sg.box(float(x), float(y), float(max_x), float(max_y)))

    def contains_point(self, x: SpatialMeasurement, y: SpatialMeasurement) -> bool:
        """Return ``True`` if the point is strictly inside the polygon."""
        self.__recompute_plain_points()
        return sg.Polygon(self.plain_points).contains(sg.Point((float(x), float(y))))

    def __convert_coordinate_sequence(
        self, coord_sequence
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        """Converts a coordinate sequence to a list of SpatialMeasurement tuples.

        :param coord_sequence: The coordinate sequence to convert (raw floats assumed in meters).
        :type coord_sequence: list
        :return: The converted coordinate sequence with SpatialMeasurement objects.
        :rtype: list
        """

        return [(Meter(x), Meter(y)) for x, y in coord_sequence]

    def get_bounded_rectangle(self) -> "Rectangle":
        """Returns the bounded rectangle of the boundary.

        :return: The bounded rectangle of the boundary.
        :rtype: :class:`Rectangle`
        """

        self.__recompute_plain_points()
        min_x = min([point[0] for point in self.plain_points])
        min_y = min([point[1] for point in self.plain_points])
        max_x = max([point[0] for point in self.plain_points])
        max_y = max([point[1] for point in self.plain_points])
        return Rectangle(min_x, min_y, max_x - min_x, max_y - min_y)

    def buffered(
        self, distance: SpatialMeasurement
    ) -> "DiscreteBoundary":  # Efficiency is cooked here...but its easy
        """Returns a buffered version of the boundary, providing a certain amount of padding around the boundary. Used to indicate safe areas which account for object sizes.

        :param distance: The distance to buffer the boundary by.
        :type distance: :class:`SpatialMeasurement`
        :return: The buffered boundary.
        :rtype: :class:`DiscreteBoundary`"""

        self.__recompute_plain_points()
        coords = self.__convert_coordinate_sequence(
            sg.Polygon(self.plain_points).buffer(distance, quad_segs=1).exterior.coords
        )
        return Polygon(coords)

    def get_3d(
        self, z_start: SpatialMeasurement = Feet(0), z_end: SpatialMeasurement = Feet(0)
    ) -> "DiscreteBoundary3D":
        """Returns the 3D version of the boundary.

        :param z_start: The starting z value of the boundary.
        :type z_start: :class:`SpatialMeasurement`
        :param z_end: The ending z value of the boundary.
        :type z_end: :class:`SpatialMeasurement`
        :return: The 3D version of the boundary.
        :rtype: :class:`DiscreteBoundary3D`
        """
        return PolygonalPrism(
            self.get_vertices(),
            z_end - z_start,
            Transform3D((Feet(0), Feet(0), z_start)),
        )


import math
import numpy as np


class Transform3D:
    """Class representing a 3D transformation including position, rotation, and scale."""

    def __init__(
        self,
        position: Tuple[SpatialMeasurement, SpatialMeasurement, SpatialMeasurement] = (
            Feet(0),
            Feet(0),
            Feet(0),
        ),
        rotation: Tuple[AngularMeasurement, AngularMeasurement, AngularMeasurement] = (
            Radian(0),
            Radian(0),
            Radian(0),
        ),  # Euler angles: (yaw, pitch, roll)
        scale: Tuple[float, float, float] = (1, 1, 1),
    ):
        self.position = position
        self.rotation = rotation
        self.scale = scale

    def apply(
        self,
        points: List[Tuple[SpatialMeasurement, SpatialMeasurement, SpatialMeasurement]],
        center: Tuple[SpatialMeasurement, SpatialMeasurement, SpatialMeasurement] = (
            Feet(0),
            Feet(0),
            Feet(0),
        ),
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement, SpatialMeasurement]]:
        """Applies the transformation to a list of 3D points."""

        S = np.diag([self.scale[0], self.scale[1], self.scale[2]])
        # Create rotation matrices
        yaw, pitch, roll = self.rotation
        yaw = yaw.to(Radian)
        pitch = pitch.to(Radian)
        roll = roll.to(Radian)

        Rz = np.array(
            [
                [math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0],
                [0, 0, 1],
            ]
        )

        Ry = np.array(
            [
                [math.cos(pitch), 0, math.sin(pitch)],
                [0, 1, 0],
                [-math.sin(pitch), 0, math.cos(pitch)],
            ]
        )
        # Rotation around x-axis (roll)
        Rx = np.array(
            [
                [1, 0, 0],
                [0, math.cos(roll), -math.sin(roll)],
                [0, math.sin(roll), math.cos(roll)],
            ]
        )

        R = Rz @ Ry @ Rx

        # Combine transformations
        transformed_points = []
        for point in points:
            point_num = [component.to(Meter) for component in point]

            # Apply scaling
            p = np.array(point_num, dtype=float)
            # Apply Centering
            p = p - np.array([component.to(Meter) for component in center])

            p = S @ p
            # Apply rotation
            p = R @ p

            # Revert centering
            p = p + np.array([component.to(Meter) for component in center])

            # Apply translation
            px = self.position[0].to(Meter) + p[0]
            py = self.position[1].to(Meter) + p[1]
            pz = self.position[2].to(Meter) + p[2]
            transformed_points.append((Meter(px), Meter(py), Meter(pz)))
        return transformed_points

    def reflect_pos_x(self) -> None:
        """Reflects the position over the x-axis."""
        self.position = (
            -self.position[0],
            self.position[1],
            self.position[2],
        )

        self.scale = (
            -self.scale[0],
            self.scale[1],
            self.scale[2],
        )

    def reflect_pos_y(self) -> None:
        """Reflects the position over the y-axis."""
        self.position = (
            self.position[0],
            -self.position[1],
            self.position[2],
        )

        self.scale = (
            self.scale[0],
            -self.scale[1],
            self.scale[2],
        )

    def reflect_pos_z(self) -> None:
        """Reflects the position over the z-axis."""
        self.position = (
            self.position[0],
            self.position[1],
            -self.position[2],
        )

        self.scale = (
            self.scale[0],
            self.scale[1],
            -self.scale[2],
        )


class Boundary3D(Boundary):
    """Base class for representing boundaries in 3D space."""

    def __init__(self, transform: Transform3D = None):
        self.transform = transform if transform else Transform3D()

    @abstractmethod
    def get_slice(self, z: SpatialMeasurement) -> Boundary:
        """Returns a slice of the boundary at a certain z value."""
        pass

    @abstractmethod
    def discretized(self, curve_segments: int = 5) -> "DiscreteBoundary3D":
        pass

    def get_bullet_shape(self):
        """Return a Bullet shape for physics.

        This requires the Bullet/pybullet-dependent modules. To keep this
        module importable without pybullet, the import is done lazily here.
        """
        try:
            from gamegine.simulation.environment.shape import BulletBox
        except Exception as e:
            raise RuntimeError(
                "PyBullet (and related Bullet shapes) not available; "
                "get_bullet_shape cannot be used without pybullet."
            ) from e

        logging.Warn("No bullet shape defined for this boundary.")
        return BulletBox(Feet(1), Feet(1), Feet(1))

    def get_transform(self) -> Transform3D:
        """Returns the transform of the boundary."""
        return copy.deepcopy(self.transform)

    def get_3d(
        self, z_start: SpatialMeasurement = Inch(0), z_end: SpatialMeasurement = Inch(0)
    ) -> "Boundary3D":
        """Returns the 3D version of the boundary. Simply returns self."""
        return self

    def translate(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        z: SpatialMeasurement = Inch(0),
    ) -> "Boundary3D":
        """Translates the boundary by the given x, y, z values."""
        self.transform.position = (
            self.transform.position[0] + x,
            self.transform.position[1] + y,
            self.transform.position[2] + z,
        )
        return self

    # Create properties for x and y
    @property
    def x(self):
        return self.transform.position[0]

    @x.setter
    def x(self, value):
        self.transform.position[0] = value

    @property
    def y(self):
        return self.transform.position[1]

    @y.setter
    def y(self, value):
        self.transform.position[1] = value

    def rotate(
        self,
        yaw: AngularMeasurement = Radian(0),
        pitch: AngularMeasurement = Radian(0),
        roll: AngularMeasurement = Radian(0),
    ) -> "Boundary3D":
        """Rotates the boundary by the given yaw, pitch, and roll angles."""
        self.transform.rotation = (
            self.transform.rotation[0] + yaw,
            self.transform.rotation[1] + pitch,
            self.transform.rotation[2] + roll,
        )
        return self

    def scale(self, sx: float = 1, sy: float = 1, sz: float = 1) -> "Boundary3D":
        """Scales the boundary by the given factors."""
        self.transform.scale = (
            self.transform.scale[0] * sx,
            self.transform.scale[1] * sy,
            self.transform.scale[2] * sz,
        )
        return self

    def reflect_x(self, axis: SpatialMeasurement = Inch(0)) -> "Boundary3D":
        """Reflects the boundary over the plane x = axis."""
        self.translate(-axis, Inch(0), Inch(0))
        self.transform.reflect_pos_x()
        self.translate(axis, Inch(0), Inch(0))
        return self

    def reflect_y(self, axis: SpatialMeasurement = Inch(0)) -> "Boundary3D":
        """Reflects the boundary over the plane y = axis."""
        self.translate(Inch(0), -axis, Inch(0))
        self.transform.reflect_pos_y()
        self.translate(Inch(0), axis, Inch(0))
        return self

    def reflect_z(self, axis: SpatialMeasurement = Inch(0)) -> "Boundary3D":
        """Reflects the boundary over the plane z = axis."""
        self.translate(Inch(0), Inch(0), -axis)
        self.transform.reflect_pos_z()
        self.translate(Inch(0), Inch(0), axis)
        return self

    @abstractmethod
    def project_to_2d(self) -> Boundary:
        """Projects the 3D boundary onto the XY-plane."""
        pass


class DiscreteBoundary3D(Boundary3D, DiscreteBoundary):
    """Base class for representing discrete boundaries in 3D space."""

    def get_3d_vertices(
        self,
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement, SpatialMeasurement]]:
        """Returns the 3D vertices of the boundary."""
        interval = self.get_z_interval()
        twoD_vertices = super(Boundary3D, self).get_vertices()
        vertices = [(x, y, interval[0]) for x, y in twoD_vertices]

        vertices += [(x, y, interval[1]) for x, y in twoD_vertices]

        return vertices

    def discretized(self, curve_segments: int = 5) -> "DiscreteBoundary3D":
        """Returns self for already discretized 3D boundaries."""
        return self

    @abstractmethod
    def get_z_interval(self) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        """Returns the z interval of the boundary."""
        pass

    def get_slice(self, z: SpatialMeasurement) -> DiscreteBoundary:
        """Returns a slice of the boundary at a certain z value."""
        interval = self.get_z_interval()
        if z < interval[0] or z > interval[1]:
            return None
        return self.project_to_2d()

    def project_to_2d(self) -> DiscreteBoundary:
        """Projects the 3D boundary onto the XY-plane."""
        transformed_points = self.transform.apply(self.get_3d_vertices())
        points = [(p[0].to(Meter), p[1].to(Meter)) for p in transformed_points]
        convex_hull = [
            (Meter(x), Meter(y))
            for x, y in sg.MultiPoint(points).convex_hull.exterior.coords
        ]
        return Polygon(convex_hull)


def RegularPolygon(
    center: Tuple[SpatialMeasurement, SpatialMeasurement],
    radius: SpatialMeasurement,
    num_sides: int,
):
    """Creates a regular polygon from the center, radius, and number of sides.

    :param center: The center of the polygon.
    :type center: Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]
    :param radius: The radius of the polygon.
    :type radius: :class:`SpatialMeasurement`
    :param num_sides: The number of sides of the polygon.
    :type num_sides: int
    :return: The regular polygon.
    :rtype: :class:`Polygon`
    """
    vertices = []
    for i in range(num_sides):
        angle = Radian(2 * math.pi * i / num_sides)
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        vertices.append((x, y))
    return Polygon(vertices)


class Rectangle(DiscreteBoundary):
    """Class for representing a rectangle boundary, extending the :class:`DiscreteBoundary` class. Rectangles can be used to represent the shape of an object in 2D space. Includes additional methods for getting the minimum and maximum x and y values of the rectangle.

    :param x: The x value of the bottom left corner of the rectangle.
    :type x: :class:`SpatialMeasurement`
    :param y: The y value of the bottom left corner of the rectangle.
    :type y: :class:`SpatialMeasurement`
    :param width: The width of the rectangle.
    :type width: :class:`SpatialMeasurement`
    :param height: The height of the rectangle.
    :type height: :class:`SpatialMeasurement`"""

    def __init__(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        width: SpatialMeasurement,
        height: SpatialMeasurement,
    ):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def __str__(self):
        return f"Rectangle(x={self.x}, y={self.y}, width={self.width}, height={self.height})"

    def translate(self, x: SpatialMeasurement, y: SpatialMeasurement) -> "Rectangle":
        self.x += x
        self.y += y
        return self

    def scale(self, factor: SpatialMeasurement) -> "Rectangle":
        self.width *= factor
        self.height *= factor
        return self

    def reflect_x(self, axis: SpatialMeasurement) -> "Rectangle":
        self.x = ReflectValue1D(self.x, axis)
        self.x -= self.width
        return self

    def reflect_y(self, axis: SpatialMeasurement) -> "Rectangle":
        self.y = ReflectValue1D(self.y, axis)
        self.y -= self.height
        return self

    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary:
        return self

    def get_vertices(self) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        return [
            (self.x, self.y),
            (self.x + self.width, self.y),
            (self.x + self.width, self.y + self.height),
            (self.x, self.y + self.height),
        ]

    def get_min_x(self) -> SpatialMeasurement:
        """Returns the minimum x value of the rectangle, being the x value of the left side of the rectangle.

        :return: The minimum x value of the rectangle.
        :rtype: :class:`SpatialMeasurement`
        """
        return self.x

    def get_max_x(self) -> SpatialMeasurement:
        """Returns the maximum x value of the rectangle, being the x value of the right side of the rectangle.

        :return: The maximum x value of the rectangle.
        :rtype: :class:`SpatialMeasurement`
        """
        return self.x + self.width

    def get_min_y(self) -> SpatialMeasurement:
        """Returns the minimum y value of the rectangle, being the y value of the top side of the rectangle.

        :return: The minimum y value of the rectangle.
        :rtype: :class:`SpatialMeasurement`
        """
        return self.y

    def get_max_y(self) -> SpatialMeasurement:
        """Returns the maximum y value of the rectangle, being the y value of the bottom side of the rectangle.

        :return: The maximum y value of the rectangle.
        :rtype: :class:`SpatialMeasurement`
        """
        return self.y + self.height

    def get_center(self) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        """Returns the center of the rectangle.

        :return: The center of the rectangle.
        :rtype: Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]
        """
        return (self.x + self.width / 2, self.y + self.height / 2)

    def get_3d(self, z_start=Feet(0), z_end=Feet(0)):
        centered_vertices = self.get_vertices()
        center = self.get_center()
        centered_vertices = [
            (x - center[0], y - center[1]) for x, y in centered_vertices
        ]
        return PolygonalPrism(
            centered_vertices,
            z_end - z_start,
            Transform3D((center[0], center[1], Feet(0))),
        )

    @classmethod
    def from_center(
        cls,
        center: Tuple[SpatialMeasurement, SpatialMeasurement],
        width: SpatialMeasurement,
        height: SpatialMeasurement,
    ) -> "Rectangle":
        """Creates a rectangle from the center point, width, and height.

        :param center: The center point of the rectangle.
        :type center: Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]
        :param width: The width of the rectangle.
        :type width: :class:`SpatialMeasurement`
        :param height: The height of the rectangle.
        :type height: :class:`SpatialMeasurement`
        :return: The rectangle.
        :rtype: :class:`Rectangle`
        """
        return cls(
            center[0] - width / 2,
            center[1] - height / 2,
            width,
            height,
        )


class Square(Rectangle):
    def __init__(
        self, x: SpatialMeasurement, y: SpatialMeasurement, side: SpatialMeasurement
    ):
        super().__init__(x, y, side, side)

    def __str__(self):
        return f"Square(x={self.x}, y={self.y}, side={self.width})"


class Circle(Boundary):
    """Class for representing a circle boundary, extending the :class:`Boundary` class. Circles can be used to represent the shape of an object in 2D space.

    :param x: The x value of the center of the circle.
    :type x: :class:`SpatialMeasurement`
    :param y: The y value of the center of the circle.
    :type y: :class:`SpatialMeasurement`
    :param radius: The radius of the circle.
    :type radius: :class:`SpatialMeasurement`"""

    def __init__(
        self, x: SpatialMeasurement, y: SpatialMeasurement, radius: SpatialMeasurement
    ):
        self.x = x
        self.y = y
        self.radius = radius

    def __str__(self):
        return f"Circle(x={self.x}, y={self.y}, radius={self.radius})"

    def translate(self, x: SpatialMeasurement, y: SpatialMeasurement) -> "Circle":
        self.x += x
        self.y += y
        return self

    def scale(self, factor: SpatialMeasurement) -> "Circle":
        self.radius *= factor
        return self

    def reflect_x(self, axis: SpatialMeasurement) -> "Circle":
        self.x = ReflectValue1D(self.x, axis)
        return self

    def reflect_y(self, axis: SpatialMeasurement) -> "Circle":
        self.y = 2 * axis - self.y
        return self

    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary:
        points = []
        step = 2 * math.pi / curve_segments
        for i in range(curve_segments):
            angle = step * i
            points.append(
                (
                    math.cos(angle) * self.radius + self.x,
                    math.sin(angle) * self.radius + self.y,
                )
            )
        return Polygon(points)

    def get_3d(
        self, z_start: SpatialMeasurement = 0, z_end: SpatialMeasurement = 0
    ) -> "Cylinder":
        return Cylinder(
            self.radius, z_end - z_start, Transform3D((self.x, self.y, z_start))
        )


class Cylinder(DiscreteBoundary3D):
    """Class for representing a cylinder."""

    def __init__(
        self,
        radius: SpatialMeasurement,
        height: SpatialMeasurement,
        transform: Transform3D = None,
        curve_segments: int = 16,
    ):
        super().__init__(transform)
        self.radius = radius
        self.height = height
        self.curve_segments = curve_segments

    def get_z_interval(self) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        z = self.transform.position[2]
        return (z, z + self.height * self.transform.scale[2])

    def discretized(self, curve_segments: int = None) -> "Cylinder":
        if curve_segments is not None:
            self.curve_segments = curve_segments
        return self

    def get_vertices(self) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        return self.project_to_2d().get_vertices()

    def project_to_2d(self) -> "Polygon":
        points_3d = self.get_3d_vertices()
        points_2d = [(p[0].to(Meter), p[1].to(Meter)) for p in points_3d]
        convex_hull = [
            (Meter(x), Meter(y))
            for x, y in sg.MultiPoint(points_2d).convex_hull.exterior.coords
        ]
        return Polygon(convex_hull)

    def get_3d_vertices(self):
        points_3d = []
        angle_step = 2 * math.pi / self.curve_segments
        interval = self.get_z_interval()
        for i in range(self.curve_segments):
            angle = angle_step * i
            x = self.radius * math.cos(angle)
            y = self.radius * math.sin(angle)
            points_3d.append((x, y, interval[0]))
            points_3d.append((x, y, interval[1]))
        transformed_points = self.transform.apply(
            points_3d, center=(Feet(0), Feet(0), (interval[0] + interval[1]) / 2)
        )
        return transformed_points

    def get_bullet_shape(self):
        try:
            from gamegine.simulation.environment.shape import BulletCylinder
        except Exception as e:
            raise RuntimeError(
                "PyBullet (and related Bullet shapes) not available; "
                "get_bullet_shape cannot be used without pybullet."
            ) from e
        return BulletCylinder(self.radius, self.height)


class Polygon(DiscreteBoundary):
    """Class for representing a polygon boundary, extending the :class:`DiscreteBoundary` class. Polygons can be used to represent the shape of an object in 2D space.

    :param points: The points of the polygon.
    :type points: List[Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]]
    """

    def __init__(self, points: List[Tuple[SpatialMeasurement, SpatialMeasurement]]):
        self.points = points

    def __str__(self):
        return f"Polygon(points={self.points})"

    def translate(self, x: SpatialMeasurement, y: SpatialMeasurement) -> "Polygon":
        self.points = [(x + point[0], y + point[1]) for point in self.points]
        return self

    def scale(self, factor: float) -> "Polygon":
        self.points = [(factor * point[0], factor * point[1]) for point in self.points]
        return self

    def reflect_x(self, axis: SpatialMeasurement) -> "Polygon":
        self.points = [
            (ReflectValue1D(point[0], axis), point[1]) for point in self.points
        ]
        return self

    def reflect_y(self, axis: SpatialMeasurement) -> "Polygon":
        self.points = [
            (point[0], ReflectValue1D(point[1], axis)) for point in self.points
        ]
        return self

    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary:
        return self

    def get_vertices(self) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        return self.points


class PolygonalPrism(DiscreteBoundary3D):
    """Class for representing a polygonal prism."""

    def __init__(
        self,
        points: List[Tuple[SpatialMeasurement, SpatialMeasurement]],
        height: SpatialMeasurement,
        transform: Transform3D = None,
    ):
        super().__init__(transform)
        self.local_points = [(p[0], p[1], Inch(0)) for p in points]
        self.height = height

    def get_z_interval(self) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        z = self.transform.position[2]
        return (z, z + self.height * self.transform.scale[2])

    def discretized(self, curve_segments: int = 5) -> "PolygonalPrism":
        return self

    def get_vertices(self) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        return self.project_to_2d().get_vertices()

    def get_2d_local_points(
        self,
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        return [(p[0], p[1]) for p in self.local_points]

    def project_to_2d(self) -> "Polygon":
        transformed_points = self.transform.apply(self.local_points)
        points_2d = [(p[0], p[1]) for p in transformed_points]
        return Polygon(points_2d)

    def get_max_x(self) -> SpatialMeasurement:
        return max([point[0] for point in self.local_points])

    def get_min_x(self) -> SpatialMeasurement:
        return min([point[0] for point in self.local_points])

    def get_max_y(self) -> SpatialMeasurement:
        return max([point[1] for point in self.local_points])

    def get_min_y(self) -> SpatialMeasurement:
        return min([point[1] for point in self.local_points])

    def get_bullet_shape(self):
        try:
            # Import inside to avoid pybullet dependency at module import time
            from gamegine.simulation.environment import shape as bullet_shape_mod
        except Exception as e:
            raise RuntimeError(
                "PyBullet (and related Bullet shapes) not available; "
                "get_bullet_shape cannot be used without pybullet."
            ) from e
        return bullet_shape_mod.PolygonalPrism(self.get_2d_local_points(), self.height)


class Line(DiscreteBoundary):
    """Class for representing a line boundary, extending the :class:`DiscreteBoundary` class. Lines can be used to represent the shape of an object in 2D space.

    :param x1: The x value of the first point of the line.
    :type x1: :class:`SpatialMeasurement`
    :param y1: The y value of the first point of the line.
    :type y1: :class:`SpatialMeasurement`
    :param x2: The x value of the second point of the line.
    :type x2: :class:`SpatialMeasurement`
    :param y2: The y value of the second point of the line.
    :type y2: :class:`SpatialMeasurement`"""

    def __init__(
        self,
        x1: SpatialMeasurement,
        y1: SpatialMeasurement,
        x2: SpatialMeasurement,
        y2: SpatialMeasurement,
    ):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def __str__(self):
        return f"Line(x1={self.x1}, y1={self.y1}, x2={self.x2}, y2={self.y2}"

    def translate(self, x: SpatialMeasurement, y: SpatialMeasurement) -> "Line":
        self.x1 += x
        self.y1 += y
        self.x2 += x
        self.y2 += y
        return self

    def scale(self, factor: SpatialMeasurement) -> "Line":
        self.x1 *= factor
        self.y1 *= factor
        self.x2 *= factor
        self.y2 *= factor
        return self

    def reflect_y(self, axis: SpatialMeasurement) -> "Line":
        self.y1 = ReflectValue1D(self.y1, axis)
        self.y2 = ReflectValue1D(self.y2, axis)
        return self

    def reflect_x(self, axis: SpatialMeasurement) -> "Line":
        self.x1 = ReflectValue1D(self.x1, axis)
        self.x2 = ReflectValue1D(self.x2, axis)
        return self

    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary:
        return self

    def get_vertices(self) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        return [(self.x1, self.y1), (self.x2, self.y2)]


class Point(DiscreteBoundary3D):
    """Class for representing a point boundary, extending the :class:`DiscreteBoundary3D` class. Points can be used to represent a point in 3D space.

    :param x: The x value of the point.
    :type x: :class:`SpatialMeasurement`
    :param y: The y value of the point.
    :type y: :class:`SpatialMeasurement`
    :param z: The z value of the point.
    :type z: :class:`SpatialMeasurement`"""

    def __init__(
        self, x: SpatialMeasurement, y: SpatialMeasurement, z: SpatialMeasurement = 0
    ):
        super().__init__(Transform3D((x, y, z)))

    def get_z_interval(self) -> Tuple[SpatialMeasurement]:
        return (self.z, self.z)

    def __str__(self):
        return f"Point(x={self.x}, y={self.y}, z={self.z})"

    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary:
        return self

    def get_vertices(self) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        return [(self.x, self.y)]

    def project_to_2d(self) -> "Point":
        return Point(self.x, self.y)

    def draw(self, render_scale):
        pygame.draw.circle(
            pygame.display.get_surface(),
            (255, 255, 0),
            (RatioOf(self.x, render_scale), RatioOf(self.y, render_scale)),
            5,
        )


class BoundedObject(NamedObject):
    """Base class for representing game objects which occupy a space on the field. Contain a boundary which indicates the space the object occupies."""

    def __init__(self, bounds: Boundary, name: str = "") -> None:
        super().__init__(name)
        self.bounds = bounds

    def mirrored_over_horizontal(self, axis: SpatialMeasurement) -> "BoundedObject":
        """Returns a new object which is a mirror of the object over the horizontal axis at the given axis.

        :param axis: The axis to reflect the object over.
        :type axis: :class:`SpatialMeasurement`
        :return: The mirrored object.
        :rtype: :class:`BoundedObject`
        """

        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.reflect_y(axis)
        return obj

    def mirrored_over_horizontal_ip(self, axis: SpatialMeasurement) -> None:
        """Mirrors the object over the horizontal axis at the given axis in place, not returning a new object.

        :param axis: The axis to reflect the object over.
        :type axis: :class:`SpatialMeasurement`
        """

        self.bounds = self.bounds.reflect_y(axis)

    def mirrored_over_vertical(self, axis: SpatialMeasurement) -> "BoundedObject":
        """Returns a new object which is a mirror of the object over the vertical axis at the given axis.

        :param axis: The axis to reflect the object over.
        :type axis: :class:`SpatialMeasurement`
        :return: The mirrored object.
        :rtype: :class:`BoundedObject`
        """

        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.reflect_x(axis)
        return obj

    def mirrored_over_vertical_ip(self, axis: SpatialMeasurement) -> "BoundedObject":
        """Mirrors the object over the vertical axis at the given axis in place, not returning a new object.

        :param axis: The axis to reflect the object over.
        :type axis: :class:`SpatialMeasurement`
        :return: This object.
        :rtype: :class:`BoundedObject`
        """

        self.bounds = self.bounds.reflect_x(axis)
        return self

    def scaled(self, factor: SpatialMeasurement) -> "BoundedObject":
        """Returns a new object which is a scaled version of the object by the given factor.

        :param factor: The factor to scale the object by.
        :type factor: :class:`SpatialMeasurement`
        :return: The scaled object.
        :rtype: :class:`BoundedObject`
        """

        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.scale(factor)
        return obj

    def translated(
        self, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> "BoundedObject":
        """Returns a new object which is a translated version of the object by the given x and y values.

        :param x: The x value to translate the object by.
        :type x: :class:`SpatialMeasurement`
        :param y: The y value to translate the object by.
        :type y: :class:`SpatialMeasurement`
        :return: The translated object.
        :rtype: :class:`BoundedObject`
        """

        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.translate(x, y)
        return obj

    def translate_ip(
        self, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> "BoundedObject":
        """Translates the object by the given x and y values in place, not returning a new object.

        :param x: The x value to translate the object by.
        :type x: :class:`SpatialMeasurement`
        :param y: The y value to translate the object by.
        :type y: :class:`SpatialMeasurement`
        :return: This object.
        :rtype: :class:`BoundedObject`
        """

        self.bounds = self.bounds.translate(x, y)
        return self

    def discretized(self, curve_segments: int = 5) -> "BoundedObject":
        """Returns a new object which contains a discretized version of its boundary.

        :param curve_segments: The number of curve segments to use when discretizing the object.
        :type curve_segments: int
        :return: The discretized object.
        :rtype: :class:`BoundedObject`
        """

        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.discretized(curve_segments)
        return obj

    def discretize_ip(self, curve_segments: int = 5) -> "BoundedObject":
        """Discretizes the object in place, not returning a new object.

        :param curve_segments: The number of curve segments to use when discretizing the object.
        :type curve_segments: int
        :return: This object.
        :rtype: :class:`BoundedObject`
        """

        self.bounds = self.bounds.discretized(curve_segments)
        return self


def LineIntersectsAnyBound(
    bounds: List[DiscreteBoundary],
    x1: SpatialMeasurement,
    y1: SpatialMeasurement,
    x2: SpatialMeasurement,
    y2: SpatialMeasurement,
) -> bool:
    """Checks if a line intersects any of the given boundaries.

    :param bounds: The boundaries to check for intersection.
    :type bounds: List[:class:`DiscreteBoundary`]
    :param x1: The x value of the first point of the line.
    :type x1: :class:`SpatialMeasurement`
    :param y1: The y value of the first point of the line.
    :type y1: :class:`SpatialMeasurement`
    :param x2: The x value of the second point of the line.
    :type x2: :class:`SpatialMeasurement`
    :param y2: The y value of the second point of the line.
    :type y2: :class:`SpatialMeasurement`
    :return: True if the line intersects any of the boundaries, False otherwise.
    :rtype: bool
    """

    for bound in bounds:
        if not isinstance(bound, DiscreteBoundary):
            print(f"Discretizing continous bound {bound}")
        bound.discretized()  # If not already discretized
        if bound.intersects_line(x1, y1, x2, y2):
            return True
    return False


def CircularPattern(
    objects: List[BoundedObject],
    center: Tuple[SpatialMeasurement, SpatialMeasurement],
    angle: AngularMeasurement,
    num_objects: int,
    prefix_func: Callable[[int], str],
) -> List[BoundedObject]:
    """Creates a circular pattern of objects around a center point, allowing for a :class:`BoundedObject` to be rotated and copied around a center point.

    :param objects: The objects to create a circular pattern of.
    :type objects: List[:class:`BoundedObject`]
    :param center: The center point of the circular pattern.
    :type center: Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]
    :param angle: The angle to rotate each object by.
    :type angle: :class:`AngularMeasurement`
    :param num_objects: The number of objects to create in the circular pattern, inclusive of the original objects.
    :type num_objects: int
    :param prefix_func: The function to prefix the name of each object with.
    :type prefix_func: Callable[[int], str]
    :return: The objects in the circular pattern.
    :rtype: List[:class:`BoundedObject`]
    """

    if num_objects <= 1:
        raise Exception("Number of objects must be greater than 1")
    out = []
    Debug(f"Circular pattern with angle: {angle}")
    angle_increment = angle / num_objects
    Debug(f"Increment: {angle_increment}")
    Debug(f"Doubled: {angle_increment*2}")
    for object in objects:
        centerToObject = [object.bounds.x - center[0], object.bounds.y - center[1]]
        objectToCenter = [centerToObject[0] * -1, centerToObject[1] * -1]

        for i in range(1, num_objects):
            angle = angle_increment * i
            Debug(angle)
            new_vector = RotateAboutOrigin(centerToObject, angle * -1)
            out.append(
                object.translated(objectToCenter[0], objectToCenter[1])
                .translate_ip(new_vector[0], new_vector[1])
                .prefix(prefix_func(i))
            )
    return objects + out


def SymmetricalX(
    objects: List[BoundedObject],
    axis: SpatialMeasurement,
    prefix: str,
    prefix_og: str = "",
) -> List[BoundedObject]:
    """Creates a symmetrical pattern of objects across the x-axis at the given axis, copying the objects and mirroring them across the axis.

    :param objects: The objects to create a symmetrical pattern of.
    :type objects: List[:class:`BoundedObject`]
    :param axis: The axis to create the symmetrical pattern across, x = axis will be the axis of symmetry.
    :type axis: :class:`SpatialMeasurement`
    :param prefix: The prefix to add to the name of each object.
    :type prefix: str
    :param prefix_og: The prefix to add to the name of the original objects.
    :type prefix_og: str
    :return: The objects in the symmetrical pattern.
    :rtype: List[:class:`BoundedObject`]
    """
    return [obj.prefix(prefix_og) for obj in objects] + [
        obj.mirrored_over_vertical(axis).prefix(prefix) for obj in objects
    ]


def SymmetricalY(
    objects: List[BoundedObject],
    axis: SpatialMeasurement,
    prefix: str,
    prefix_og: str = "",
) -> List[BoundedObject]:
    """Creates a symmetrical pattern of objects across the y-axis at the given axis, copying the objects and mirroring them across the axis.

    :param objects: The objects to create a symmetrical pattern of.
    :type objects: List[:class:`BoundedObject`]
    :param axis: The axis to create the symmetrical pattern across, y = axis will be the axis of symmetry.
    :type axis: :class:`SpatialMeasurement`
    :param prefix: The prefix to add to the name of each object.
    :type prefix: str
    :param prefix_og: The prefix to add to the name of the original objects.
    :type prefix_og: str
    :return: The objects in the symmetrical pattern.
    :rtype: List[:class:`BoundedObject`]
    """
    return [obj.prefix(prefix_og) for obj in objects] + [
        obj.mirrored_over_horizontal(axis).prefix(prefix) for obj in objects
    ]


def SymmetricalXY(
    objects: List[BoundedObject],
    axis_x: SpatialMeasurement,
    axis_y: SpatialMeasurement,
    prefix: str,
    prefix_og: str = "",
) -> List[BoundedObject]:
    """Creates a symmetrical pattern of objects across both the x and y axes at the given axes, copying the objects and mirroring them across the axes.

    :param objects: The objects to create a symmetrical pattern of.
    :type objects: List[:class:`BoundedObject`]
    :param axis_x: The x-axis to create the symmetrical pattern across, x = axis will be the axis of symmetry.
    :type axis_x: :class:`SpatialMeasurement`
    :param axis_y: The y-axis to create the symmetrical pattern across, y = axis will be the axis of symmetry.
    :type axis_y: :class:`SpatialMeasurement`
    :param prefix: The prefix to add to the name of each object.
    :type prefix: str
    :param prefix_og: The prefix to add to the name of the original objects.
    :type prefix_og: str
    :return: The objects in the symmetrical pattern.
    :rtype: List[:class:`BoundedObject`]
    """

    return [obj.prefix(prefix_og) for obj in objects] + [
        obj.mirrored_over_vertical(axis_y)
        .mirrored_over_horizontal_ip(axis_x)
        .prefix(prefix)
        for obj in objects
    ]


def ExpandedObjectBounds(
    objects: List[BoundedObject],
    robot_radius: SpatialMeasurement = Inch(21),
    discretization_quality=4,
) -> List[DiscreteBoundary]:
    """Returns the expanded bounds of a list of objects, providing a certain amount of padding around the objects to account for object sizes.

    :param objects: The objects to get the expanded bounds of.
    :type objects: List[:class:`BoundedObject`]
    :param robot_radius: The radius of the robot to expand the bounds by.
    :type robot_radius: :class:`SpatialMeasurement`
    :param discretization_quality: The quality of discretization to use when expanding the bounds.
    :type discretization_quality: int
    :return: The expanded bounds of the objects.
    :rtype: List[:class:`DiscreteBoundary`]
    """
    return [
        object.bounds.discretized(discretization_quality).buffered(robot_radius)
        for object in objects
    ]


def ExpandedBounds(
    bounds: List[Boundary],
    robot_radius: SpatialMeasurement = Inch(21),
    discretization_quality=4,
) -> List[DiscreteBoundary]:
    """Returns the expanded bounds of a list of boundaries, providing a certain amount of padding around the boundaries to account for object sizes.

    :param bounds: The boundaries to get the expanded bounds of.
    :type bounds: List[:class:`Boundary`]
    :param robot_radius: The radius of the robot to expand the bounds by.
    :type robot_radius: :class:`SpatialMeasurement`
    :param discretization_quality: The quality of discretization to use when expanding the bounds.
    :type discretization_quality: int
    :return: The expanded bounds of the boundaries.
    :rtype: List[:class:`DiscreteBoundary`]
    """

    return [
        bound.discretized(discretization_quality).buffered(robot_radius)
        for bound in bounds
    ]
