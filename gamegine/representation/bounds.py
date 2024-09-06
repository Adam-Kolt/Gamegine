from typing import Callable, List, Tuple
from abc import ABC, abstractmethod
import copy

import pint
import math

import pygame
from gamegine.render.drawable import Drawable
from gamegine.render.style import Palette
from gamegine.representation.base import NamedObject
from gamegine.utils.NCIM.Dimensions.spatial import Feet
from gamegine.utils.logging import Debug
from gamegine.utils.matematika import ReflectValue1D, RotateAboutOrigin
import shapely.geometry as sg

from gamegine.utils.NCIM.ncim import (
    AngularMeasurement,
    Inch,
    RatioOf,
    SpatialMeasurement,
)


class Boundary(ABC):
    """Abstract base class for representing a boundary of an object. Boundaries are used to represent the shape of an object in 2D space. Boundaries can be translated, scaled, and reflected across the x and y axes. Boundaries can also be discretized into a series of points."""

    @abstractmethod
    def translate(self, x: SpatialMeasurement, y: SpatialMeasurement) -> "Boundary":
        """Translates the boundary by the given x and y values.

        :param x: The x value to translate the boundary by.
        :type x: :class:`SpatialMeasurement`
        :param y: The y value to translate the boundary by.
        :type y: :class:`SpatialMeasurement`
        :return: The translated boundary.
        :rtype: :class:`Boundary`
        """
        pass

    @abstractmethod
    def scale(self, factor: SpatialMeasurement) -> "Boundary":
        """Scales the boundary by the given factor.

        :param factor: The factor to scale the boundary by.
        :type factor: :class:`SpatialMeasurement`
        :return: The scaled boundary.
        :rtype: :class:`Boundary`
        """
        pass

    @abstractmethod
    def reflect_x(self, axis: SpatialMeasurement) -> "Boundary":
        """Reflects the boundary across the x-axis at the given axis.

        :param axis: The axis to reflect the boundary across.
        :type axis: :class:`SpatialMeasurement`
        :return: The reflected boundary.
        :rtype: :class:`Boundary`
        """
        pass

    @abstractmethod
    def reflect_y(self, axis: SpatialMeasurement) -> "Boundary":
        """Reflects the boundary across the y-axis at the given axis.

        :param axis: The axis to reflect the boundary across.
        :type axis: :class:`SpatialMeasurement`
        :return: The reflected boundary.
        :rtype: :class:`Boundary`
        """
        pass

    @abstractmethod
    def get_3d(
        self, z_start: SpatialMeasurement = 0, z_end: SpatialMeasurement = 0
    ) -> "Boundary3D":
        pass

    @abstractmethod
    def discretized(self, curve_segments: int = 5) -> "DiscreteBoundary":
        """Discretizes the boundary into a series of points.

        :param curve_segments: The number of curve segments to use when discretizing the boundary.
        :type curve_segments: int
        :return: The discretized boundary.
        :rtype: :class:`DiscreteBoundary`
        """
        pass


class DiscreteBoundary(Boundary, Drawable):
    """Base class for representing discrete boundaries which can be marked by a series of points, extending the :class:`Boundary` class. Discrete boundaries can be used to represent the shape of an object in 2D space. Can be used in all the manner of the base class, but also provides methods for checking if the boundary intersects a line, rectangle, or point, and for getting the bounded rectangle of the boundary."""

    @abstractmethod
    def get_vertices(self) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        """Returns the vertices of the boundary.

        :return: The vertices of the boundary.
        :rtype: List[Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]]
        """

        pass

    def discretized(self, curve_segments: int = 5) -> "DiscreteBoundary":
        """Discretizes the boundary into a series of points.

        :param curve_segments: The number of curve segments to use when discretizing the boundary.
        :type curve_segments: int
        :return: The discretized boundary.
        :rtype: :class:`DiscreteBoundary`
        """
        return self

    def __recompute_plain_points(self):
        """Recomputes the plain points of the boundary, used for only updating points when necessary to keep them consistent with the shape."""
        self.plain_points = [point for point in self.get_vertices()]

    def intersects_line(
        self,
        x1: SpatialMeasurement,
        y1: SpatialMeasurement,
        x2: SpatialMeasurement,
        y2: SpatialMeasurement,
    ) -> bool:
        """Checks if the boundary intersects a line.

        :param x1: The x value of the first point of the line.
        :type x1: :class:`SpatialMeasurement`
        :param y1: The y value of the first point of the line.
        :type y1: :class:`SpatialMeasurement`
        :param x2: The x value of the second point of the line.
        :type x2: :class:`SpatialMeasurement`
        :param y2: The y value of the second point of the line.
        :type y2: :class:`SpatialMeasurement`
        :return: True if the boundary intersects the line, False otherwise.
        :rtype: bool"""
        self.__recompute_plain_points()
        return sg.Polygon(self.plain_points).intersects(
            sg.LineString([(x1, y1), (x2, y2)])
        )

    def intersects_rectangle(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        max_x: SpatialMeasurement,
        max_y: SpatialMeasurement,
    ) -> bool:
        """Checks if the boundary intersects a rectangle.

        :param x: The x value of the bottom left corner of the rectangle.
        :type x: :class:`SpatialMeasurement`
        :param y: The y value of the bottom left corner of the rectangle.
        :type y: :class:`SpatialMeasurement`
        :param max_x: The x value of the top right corner of the rectangle.
        :type max_x: :class:`SpatialMeasurement`
        :param max_y: The y value of the top right corner of the rectangle.
        :type max_y: :class:`SpatialMeasurement`
        :return: True if the boundary intersects the rectangle, False otherwise.
        :rtype: bool
        """
        self.__recompute_plain_points()
        return sg.Polygon(self.plain_points).intersects(sg.box(x, y, max_x, max_y))

    def contains_point(self, x: SpatialMeasurement, y: SpatialMeasurement) -> bool:
        """Checks if the boundary contains a point.

        :param x: The x value of the point.
        :type x: :class:`SpatialMeasurement`
        :param y: The y value of the point.
        :type y: :class:`SpatialMeasurement`
        :return: True if the boundary contains the point, False otherwise.
        :rtype: bool
        """

        self.__recompute_plain_points()
        return sg.Polygon(self.plain_points).contains(sg.Point((x, y)))

    def __convert_coordinate_sequence(
        self, coord_sequence
    ) -> List[Tuple[float, float]]:
        """Converts a coordinate sequence to a list of tuples.

        :param coord_sequence: The coordinate sequence to convert.
        :type coord_sequence: list
        :return: The converted coordinate sequence.
        :rtype: list
        """

        return [(x, y) for x, y in coord_sequence]

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

    def draw(self, render_scale: SpatialMeasurement) -> None:
        """Draws the boundary, used by the :class:`Renderer` to draw the boundary.

        :param render_scale: The scale to render the boundary at.
        :type render_scale: :class:`SpatialMeasurement`
        """

        pygame.draw.polygon(
            pygame.display.get_surface(),
            (255, 255, 0),
            [
                (RatioOf(point[0], render_scale), RatioOf(point[1], render_scale))
                for point in self.get_vertices()
            ],
        )

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
        self, z_start: SpatialMeasurement = 0, z_end: SpatialMeasurement = 0
    ) -> "DiscreteBoundary3D":
        """Returns the 3D version of the boundary.

        :param z_start: The starting z value of the boundary.
        :type z_start: :class:`SpatialMeasurement`
        :param z_end: The ending z value of the boundary.
        :type z_end: :class:`SpatialMeasurement`
        :return: The 3D version of the boundary.
        :rtype: :class:`DiscreteBoundary3D`
        """
        return PolygonalPrism(self.get_vertices(), z_start, z_end)


class DiscreteBoundary3D(DiscreteBoundary):
    """Base class for representing discrete boundaries in 3D space, extending the :class:`DiscreteBoundary` class. Discrete 3D boundaries can be used to represent the shape of an object in 3D space. Can be used in all the manner of the base class, but also provides methods for getting the z interval of the boundary and for getting a slice of the 3D Shape at a certain z value."""

    def get_slice(self, z: SpatialMeasurement) -> DiscreteBoundary:
        """Returns a slice of the boundary at a certain z value.

        :param z: The z value of the slice.
        :type z: :class:`SpatialMeasurement`
        :return: The 2-dimensional slice of the boundary at the given z value.
        :rtype: :class:`DiscreteBoundary`
        """
        interval = self.get_z_interval()
        if z < interval[0] or z > interval[1]:
            return None
        return self

    def discretized(self, curve_segments: int = 5) -> "DiscreteBoundary3D":
        """Discretizes the boundary into a series of points. Simply returns self for already discretized 3D boundaries.

        :param curve_segments: The number of curve segments to use when discretizing the boundary.
        :type curve_segments: int
        :return: The discretized boundary.
        :rtype: :class:`DiscreteBoundary3D`
        """
        return self

    @abstractmethod
    def get_z_interval(self) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        """Returns the z interval of the boundary, indicating the range of z values the boundary occupies.

        :return: The z interval of the boundary.
        :rtype: Tuple[:class:`SpatialMeasurement`, :class:`SpatialMeasurement`]
        """
        pass

    def get_3d(
        self, z_start: SpatialMeasurement = 0, z_end: SpatialMeasurement = 0
    ) -> "DiscreteBoundary3D":
        """Returns the 3D version of the boundary. Simply returns self for already 3D boundaries.

        :param z_start: The starting z value of the boundary.
        :type z_start: :class:`SpatialMeasurement`
        :param z_end: The ending z value of the boundary.
        :type z_end: :class:`SpatialMeasurement`
        :return: The 3D version of the boundary.
        :rtype: :class:`DiscreteBoundary3D`
        """
        return self

    def draw(self, render_scale: SpatialMeasurement) -> None:
        """Draws the boundary, used by the :class:`Renderer` to draw the boundary.

        :param render_scale: The scale to render the boundary at.
        :type render_scale: :class:`SpatialMeasurement`
        """

        color_scale = 255 - int(255 * self.get_z_interval()[1] / Feet(10))
        pygame.draw.polygon(
            pygame.display.get_surface(),
            (color_scale, color_scale, 0),
            [
                (RatioOf(point[0], render_scale), RatioOf(point[1], render_scale))
                for point in self.get_vertices()
            ],
        )


class Boundary3D(Boundary):
    """Base class for representing boundaries in 3D space, extending the :class:`Boundary` class. Boundaries in 3D space can be used to represent the shape of an object in 3D space. Can be used in all the manner of the base class, but also provides methods for getting the z interval of the boundary and for getting a slice of the 3D Shape at a certain z value."""

    @abstractmethod
    def get_slice(z: SpatialMeasurement) -> Boundary:
        """Returns a slice of the boundary at a certain z value.

        :param z: The z value of the slice.
        :type z: :class:`SpatialMeasurement`
        :return: The 2-dimensional slice of the boundary at the given z value.
        :rtype: :class:`Boundary`
        """

        pass

    @abstractmethod
    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary3D:
        pass

    def get_3d(
        self, z_start: SpatialMeasurement, z_end: SpatialMeasurement
    ) -> "Boundary3D":
        """Returns the 3D version of the boundary. Simply returns self for already 3D boundaries.

        :param z_start: The starting z value of the boundary.
        :type z_start: :class:`SpatialMeasurement`
        :param z_end: The ending z value of the boundary.
        :type z_end: :class:`SpatialMeasurement`
        :return: The 3D version of the boundary.
        :rtype: :class:`Boundary3D`
        """

        return self


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
        return Cylinder(self.x, self.y, self.radius, z_start, z_end)


class Cylinder(Circle, DiscreteBoundary3D):
    """Class for representing the 3D-version of a circle, also known as a cylinder, extending the :class:`Circle` and :class:`DiscreteBoundary3D` classes."""

    def __init__(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        radius: SpatialMeasurement,
        z_start: SpatialMeasurement,
        z_end: SpatialMeasurement,
    ):
        super().__init__(x, y, radius)
        self.z_start = z_start
        self.z_end = z_end

    def get_z_interval(self) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        return (self.z_start, self.z_end)

    def get_slice(self, z: SpatialMeasurement) -> DiscreteBoundary:
        if z < self.z_start or z > self.z_end:
            return None
        return self


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

    def scale(self, factor: SpatialMeasurement) -> "Polygon":
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


class PolygonalPrism(Polygon, DiscreteBoundary3D):
    """Class for representing the 3D-version of a polygon, also known as a polygonal prism, extending the :class:`Polygon` and :class:`DiscreteBoundary3D` classes."""

    def __init__(
        self,
        points: List[Tuple[SpatialMeasurement, SpatialMeasurement]],
        z_start: SpatialMeasurement,
        z_end: SpatialMeasurement,
    ):
        super().__init__(points)
        self.z_start = z_start
        self.z_end = z_end

    def get_z_interval(self) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        return (self.z_start, self.z_end)

    def get_slice(self, z: SpatialMeasurement) -> DiscreteBoundary:
        if z < self.z_start or z > self.z_end:
            return None
        return self


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
        self.x = x
        self.y = y
        self.z = z

    def get_z_interval(self) -> Tuple[SpatialMeasurement]:
        return (self.z, self.z)

    def __str__(self):
        return f"Point(x={self.x}, y={self.y}, z={self.z})"

    def translate(self, x: SpatialMeasurement, y: SpatialMeasurement) -> "Point":
        self.x += x
        self.y += y
        return self

    def scale(self, factor: SpatialMeasurement) -> "Point":
        self.x *= factor
        self.y *= factor
        return self

    def reflect_y(self, axis: SpatialMeasurement) -> "Point":
        self.y = ReflectValue1D(self.y, axis)
        return self

    def reflect_x(self, axis: SpatialMeasurement) -> "Point":
        self.x = ReflectValue1D(self.x, axis)
        return self

    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary:
        return self

    def get_vertices(self) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        return [(self.x, self.y)]


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
            new_vector = RotateAboutOrigin(centerToObject, -angle)
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
    :rtype: List[:class:`BoundedObject
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
    :rtype: List[:class:`DiscreteBoundary
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
    :rtype: List[:class:`DiscreteBoundary
    """

    return [
        bound.discretized(discretization_quality).buffered(robot_radius)
        for bound in bounds
    ]
