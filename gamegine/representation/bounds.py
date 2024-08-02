from typing import Callable, List, Tuple
from abc import ABC, abstractmethod
import copy

import pint
import math

import pygame
from gamegine.render.drawable import Drawable
from gamegine.render.style import Palette
from gamegine.representation.base import NamedObject
from gamegine.utils.logging import Debug
from gamegine.utils.matematika import ReflectValue1D, RotateAboutOrigin
import shapely.geometry as sg

from gamegine.utils.unit import AngularMeasurement, Inch, RatioOf, SpatialMeasurement


class Boundary(ABC):

    @abstractmethod
    def translate(self, x: SpatialMeasurement, y: SpatialMeasurement) -> "Boundary":
        pass

    @abstractmethod
    def scale(self, factor: SpatialMeasurement) -> "Boundary":
        pass

    @abstractmethod
    def reflect_x(self, axis: SpatialMeasurement) -> "Boundary":
        pass

    @abstractmethod
    def reflect_y(self, axis: SpatialMeasurement) -> "Boundary":
        pass

    @abstractmethod
    def get_3d(
        self, z_start: SpatialMeasurement = 0, z_end: SpatialMeasurement = 0
    ) -> "Boundary3D":
        pass

    @abstractmethod
    def discretized(self, curve_segments: int = 5) -> "DiscreteBoundary":
        pass


class DiscreteBoundary(Boundary, Drawable):
    @abstractmethod
    def get_vertices(self) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        pass

    def discretized(self, curve_segments: int = 5) -> "DiscreteBoundary":
        return self

    def __recompute_plain_points(self):
        self.plain_points = [point for point in self.get_vertices()]

    def intersects_line(
        self,
        x1: SpatialMeasurement,
        y1: SpatialMeasurement,
        x2: SpatialMeasurement,
        y2: SpatialMeasurement,
    ) -> bool:
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
        self.__recompute_plain_points()
        return sg.Polygon(self.plain_points).intersects(sg.box(x, y, max_x, max_y))

    def contains_point(self, x: SpatialMeasurement, y: SpatialMeasurement) -> bool:
        self.__recompute_plain_points()
        return sg.Polygon(self.plain_points).contains(sg.Point((x, y)))

    def __convert_coordinate_sequence(
        self, coord_sequence
    ) -> List[Tuple[float, float]]:
        return [(x, y) for x, y in coord_sequence]

    def get_bounded_rectangle(self) -> "Rectangle":
        self.__recompute_plain_points()
        min_x = min([point[0] for point in self.plain_points])
        min_y = min([point[1] for point in self.plain_points])
        max_x = max([point[0] for point in self.plain_points])
        max_y = max([point[1] for point in self.plain_points])
        return Rectangle(min_x, min_y, max_x - min_x, max_y - min_y)

    def draw(self, render_scale: SpatialMeasurement) -> None:
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
        self.__recompute_plain_points()
        coords = self.__convert_coordinate_sequence(
            sg.Polygon(self.plain_points).buffer(distance, quad_segs=1).exterior.coords
        )
        return Polygon(coords)

    def get_3d(
        self, z_start: SpatialMeasurement = 0, z_end: SpatialMeasurement = 0
    ) -> "DiscreteBoundary3D":
        return PolygonalPrism(self.get_vertices(), z_start, z_end)


class DiscreteBoundary3D(DiscreteBoundary):
    def get_slice(self, z: SpatialMeasurement) -> DiscreteBoundary:
        interval = self.get_z_interval()
        if z < interval[0] or z > interval[1]:
            return None
        return self

    def discretized(self, curve_segments: int = 5) -> "DiscreteBoundary3D":
        return self

    @abstractmethod
    def get_z_interval(self) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        pass

    def get_3d(
        self, z_start: SpatialMeasurement = 0, z_end: SpatialMeasurement = 0
    ) -> "DiscreteBoundary3D":
        return self


class Boundary3D(Boundary):
    @abstractmethod
    def get_slice(z: SpatialMeasurement) -> Boundary:
        pass

    @abstractmethod
    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary3D:
        pass

    def get_3d(
        self, z_start: SpatialMeasurement, z_end: SpatialMeasurement
    ) -> "Boundary3D":
        return self


class Rectangle(DiscreteBoundary):
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


class Square(Rectangle):
    def __init__(
        self, x: SpatialMeasurement, y: SpatialMeasurement, side: SpatialMeasurement
    ):
        super().__init__(x, y, side, side)

    def __str__(self):
        return f"Square(x={self.x}, y={self.y}, side={self.width})"


class Circle(Boundary):
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
    def __init__(self, bounds: Boundary) -> None:
        self.bounds = bounds

    def mirrored_over_horizontal(self, axis: SpatialMeasurement) -> "BoundedObject":
        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.reflect_y(axis)
        return obj

    def mirrored_over_horizontal_ip(self, axis: SpatialMeasurement) -> None:
        self.bounds = self.bounds.reflect_y(axis)

    def mirrored_over_vertical(self, axis: SpatialMeasurement) -> "BoundedObject":
        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.reflect_x(axis)
        return obj

    def mirrored_over_vertical_ip(self, axis: SpatialMeasurement) -> "BoundedObject":
        self.bounds = self.bounds.reflect_x(axis)
        return self

    def scaled(self, factor: SpatialMeasurement) -> "BoundedObject":
        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.scale(factor)
        return obj

    def translated(
        self, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> "BoundedObject":
        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.translate(x, y)
        return obj

    def translate_ip(
        self, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> "BoundedObject":
        self.bounds = self.bounds.translate(x, y)
        return self

    def discretized(self, curve_segments: int = 5) -> "BoundedObject":
        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.discretized(curve_segments)
        return obj

    def discretize_ip(self, curve_segments: int = 5) -> "BoundedObject":
        self.bounds = self.bounds.discretized(curve_segments)
        return self


def LineIntersectsAnyBound(
    bounds: List[DiscreteBoundary],
    x1: SpatialMeasurement,
    y1: SpatialMeasurement,
    x2: SpatialMeasurement,
    y2: SpatialMeasurement,
) -> bool:
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
    return [obj.prefix(prefix_og) for obj in objects] + [
        obj.mirrored_over_vertical(axis).prefix(prefix) for obj in objects
    ]


def SymmetricalY(
    objects: List[BoundedObject],
    axis: SpatialMeasurement,
    prefix: str,
    prefix_og: str = "",
) -> List[BoundedObject]:
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
    return [
        object.bounds.discretized(discretization_quality).buffered(robot_radius)
        for object in objects
    ]


def ExpandedBounds(
    bounds: List[Boundary],
    robot_radius: SpatialMeasurement = Inch(21),
    discretization_quality=4,
) -> List[DiscreteBoundary]:
    return [
        bound.discretized(discretization_quality).buffered(robot_radius)
        for bound in bounds
    ]
