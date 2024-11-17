from abc import ABC, abstractmethod
from math import cos, pi, sin
from typing import List

from gamegine.render.drawable import Drawable
from gamegine.representation.boundary.boundary import Boundary
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter
from gamegine.utils.NCIM.vector import Vector2D
import shapely.geometry as sg

from gamegine.utils.transform import Transform2D, Translation2D


class Shape(ABC, Drawable):
    """Abstract base class for shapes."""

    def __init__(self):
        super().__init__()
        self._vertice_cache = None
        self._vertice_modified = True

    def get_vertices(
        self, discretization_segments: int = 16
    ) -> List[Vector2D[SpatialMeasurement]]:
        """Return the vertices of the shape in local coordinates."""
        if self._vertice_modified:
            self._vertice_cache = self.compute_vertices(discretization_segments)
            self._vertice_modified = False
        return self._vertice_cache

    def get_plain_vertices(self):
        return [vertex.to(Meter) for vertex in self.get_vertices()]

    @abstractmethod
    def compute_vertices(self, discretization_segments: int):
        """Compute the vertices of the shape in local coordinates."""
        pass

    @abstractmethod
    def reflect_x(self):
        pass

    @abstractmethod
    def reflect_y(self):
        pass

    @abstractmethod
    def intersects_line(
        self, start: Vector2D[SpatialMeasurement], end: Vector2D[SpatialMeasurement]
    ) -> bool:
        pass

    @abstractmethod
    def contains_point(self, point: Vector2D[SpatialMeasurement]) -> bool:
        pass

    @abstractmethod
    def intersects_box(
        self, start: Vector2D[SpatialMeasurement], end: Vector2D[SpatialMeasurement]
    ) -> bool:
        pass

    def get_bounded_rectangle(self):
        """Returns the bounding rectangle of the shape."""
        vertices = self.get_vertices()
        min_x = min(vertex.x for vertex in vertices)
        max_x = max(vertex.x for vertex in vertices)
        min_y = min(vertex.y for vertex in vertices)
        max_y = max(vertex.y for vertex in vertices)
        return min_x, max_x, min_y, max_y

    def at(self, x: SpatialMeasurement, y: SpatialMeasurement):
        return Boundary(self, Transform2D(Translation2D(x, y)))


class ShapelyImplementedShape(Shape):
    def intersects_line(
        self, start: Vector2D[SpatialMeasurement], end: Vector2D[SpatialMeasurement]
    ) -> bool:
        return sg.Polygon(self.get_plain_vertices()).intersects(
            sg.LineString([start.to(Meter), end.to(Meter)])
        )

    def contains_point(self, point: Vector2D[SpatialMeasurement]) -> bool:
        return sg.Polygon(self.get_plain_vertices()).contains(sg.Point(point.to(Meter)))

    def intersects_box(
        self, start: Vector2D[SpatialMeasurement], end: Vector2D[SpatialMeasurement]
    ) -> bool:
        return sg.Polygon(self.get_plain_vertices()).intersects(
            sg.box(
                start.x.to(Meter),
                start.y.to(Meter),
                end.x.to(Meter),
                end.y.to(Meter),
            )
        )


class Circle(ShapelyImplementedShape):
    """Class representing a circle."""

    def __init__(self, radius: SpatialMeasurement):
        super().__init__()
        self.radius = radius

    def compute_vertices(self, discretization_segments: int = 16):
        return [
            Vector2D(
                self.radius * cos(2 * pi * i / discretization_segments),
                self.radius * sin(2 * pi * i / discretization_segments),
            )
            for i in range(discretization_segments)
        ]

    def reflect_x(self):
        return self

    def reflect_y(self):
        return self


class Rectangle(ShapelyImplementedShape):
    """Class representing a rectangle."""

    def __init__(self, width: SpatialMeasurement, height: SpatialMeasurement):
        super().__init__()
        self.width = width
        self.height = height

    def compute_vertices(self, discretization_segments: int = 16):
        return [
            Vector2D(self.width / 2, self.height / 2),
            Vector2D(-self.width / 2, self.height / 2),
            Vector2D(-self.width / 2, -self.height / 2),
            Vector2D(self.width / 2, -self.height / 2),
        ]

    def reflect_x(self):
        return self

    def reflect_y(self):
        return self


class IsocolesTriangle(ShapelyImplementedShape):
    """Class representing an isocoles triangle."""

    def __init__(self, base: SpatialMeasurement, height: SpatialMeasurement):
        super().__init__()
        self.base = base
        self.height = height

    def compute_vertices(self, discretization_segments: int = 16):
        return [
            Vector2D(0, self.height / 2),
            Vector2D(-self.base / 2, -self.height / 2),
            Vector2D(self.base / 2, -self.height / 2),
        ]

    def reflect_x(self):
        self.height = -self.height
        return self

    def reflect_y(self):
        return self


class Polygon(ShapelyImplementedShape):
    """Class representing a polygon."""

    def __init__(self, vertices: List[Vector2D[SpatialMeasurement]]):
        super().__init__()
        self.vertices = vertices

    def compute_vertices(self, discretization_segments: int = 16):
        return self.vertices

    def reflect_x(self):
        return Polygon([vertex.reflect_x() for vertex in self.vertices])

    def reflect_y(self):
        return Polygon([vertex.reflect_y() for vertex in self.vertices])

    @classmethod
    def createCenteredBound(cls, vertices: List[Vector2D[SpatialMeasurement]]):
        min_x = min(vertex.x for vertex in vertices)
        max_x = max(vertex.x for vertex in vertices)
        min_y = min(vertex.y for vertex in vertices)
        max_y = max(vertex.y for vertex in vertices)
        center = Vector2D((min_x + max_x) / 2, (min_y + max_y) / 2)
        return Polygon([vertex - center for vertex in vertices]).at(center.x, center.y)
