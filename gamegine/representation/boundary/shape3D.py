from abc import abstractmethod
from math import cos, pi, sin
from tkinter.tix import Meter
from typing import List
from gamegine.representation.boundary.boundary import Boundary3D
from gamegine.representation.boundary.shape2D import (
    Circle,
    Polygon,
    Shape,
    ShapelyImplementedShape,
)
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement
from gamegine.utils.NCIM.ncim import Zero
from gamegine.utils.NCIM.vector import Vector2D, Vector3D
from gamegine.utils.transform import Transform3D, Translation3D


class Shape3D(Shape):
    def __init__(self):
        super().__init__()
        self._3d_vertices_cache = None

    def compute_vertices(self, discretization_segments=16):
        return self.project_to_2D().compute_vertices(discretization_segments)

    @abstractmethod
    def get_2d_slice(self, z: SpatialMeasurement) -> Shape:
        pass

    @abstractmethod
    def project_to_2D(self) -> Shape:
        pass

    def get_3d_vertices(self) -> List[Vector3D[SpatialMeasurement]]:
        if self._vertice_modified:
            self._3d_vertices_cache = self.compute_3d_vertices(16)
            self._vertice_modified = False
        return self._3d_vertices_cache

    @abstractmethod
    def compute_3d_vertices(
        self, discretization_segments: int
    ) -> List[Vector3D[SpatialMeasurement]]:
        pass

    @abstractmethod
    def get_face_indices(self) -> List[List[int]]:
        pass

    def get_3d_plain_vertices(self):
        return [vertex.to(Meter) for vertex in self.get_3d_vertices()]

    def at(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        z: SpatialMeasurement = Zero(),
    ):
        return Boundary3D(self, Transform3D(Translation3D(x, y, z)))


class Cylinder(Shape3D, ShapelyImplementedShape):
    def __init__(self, radius: SpatialMeasurement, height: SpatialMeasurement):
        super().__init__()
        self.radius = radius
        self.height = height

    def get_2d_slice(self, z: SpatialMeasurement) -> Circle:
        return Circle(self.radius)

    def project_to_2D(self) -> Circle:
        return Circle(self.radius)

    def compute_vertices(
        self, discretization_segments: int
    ) -> List[Vector3D[SpatialMeasurement]]:
        self.project_to_2D().compute_vertices(discretization_segments)

    def compute_3d_vertices(
        self, discretization_segments: int
    ) -> List[Vector3D[SpatialMeasurement]]:
        vertices = []
        face_vertices = self.get_vertices(discretization_segments)

        for vertice in face_vertices:
            vertices.append(Vector3D(vertice.x, vertice.y, -self.height / 2))
            vertices.append(Vector3D(vertice.x, vertice.y, self.height / 2))

        return vertices

    def project_to_2D(self) -> Circle:
        return Circle(self.radius)

    def reflect_x(self):
        return self

    def reflect_y(self):
        return self


class RectangularPrism(Shape3D, ShapelyImplementedShape):
    def __init__(
        self,
        width: SpatialMeasurement,
        length: SpatialMeasurement,
        height: SpatialMeasurement,
    ):
        super().__init__()
        self.width = width
        self.length = length
        self.height = height

    def get_2d_slice(self, z: SpatialMeasurement) -> Shape:
        return RectangularPrism(self.width, self.length, Meter(0))

    def project_to_2D(self) -> Shape:
        return RectangularPrism(self.width, self.length, Meter(0))

    def compute_vertices(
        self, discretization_segments: int
    ) -> List[Vector3D[SpatialMeasurement]]:
        self.project_to_2D().compute_vertices(discretization_segments)

    def compute_3d_vertices(
        self, discretization_segments: int
    ) -> List[Vector3D[SpatialMeasurement]]:
        vertices = []
        face_vertices = self.get_vertices(discretization_segments)

        for vertice in face_vertices:
            vertices.append(Vector3D(vertice.x, vertice.y, -self.height / 2))
            vertices.append(Vector3D(vertice.x, vertice.y, self.height / 2))

        return vertices

    def project_to_2D(self) -> Shape:
        return RectangularPrism(self.width, self.length, Meter(0))

    def reflect_x(self):
        return self

    def reflect_y(self):
        return self

    def get_face_indices(self) -> List[List[int]]:
        pass


class Sphere(Shape3D, ShapelyImplementedShape):
    def __init__(self, radius: SpatialMeasurement):
        super().__init__()
        self.radius = radius

    def get_2d_slice(self, z: SpatialMeasurement) -> Circle:
        radius = (self.radius**2 - z**2) ** 0.5
        return Circle(radius)

    def project_to_2D(self) -> Circle:
        return Circle(self.radius)

    def compute_vertices(
        self, discretization_segments: int
    ) -> List[Vector3D[SpatialMeasurement]]:
        self.project_to_2D().compute_vertices(discretization_segments)

    def compute_3d_vertices(
        self, discretization_segments: int
    ) -> List[Vector3D[SpatialMeasurement]]:
        vertices = []
        face_vertices = self.get_vertices(discretization_segments)

        for vertice in face_vertices:
            vertices.append(Vector3D(vertice.x, vertice.y, -self.radius))
            vertices.append(Vector3D(vertice.x, vertice.y, self.radius))

        return vertices

    def reflect_x(self):
        return self

    def reflect_y(self):
        return self


class PolygonalPrism(Shape3D, ShapelyImplementedShape):
    def __init__(
        self, vertices: List[Vector2D[SpatialMeasurement]], height: SpatialMeasurement
    ):
        super().__init__()
        self.vertices = vertices
        self.height = height

    def get_2d_slice(self, z: SpatialMeasurement) -> Shape:
        return Polygon([vertex for vertex in self.vertices])

    def project_to_2D(self) -> Shape:
        return Polygon([vertex for vertex in self.vertices])

    def compute_vertices(
        self, discretization_segments: int
    ) -> List[Vector3D[SpatialMeasurement]]:
        self.project_to_2D().compute_vertices(discretization_segments)

    def compute_3d_vertices(
        self, discretization_segments: int
    ) -> List[Vector3D[SpatialMeasurement]]:
        vertices = []
        face_vertices = self.get_vertices(discretization_segments)

        for vertice in face_vertices:
            vertices.append(Vector3D(vertice.x, vertice.y, -self.height / 2))
            vertices.append(Vector3D(vertice.x, vertice.y, self.height / 2))

        return vertices

    def project_to_2D(self) -> Shape:
        return Polygon([vertex for vertex in self.vertices])

    def reflect_x(self):
        self.vertices = [vertex.reflect_x() for vertex in self.vertices]

    def reflect_y(self):
        self.vertices = [vertex.reflect_y() for vertex in self.vertices]

    def get_face_indices(self) -> List[List[int]]:
        pass
