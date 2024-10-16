from abc import ABC, abstractmethod
from dataclasses import dataclass
import pybullet as p
from gamegine.simulation.environment import PYBULLET_UNITS
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement


@dataclass(frozen=True)
class BulletShape(ABC):
    def create_shape(self):
        pass


@dataclass(frozen=True)
class BulletBox(BulletShape):
    width: SpatialMeasurement
    length: SpatialMeasurement
    height: SpatialMeasurement

    def create_shape(self):
        halfExtents = [
            self.width.to(PYBULLET_UNITS.SPATIAL) / 2,
            self.length.to(PYBULLET_UNITS.SPATIAL) / 2,
            self.height.to(PYBULLET_UNITS.SPATIAL) / 2,
        ]

        return p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=halfExtents,
        )


@dataclass(frozen=True)
class BulletSphere(BulletShape):
    radius: SpatialMeasurement

    def create_shape(self):
        radius = self.radius.to(PYBULLET_UNITS.SPATIAL)

        return p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=radius,
        )


@dataclass(frozen=True)
class BulletCylinder(BulletShape):
    radius: SpatialMeasurement
    height: SpatialMeasurement

    def create_shape(self):
        radius = self.radius.to(PYBULLET_UNITS.SPATIAL)
        height = self.height.to(PYBULLET_UNITS.SPATIAL)

        return p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=radius,
            height=height,
        )


@dataclass(frozen=True)
class BulletCapsule(BulletShape):
    radius: SpatialMeasurement
    height: SpatialMeasurement

    def create_shape(self):
        radius = self.radius.to(PYBULLET_UNITS.SPATIAL)
        height = self.height.to(PYBULLET_UNITS.SPATIAL)

        return p.createCollisionShape(
            p.GEOM_CAPSULE,
            radius=radius,
            height=height,
        )


@dataclass(frozen=True)
class BulletMesh(BulletShape):
    mesh_file: str
    scale: SpatialMeasurement

    def create_shape(self):
        scale = self.scale.to(PYBULLET_UNITS.SPATIAL)

        return p.createCollisionShape(
            p.GEOM_MESH,
            fileName=self.mesh_file,
            meshScale=[scale, scale, scale],
        )


@dataclass(frozen=True)
class BulletPlane(BulletShape):
    width: SpatialMeasurement
    length: SpatialMeasurement
    thickness: SpatialMeasurement

    def create_shape(self):
        width = self.width.to(PYBULLET_UNITS.SPATIAL)
        length = self.length.to(PYBULLET_UNITS.SPATIAL)
        thickness = self.thickness.to(PYBULLET_UNITS.SPATIAL)

        return p.createCollisionShape(
            p.GEOM_PLANE,
            halfExtents=[width, length, thickness],
        )


@dataclass(frozen=True)
class BulletCompound(BulletShape):
    shapes: tuple[BulletShape]

    def create_shape(self):
        child_shapes = [shape.create_shape() for shape in self.shapes]

        return p.createCollisionShape(
            p.GEOM_COMPOUND,
            childShapes=child_shapes,
        )


class ShapeBuilder:
    __cache = {}

    @classmethod
    def build(cls, shape: BulletShape):
        if shape not in cls.__cache:
            cls.__cache[shape] = shape.create_shape()

        return cls.__cache[shape]
