from abc import ABC, abstractmethod
from dataclasses import dataclass
import os
import tempfile
from typing import List, Tuple

import numpy as np
from gamegine.utils.geometrika import construct_faces_from_prism_vertices
import pybullet as p
from gamegine.simulation.environment import PYBULLET_UNITS, curr_temp_count
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
    mesh_file: str = None
    vertices: Tuple[Tuple[SpatialMeasurement]] = None
    triangles: Tuple[Tuple[int]] = None
    scale: SpatialMeasurement = PYBULLET_UNITS.SPATIAL(1)

    def create_shape(self):
        scale = self.scale.to(PYBULLET_UNITS.SPATIAL)

        if self.mesh_file:
            return p.createCollisionShape(
                p.GEOM_MESH,
                fileName=self.mesh_file,
                meshScale=[scale, scale, scale],
            )
        elif self.vertices and self.triangles:

            bullet_vertices = [
                [v.to(PYBULLET_UNITS.SPATIAL) for v in vertex]
                for vertex in self.vertices
            ]
            bullet_vertices = bullet_vertices
            bullet_indices = self.triangles

            # Check if directory exists and create if not
            os.makedirs("temp", exist_ok=True)

            global curr_temp_count
            # TODO: Should be much better way to do this
            with open("temp/temp{0}.obj".format(curr_temp_count), "wb") as temp_file:
                temp_file.write(b"o PolygonMesh \n")
                for item in bullet_vertices:
                    temp_file.write(
                        str.encode("v {0} {1} {2}\n".format(item[0], item[1], item[2]))
                    )
                temp_file.write(b"s off\n")
                for item in bullet_indices:
                    temp_file.write(
                        str.encode("f " + " ".join([str(i + 1) for i in item]) + "\n")
                    )
                temp_file.close()

            obj = p.createCollisionShape(
                p.GEOM_MESH,
                fileName="temp/temp{0}.obj".format(curr_temp_count),
                meshScale=[scale, scale, scale],
            )
            curr_temp_count += 1
            return obj

        raise ValueError("Either mesh_file or vertices and triangles must be provided")


def PolygonalPrism(
    vertices: List[Tuple[SpatialMeasurement, SpatialMeasurement]],
    height: SpatialMeasurement,
) -> BulletMesh:
    vertices = [
        (vertex[0], vertex[1], PYBULLET_UNITS.SPATIAL(0)) for vertex in vertices
    ]

    vertices.extend(
        [
            (
                vertex[0],
                vertex[1],
                height,
            )
            for vertex in vertices
        ]
    )

    triangles = construct_faces_from_prism_vertices(vertices)

    return BulletMesh(
        vertices=tuple(vertices),
        triangles=tuple(triangles),
        scale=PYBULLET_UNITS.SPATIAL(1),
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
