import copy
from typing import Callable, List, Tuple
from gamegine.render.drawable import Drawable
from gamegine.representation.base import NamedObject

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from gamegine.representation.boundary.shape2D import Shape
    from gamegine.representation.boundary.shape3D import Shape3D
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter, Inch
import pygame
from gamegine.utils.logging import Debug
from gamegine.utils.matematika import RotateAboutOrigin
from gamegine.utils.transform import (
    Transform2D,
    Transform3D,
    Translation2D,
    Translation3D,
)


class Boundary(Drawable):
    def __init__(self, shape: Shape, transform: Transform2D):
        self.shape = shape
        self.transform = transform

    def translate(self, translation: Translation2D):
        self.transform.translate(translation)

    def scale(self, factor: float):
        self.transform.scale(factor)

    def reflect_x(self, axis: SpatialMeasurement):
        self.transform.translate(
            Translation2D(
                self.transform.position.x, 2 * axis - self.transform.position.y
            )
        )

        self.shape.reflect_x()

    def reflect_y(self, axis: SpatialMeasurement):
        self.transform.translate(
            Translation2D(
                2 * axis - self.transform.position.x, self.transform.position.y
            )
        )

        self.shape.reflect_y()

    def draw(self, render_scale: SpatialMeasurement):

        verts = self.shape.get_vertices()

        transformed_verts = self.transform.apply(verts)

        draw_points = []
        for i in range(len(transformed_verts)):
            start = transformed_verts[i]
            end = transformed_verts[(i + 1) % len(transformed_verts)]

            start = start / render_scale
            end = end / render_scale

            draw_points.append((start.x, start.y))

        pygame.draw.polygon(pygame.display.get_surface(), (255, 255, 255), draw_points)


class Boundary3D(Boundary):
    def __init__(self, shape: Shape3D, transform: Transform3D):
        self.shape = shape
        self.transform = transform

    def translate(self, translation: Translation3D):
        self.transform.translate(translation)

    def scale(self, factor: float):
        self.transform.scale(factor)

    def reflect_x(self, axis: SpatialMeasurement):
        self.transform.translate(
            Translation3D(
                self.transform.position.x,
                2 * axis - self.transform.position.y,
                self.transform.position.z,
            )
        )

        self.shape.reflect_x()

    def reflect_y(self, axis: SpatialMeasurement):
        self.transform.translate(
            Translation3D(
                2 * axis - self.transform.position.x,
                self.transform.position.y,
                self.transform.position.z,
            )
        )

        self.shape.reflect_y()


def LineIntersectsAnyBound(
    bounds: List[Boundary],
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
        if bound.intersects_line(x1, y1, x2, y2):
            return True
    return False


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
        obj.bounds = obj.bounds.reflect_x(axis)
        return obj

    def mirrored_over_horizontal_ip(self, axis: SpatialMeasurement) -> None:
        """Mirrors the object over the horizontal axis at the given axis in place, not returning a new object.

        :param axis: The axis to reflect the object over.
        :type axis: :class:`SpatialMeasurement`
        """

        self.bounds = self.bounds.reflect_x(axis)

    def mirrored_over_vertical(self, axis: SpatialMeasurement) -> "BoundedObject":
        """Returns a new object which is a mirror of the object over the vertical axis at the given axis.

        :param axis: The axis to reflect the object over.
        :type axis: :class:`SpatialMeasurement`
        :return: The mirrored object.
        :rtype: :class:`BoundedObject`
        """

        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.reflect_y(axis)
        return obj

    def mirrored_over_vertical_ip(self, axis: SpatialMeasurement) -> "BoundedObject":
        """Mirrors the object over the vertical axis at the given axis in place, not returning a new object.

        :param axis: The axis to reflect the object over.
        :type axis: :class:`SpatialMeasurement`
        :return: This object.
        :rtype: :class:`BoundedObject`
        """

        self.bounds = self.bounds.reflect_y(axis)
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
        centerToObject = [
            object.bounds.transform.position.x - center[0],
            object.bounds.transform.position.y - center[1],
        ]
        objectToCenter = [centerToObject[0] * -1, centerToObject[1] * -1]

        for i in range(1, num_objects):
            angle = angle_increment * i
            Debug(angle)
            new_vector = RotateAboutOrigin(centerToObject, -angle)
            # TODO: Add rotation too
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
) -> List[Boundary]:

    return [
        object.bounds.discretized(discretization_quality).buffered(robot_radius)
        for object in objects
    ]


def ExpandedBounds(
    bounds: List[Boundary],
    robot_radius: SpatialMeasurement = Inch(21),
    discretization_quality=4,
) -> List[Boundary]:

    return [
        bound.discretized(discretization_quality).buffered(robot_radius)
        for bound in bounds
    ]
