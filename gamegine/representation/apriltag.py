import math
# Rendering is handled by gamegine.render.handlers, not embedded here
from gamegine.representation.bounds import Boundary, BoundedObject, Point
from gamegine.utils.NCIM.Dimensions.angular import Degree, Radian
from gamegine.utils.NCIM.Dimensions.spatial import Centimeter, Inch, Meter
from gamegine.utils.NCIM.ncim import AngularMeasurement, SpatialMeasurement
from enum import Enum


class AprilTagFamily(Enum):
    TAG_16h5 = 0
    TAG_25h9 = 1
    TAG_36h11 = 2


class AprilTag(BoundedObject):
    """A representation of an AprilTag used on FRC fields. Contains information about the tag's position, orientation, and family.

    :param x: The x-coordinate of the tag.
    :type x: :class:`SpatialMeasurement`
    :param y: The y-coordinate of the tag.
    :type y: :class:`SpatialMeasurement`
    :param z: The z-coordinate of the tag, or the height of the tag.
    :type z: :class:`SpatialMeasurement`
    :param heading: The heading of the tag.
    :type heading: :class:`AngularMeasurement`
    :param id: The ID of the tag.
    :type id: int
    :param family: The family of the tag.
    :type family: :class:`AprilTagFamily`
    """

    def __init__(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        z: SpatialMeasurement,
        heading: AngularMeasurement,
        id: int,
        family: AprilTagFamily,
        size: SpatialMeasurement = Inch(10.5),
    ) -> None:
        self.position = Point(x, y, z)
        self.id = id
        self.family = family
        self.heading = heading
        self.name = f"AprilTag {id}"
        self.size = size
        super().__init__(self.position)


