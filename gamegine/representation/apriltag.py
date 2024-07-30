from gamegine.representation.bounds import Boundary, BoundedObject, Point
from gamegine.utils.unit import AngularMeasurement, SpatialMeasurement
from enum import Enum


class AprilTagFamily(Enum):
    TAG_16h5 = 0
    TAG_25h9 = 1
    TAG_36h11 = 2


class AprilTag(BoundedObject):

    def __init__(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        z: SpatialMeasurement,
        heading: AngularMeasurement,
        id: int,
        family: AprilTagFamily,
    ) -> None:
        self.position = Point(x, y, z)
        self.id = id
        self.family = family
        self.heading = heading
        self.name = f"AprilTag {id}"
        super().__init__(self.position)
