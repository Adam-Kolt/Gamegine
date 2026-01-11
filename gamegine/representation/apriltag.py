import math
from typing import Tuple
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
    :param heading: The heading of the tag (direction the tag faces).
    :type heading: :class:`AngularMeasurement`
    :param id: The ID of the tag.
    :type id: int
    :param family: The family of the tag.
    :type family: :class:`AprilTagFamily`
    :param size: The size (width/height) of the tag. Defaults to 10.5 inches.
    :type size: :class:`SpatialMeasurement`
    :param visible_arc: The angular range within which the tag is detectable from nominal. Defaults to 160 degrees.
    :type visible_arc: :class:`AngularMeasurement`
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
        visible_arc: AngularMeasurement = Degree(160),
    ) -> None:
        self.position = Point(x, y, z)
        self.id = id
        self.family = family
        self.heading = heading
        self.name = f"AprilTag {id}"
        self.size = size
        self.visible_arc = visible_arc
        super().__init__(self.position)

    def get_normal_vector(self) -> Tuple[float, float, float]:
        """Returns the 3D unit vector pointing outward from the tag face.
        
        The normal vector points in the direction the tag is facing based on its heading.
        For a tag mounted on a wall, this would point away from the wall.
        
        :return: Tuple (x, y, z) representing the unit normal vector.
        :rtype: Tuple[float, float, float]
        """
        heading_rad = float(self.heading.to(Radian))
        # Normal points outward in XY plane based on heading, no Z component
        return (math.cos(heading_rad), math.sin(heading_rad), 0.0)

    def get_normal_vector_2d(self) -> Tuple[float, float]:
        """Returns the 2D unit vector pointing outward from the tag face.
        
        :return: Tuple (x, y) representing the unit normal vector in the XY plane.
        :rtype: Tuple[float, float]
        """
        heading_rad = float(self.heading.to(Radian))
        return (math.cos(heading_rad), math.sin(heading_rad))

    def is_visible_from_angle(self, viewing_angle: AngularMeasurement) -> bool:
        """Check if the tag is visible when viewed from a given angle.
        
        The viewing angle is the direction FROM the camera TO the tag. For the tag
        to be visible, the camera must be within the tag's visible arc (centered
        on the tag's normal vector).
        
        :param viewing_angle: The angle from which the tag is being viewed.
        :type viewing_angle: :class:`AngularMeasurement`
        :return: True if the tag is visible from this angle, False otherwise.
        :rtype: bool
        """
        # Calculate angle difference between viewing direction and tag normal
        # The viewing angle is FROM camera TO tag, so we need the opposite direction
        # to compare with the tag's normal (which points outward from tag)
        tag_heading_rad = float(self.heading.to(Radian))
        view_rad = float(viewing_angle.to(Radian))
        
        # The camera sees the tag when it's looking at the front of the tag
        # So we compare viewing angle with tag heading + 180 degrees (back of tag from camera's view)
        opposite_view = view_rad + math.pi  # Direction from tag to camera
        
        # Calculate angular difference
        diff = abs(opposite_view - tag_heading_rad)
        # Normalize to [0, pi]
        while diff > math.pi:
            diff = abs(diff - 2 * math.pi)
        
        # Check if within half the visible arc
        half_arc_rad = float(self.visible_arc.to(Radian)) / 2
        return diff <= half_arc_rad

    def get_viewing_angle_from_position(
        self,
        viewer_x: SpatialMeasurement,
        viewer_y: SpatialMeasurement,
    ) -> AngularMeasurement:
        """Calculate the viewing angle from a position to this tag.
        
        :param viewer_x: X coordinate of viewer position.
        :param viewer_y: Y coordinate of viewer position.
        :return: Angle from viewer position toward the tag.
        :rtype: :class:`AngularMeasurement`
        """
        dx = float(self.position.x.to(Meter)) - float(viewer_x.to(Meter))
        dy = float(self.position.y.to(Meter)) - float(viewer_y.to(Meter))
        return Radian(math.atan2(dy, dx))

    def is_visible_from_position(
        self,
        viewer_x: SpatialMeasurement,
        viewer_y: SpatialMeasurement,
    ) -> bool:
        """Check if the tag is visible from a given XY position.
        
        This only checks the angular constraint (tag orientation), not obstacles.
        
        :param viewer_x: X coordinate of viewer position.
        :param viewer_y: Y coordinate of viewer position.
        :return: True if the tag face is oriented toward the viewer position.
        :rtype: bool
        """
        viewing_angle = self.get_viewing_angle_from_position(viewer_x, viewer_y)
        return self.is_visible_from_angle(viewing_angle)
