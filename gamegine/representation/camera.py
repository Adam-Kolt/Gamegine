"""Camera mounting configuration for visibility analysis.

This module defines how cameras are mounted on robots for AprilTag detection.
The camera hardware specs are in gamegine.reference.cameras.
"""

from dataclasses import dataclass, field
from typing import List, Optional

from gamegine.representation.bounds import Point, Boundary3D
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Degree
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Inch, Feet, Meter


@dataclass
class CameraMount:
    """A camera mounted at a specific pose relative to robot center.
    
    :param camera: Reference to the camera hardware specification.
    :type camera: :class:`CameraSpecification`
    :param offset: XYZ offset from robot center (x=forward, y=left, z=up).
    :type offset: :class:`Point`
    :param heading: Yaw angle relative to robot heading (0=forward, +CCW).
    :type heading: :class:`AngularMeasurement`
    :param pitch: Tilt angle from horizontal (positive=up).
    :type pitch: :class:`AngularMeasurement`
    """

    # Note: CameraSpecification imported lazily to avoid circular imports
    camera: "CameraSpecification"  # type: ignore
    offset: Point
    heading: AngularMeasurement = field(default_factory=lambda: Degree(0))
    pitch: AngularMeasurement = field(default_factory=lambda: Degree(0))

    def get_world_position(
        self,
        robot_x: SpatialMeasurement,
        robot_y: SpatialMeasurement,
        robot_heading: AngularMeasurement,
    ) -> Point:
        """Calculate the camera's world position given robot pose.
        
        :param robot_x: Robot X position in world coordinates.
        :param robot_y: Robot Y position in world coordinates.
        :param robot_heading: Robot heading in world coordinates.
        :return: Camera position in world coordinates.
        """
        import math
        
        heading_rad = float(robot_heading.to(Degree)) * math.pi / 180
        offset_x = float(self.offset.x.to(Meter))
        offset_y = float(self.offset.y.to(Meter))
        
        # Rotate offset by robot heading
        world_offset_x = offset_x * math.cos(heading_rad) - offset_y * math.sin(heading_rad)
        world_offset_y = offset_x * math.sin(heading_rad) + offset_y * math.cos(heading_rad)
        
        return Point(
            Meter(float(robot_x.to(Meter)) + world_offset_x),
            Meter(float(robot_y.to(Meter)) + world_offset_y),
            self.offset.z,
        )

    def get_world_heading(self, robot_heading: AngularMeasurement) -> AngularMeasurement:
        """Calculate the camera's world heading given robot heading.
        
        :param robot_heading: Robot heading in world coordinates.
        :return: Camera heading in world coordinates.
        """
        return Degree(float(robot_heading.to(Degree)) + float(self.heading.to(Degree)))


@dataclass
class MountConstraints:
    """3D bounds on robot where cameras can be mounted.
    
    :param valid_regions: List of 3D boundaries defining permitted mounting volumes.
    :type valid_regions: List[:class:`Boundary3D`]
    :param min_height: Minimum mounting height from ground.
    :type min_height: :class:`SpatialMeasurement`
    :param max_height: Maximum mounting height from ground.
    :type max_height: :class:`SpatialMeasurement`
    """

    valid_regions: List[Boundary3D] = field(default_factory=list)
    min_height: SpatialMeasurement = field(default_factory=lambda: Inch(6))
    max_height: SpatialMeasurement = field(default_factory=lambda: Feet(4))

    def is_valid_mount(self, offset: Point) -> bool:
        """Check if a mount position is within constraints.
        
        :param offset: Proposed mount offset from robot center.
        :return: True if the position is valid for mounting.
        """
        z = float(offset.z.to(Meter))
        if z < float(self.min_height.to(Meter)) or z > float(self.max_height.to(Meter)):
            return False
        
        # If no regions specified, allow anywhere within height bounds
        if not self.valid_regions:
            return True
        
        # Check if point is within any valid region
        for region in self.valid_regions:
            # Project to 2D and check containment
            slice_2d = region.get_slice(offset.z)
            if slice_2d and slice_2d.contains_point(offset.x, offset.y):
                return True
        
        return False
