from typing import List, Tuple
from gamegine.representation.bounds import (
    Boundary,
    BoundedObject,
    Line,
    Polygon,
    Rectangle,
    Circle,
)
from gamegine.utils.NCIM.ncim import SpatialMeasurement


class Obstacle(BoundedObject):
    """Represents a physical obstacle on the field.

    :param name: The name of the obstacle.
    :type name: str
    :param bounds: The boundary of the obstacle.
    :type bounds: :class:`Boundary`"""

    def __init__(self, name: str, bounds: Boundary) -> None:
        super().__init__(bounds)
        self.name = name
        self.rendering_visibility = True
        pass

    def __str__(self):
        return f"{self.name} Obstacle with bounds: {self.bounds}"

    def invisible(self) -> "Obstacle":
        """Sets the rendering visibility of the obstacle to False.

        :return: The obstacle object with rendering visibility set to False.
        :rtype: :class:`Obstacle`"""
        self.rendering_visibility = False
        return self

    def isVisible(self) -> bool:
        """Returns whether the obstacle is visible or not.

        :return: True if the obstacle is visible, False otherwise.
        :rtype: bool"""
        return self.rendering_visibility

    def set_rendering_visibility(self, visibility: bool) -> "Obstacle":
        """Sets the rendering visibility of the obstacle.

        :param visibility: The rendering visibility of the obstacle.
        :type visibility: bool
        :return: The obstacle object with the rendering visibility set.
        :rtype: :class:`Obstacle`"""
        self.rendering_visibility = visibility
        return self


class Rectangular(Obstacle):
    """Represents a rectangular obstacle on the field.

    :param name: The name of the obstacle.
    :type name: str
    :param x: The x-coordinate of the obstacle.
    :type x: :class:`SpatialMeasurement`
    :param y: The y-coordinate of the obstacle.
    :type y: :class:`SpatialMeasurement`
    :param width: The width of the obstacle.
    :type width: :class:`SpatialMeasurement`
    :param height: The height of the obstacle.
    :type height: :class:`SpatialMeasurement`"""

    def __init__(
        self,
        name: str,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        width: SpatialMeasurement,
        height: SpatialMeasurement,
    ) -> None:
        super().__init__(name, Rectangle(x, y, width, height))


class Circular(Obstacle):
    """Represents a circular obstacle on the field.

    :param name: The name of the obstacle.
    :type name: str
    :param x: The x-coordinate of the obstacle.
    :type x: :class:`SpatialMeasurement`
    :param y: The y-coordinate of the obstacle.
    :type y: :class:`SpatialMeasurement`
    :param radius: The radius of the obstacle.
    :type radius: :class:`SpatialMeasurement`
    """

    def __init__(
        self,
        name: str,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        radius: SpatialMeasurement,
    ) -> None:
        super().__init__(name, Circle(x, y, radius))


class Polygonal(Obstacle):
    """Represents a polygonal obstacle on the field.

    :param name: The name of the obstacle.
    :type name: str
    :param points: The points that define the polygon.
    :type points: List[Tuple[SpatialMeasurement, SpatialMeasurement]]"""

    def __init__(
        self, name: str, points: List[Tuple[SpatialMeasurement, SpatialMeasurement]]
    ) -> None:
        super().__init__(name, Polygon(points))


class Obstacle3D(Obstacle):
    """Represents an obstacle with 3D extent for height-based filtering.
    
    Useful for obstacles like bars that only affect robots above a certain height.
    
    :param name: The name of the obstacle.
    :type name: str
    :param bounds_2d: The 2D boundary for collision detection.
    :type bounds_2d: :class:`Boundary`
    :param z_min: The minimum height (clearance) of the obstacle.
    :type z_min: :class:`SpatialMeasurement`
    :param z_max: The maximum height of the obstacle.
    :type z_max: :class:`SpatialMeasurement`
    """

    def __init__(
        self,
        name: str,
        bounds_2d: Boundary,
        z_min: SpatialMeasurement,
        z_max: SpatialMeasurement,
    ) -> None:
        super().__init__(name, bounds_2d)
        self.z_min = z_min
        self.z_max = z_max
    
    def get_z_interval(self) -> Tuple[SpatialMeasurement, SpatialMeasurement]:
        """Returns the height interval of this obstacle.
        
        :return: Tuple of (z_min, z_max).
        """
        return (self.z_min, self.z_max)
    
    def applies_to_robot(self, robot_height: SpatialMeasurement) -> bool:
        """Check if this obstacle applies to a robot of the given height.
        
        A robot is blocked if its height exceeds the obstacle's clearance (z_min).
        For example, a bar at z_min=3ft allows robots under 3ft to pass freely.
        
        :param robot_height: The maximum height of the robot.
        :returns: True if the robot is too tall to pass, False otherwise.
        """
        return robot_height > self.z_min

