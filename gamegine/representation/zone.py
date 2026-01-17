"""Field zone definitions for traversal modifiers and special regions."""

from typing import Optional

from gamegine.representation.bounds import BoundedObject, Boundary, DiscreteBoundary
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter


class TraversalZone(BoundedObject):
    """Represents a field zone that modifies robot traversal behavior.
    
    Zones affect both pathfinding (edge weight scaling) and trajectory 
    generation (velocity limit scaling).
    
    By extending BoundedObject, zones can be mirrored, translated, and scaled
    using the same methods as obstacles.
    
    :param name: Unique identifier for this zone.
    :param boundary: The 2D boundary defining the zone's extent.
    :param speed_multiplier: Velocity scaling factor within the zone.
        - 1.0 = normal speed
        - 0.5 = half max speed (e.g., ramp slowdown)
        - 2.0 = double speed (theoretical)
    """
    
    def __init__(self, name: str, boundary: Boundary, speed_multiplier: float = 1.0):
        super().__init__(boundary, name)
        self.speed_multiplier = speed_multiplier
    
    @property
    def boundary(self) -> Boundary:
        """Alias for bounds to maintain backward compatibility."""
        return self.bounds
    
    def contains_point(self, x: SpatialMeasurement, y: SpatialMeasurement) -> bool:
        """Check if a point is inside this zone.
        
        :param x: X coordinate to check.
        :param y: Y coordinate to check.
        :returns: True if point is inside zone boundary.
        """
        discrete = self.bounds.discretized()
        return discrete.contains_point(x, y)
    
    def get_weight_multiplier(self) -> float:
        """Returns the pathfinding edge weight multiplier.
        
        Slower zones have higher weights to discourage routing through them
        unless necessary.
        
        :returns: Weight multiplier (inverse of speed_multiplier).
        """
        if self.speed_multiplier <= 0:
            return float('inf')
        return 1.0 / self.speed_multiplier
