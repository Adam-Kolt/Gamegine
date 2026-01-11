"""Field zone definitions for traversal modifiers and special regions."""

from dataclasses import dataclass
from typing import Optional

from gamegine.representation.bounds import Boundary, DiscreteBoundary
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Meter


@dataclass
class TraversalZone:
    """Represents a field zone that modifies robot traversal behavior.
    
    Zones affect both pathfinding (edge weight scaling) and trajectory 
    generation (velocity limit scaling).
    
    :param name: Unique identifier for this zone.
    :param boundary: The 2D boundary defining the zone's extent.
    :param speed_multiplier: Velocity scaling factor within the zone.
        - 1.0 = normal speed
        - 0.5 = half max speed (e.g., ramp slowdown)
        - 2.0 = double speed (theoretical)
    """
    name: str
    boundary: Boundary
    speed_multiplier: float = 1.0
    
    def contains_point(self, x: SpatialMeasurement, y: SpatialMeasurement) -> bool:
        """Check if a point is inside this zone.
        
        :param x: X coordinate to check.
        :param y: Y coordinate to check.
        :returns: True if point is inside zone boundary.
        """
        discrete = self.boundary.discretized()
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
