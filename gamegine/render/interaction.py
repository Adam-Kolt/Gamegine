"""
Interaction handling for rendered objects.

Provides mouse picking, hover detection, and click handling.
"""

from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Set, Tuple, Type
import math

from gamegine.render.canvas import get_canvas


@dataclass
class HitTestResult:
    """Result of a hit test (mouse picking)."""
    obj: Any
    distance: float  # Distance from mouse to object center
    layer: str


class InteractionManager:
    """
    Manages interaction state for rendered objects.
    
    Provides hit testing, hover tracking, and click detection.
    """
    
    def __init__(self):
        # Hit test functions by type
        self._hit_testers: Dict[Type, Callable[[Any, float, float], bool]] = {}
        
        # State
        self._hovered_obj: Any = None
        self._selected_obj: Any = None
        self._click_callbacks: List[Callable[[Any], None]] = []
        self._hover_callbacks: List[Callable[[Any], None]] = []
    
    def register_hit_tester(self, obj_type: Type, tester: Callable[[Any, float, float], bool]):
        """
        Register a hit test function for a type.
        
        The tester should return True if the point (x, y) hits the object.
        Coordinates are in world units (meters).
        """
        self._hit_testers[obj_type] = tester
    
    def hit_test(self, obj: Any, x: float, y: float) -> bool:
        """
        Test if a point hits an object.
        
        Args:
            obj: The object to test
            x, y: Point in world coordinates (meters)
        
        Returns:
            True if the point hits the object
        """
        obj_type = type(obj)
        
        # Check exact type
        if obj_type in self._hit_testers:
            return self._hit_testers[obj_type](obj, x, y)
        
        # Check base classes
        for base in obj_type.__mro__:
            if base in self._hit_testers:
                return self._hit_testers[base](obj, x, y)
        
        return False
    
    def find_hit(
        self,
        objects: List[Tuple[Any, str]],  # (object, layer_name) pairs
        x: float,
        y: float,
    ) -> Optional[HitTestResult]:
        """
        Find the first object hit by a point.
        
        Args:
            objects: List of (object, layer_name) tuples
            x, y: Point in world coordinates
        
        Returns:
            HitTestResult if an object was hit, None otherwise
        """
        for obj, layer in objects:
            if self.hit_test(obj, x, y):
                # Calculate distance (for sorting if needed)
                dist = 0.0
                if hasattr(obj, 'x') and hasattr(obj, 'y'):
                    from gamegine.utils.NCIM.Dimensions.spatial import Meter
                    obj_x = obj.x.to(Meter) if hasattr(obj.x, 'to') else float(obj.x)
                    obj_y = obj.y.to(Meter) if hasattr(obj.y, 'to') else float(obj.y)
                    dist = math.sqrt((x - obj_x) ** 2 + (y - obj_y) ** 2)
                return HitTestResult(obj, dist, layer)
        
        return None
    
    @property
    def hovered_object(self) -> Any:
        """Get the currently hovered object."""
        return self._hovered_obj
    
    @property
    def selected_object(self) -> Any:
        """Get the currently selected object."""
        return self._selected_obj
    
    def set_hovered(self, obj: Any):
        """Set the hovered object."""
        if obj != self._hovered_obj:
            self._hovered_obj = obj
            for callback in self._hover_callbacks:
                callback(obj)
    
    def set_selected(self, obj: Any):
        """Set the selected object."""
        self._selected_obj = obj
    
    def on_hover(self, callback: Callable[[Any], None]):
        """Register a hover callback."""
        self._hover_callbacks.append(callback)
    
    def on_click(self, callback: Callable[[Any], None]):
        """Register a click callback."""
        self._click_callbacks.append(callback)
    
    def handle_click(self, obj: Any):
        """Handle a click on an object."""
        self._selected_obj = obj
        for callback in self._click_callbacks:
            callback(obj)


# =============================================================================
# Built-in Hit Testers
# =============================================================================

def circle_hit_test(obj: Any, x: float, y: float) -> bool:
    """Hit test for Circle objects."""
    from gamegine.utils.NCIM.Dimensions.spatial import Meter
    
    obj_x = obj.x.to(Meter) if hasattr(obj.x, 'to') else float(obj.x)
    obj_y = obj.y.to(Meter) if hasattr(obj.y, 'to') else float(obj.y)
    radius = obj.radius.to(Meter) if hasattr(obj.radius, 'to') else float(obj.radius)
    
    dist_sq = (x - obj_x) ** 2 + (y - obj_y) ** 2
    return dist_sq <= radius ** 2


def rectangle_hit_test(obj: Any, x: float, y: float) -> bool:
    """Hit test for Rectangle objects."""
    from gamegine.utils.NCIM.Dimensions.spatial import Meter
    
    obj_x = obj.x.to(Meter) if hasattr(obj.x, 'to') else float(obj.x)
    obj_y = obj.y.to(Meter) if hasattr(obj.y, 'to') else float(obj.y)
    width = obj.width.to(Meter) if hasattr(obj.width, 'to') else float(obj.width)
    height = obj.height.to(Meter) if hasattr(obj.height, 'to') else float(obj.height)
    
    return obj_x <= x <= obj_x + width and obj_y <= y <= obj_y + height


def polygon_hit_test(obj: Any, x: float, y: float) -> bool:
    """Hit test for Polygon/DiscreteBoundary objects using shapely."""
    if hasattr(obj, 'contains_point'):
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        return obj.contains_point(Meter(x), Meter(y))
    return False


def register_default_hit_testers(manager: InteractionManager):
    """Register default hit testers for common types."""
    try:
        from gamegine.representation.bounds import Circle, Rectangle, DiscreteBoundary
        
        manager.register_hit_tester(Circle, circle_hit_test)
        manager.register_hit_tester(Rectangle, rectangle_hit_test)
        manager.register_hit_tester(DiscreteBoundary, polygon_hit_test)
    except ImportError:
        pass


# =============================================================================
# Global Manager
# =============================================================================

_interaction_manager: Optional[InteractionManager] = None


def get_interaction_manager() -> InteractionManager:
    """Get the global interaction manager."""
    global _interaction_manager
    if _interaction_manager is None:
        _interaction_manager = InteractionManager()
        register_default_hit_testers(_interaction_manager)
    return _interaction_manager
