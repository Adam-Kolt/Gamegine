"""
Geometry rendering handlers for Gamegine.

Handles rendering of geometric shapes: Circle, Rectangle, Polygon, etc.
"""

from typing import Any

import arcade
from arcade.types import Color as ArcadeColor

from gamegine.render.renderer import ObjectRendererRegistry, DisplayLevel
from gamegine.render.canvas import Canvas
from gamegine.render.style import Theme


# =============================================================================
# Circle Handler
# =============================================================================

try:
    from gamegine.representation.bounds import Circle
    
    @ObjectRendererRegistry.register(Circle)
    def render_circle(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel, renderer=None):
        """Render a Circle as an obstacle."""
        arcade.draw_circle_filled(
            canvas.to_pixels(obj.x),
            canvas.to_pixels(obj.y),
            canvas.to_pixels(obj.radius),
            theme.obstacle_fill,
        )
        arcade.draw_circle_outline(
            canvas.to_pixels(obj.x),
            canvas.to_pixels(obj.y),
            canvas.to_pixels(obj.radius),
            theme.obstacle_outline,
            2,
        )
except ImportError:
    pass


# =============================================================================
# Rectangle Handler
# =============================================================================

try:
    from gamegine.representation.bounds import Rectangle
    
    @ObjectRendererRegistry.register(Rectangle)
    def render_rectangle(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel, renderer=None):
        """Render a Rectangle as an obstacle."""
        x = canvas.to_pixels(obj.x)
        y = canvas.to_pixels(obj.y)
        w = canvas.to_pixels(obj.width)
        h = canvas.to_pixels(obj.height)
        
        # Draw as polygon for consistency
        points = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        arcade.draw_polygon_filled(points, theme.obstacle_fill)
        arcade.draw_polygon_outline(points, theme.obstacle_outline, 2)
except ImportError:
    pass


# =============================================================================
# DiscreteBoundary Handler (Polygons)
# =============================================================================

try:
    from gamegine.representation.bounds import DiscreteBoundary
    
    @ObjectRendererRegistry.register(DiscreteBoundary)
    def render_discrete_boundary(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel, renderer=None):
        """Render a DiscreteBoundary (polygon)."""
        if not hasattr(obj, 'get_vertices'):
            return
        
        vertices = obj.get_vertices()
        if len(vertices) < 3:
            return
        
        pixel_points = [
            (canvas.to_pixels(p[0]), canvas.to_pixels(p[1]))
            for p in vertices
        ]
        
        # Determine if this is safety padding based on alpha in theme
        # For now, use obstacle colors
        arcade.draw_polygon_filled(pixel_points, theme.obstacle_fill)
        arcade.draw_polygon_outline(pixel_points, theme.obstacle_outline, 2)
except ImportError:
    pass


# =============================================================================
# BoundedObject Handler
# =============================================================================

try:
    from gamegine.representation.bounds import BoundedObject
    
    @ObjectRendererRegistry.register(BoundedObject)
    def render_bounded_object(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel, renderer=None):
        """Render a BoundedObject by rendering its bounds."""
        if not hasattr(obj, 'bounds'):
            return
        
        bounds = obj.bounds
        
        # Try to discretize for rendering
        if hasattr(bounds, 'discretized'):
            disc = bounds.discretized(8)
            if hasattr(disc, 'get_vertices'):
                vertices = disc.get_vertices()
                if len(vertices) >= 3:
                    pixel_points = [
                        (canvas.to_pixels(p[0]), canvas.to_pixels(p[1]))
                        for p in vertices
                    ]
                    arcade.draw_polygon_filled(pixel_points, theme.obstacle_fill)
                    arcade.draw_polygon_outline(pixel_points, theme.obstacle_outline, 2)
except ImportError:
    pass


# =============================================================================
# TraversalZone Handler
# =============================================================================

try:
    from gamegine.representation.zone import TraversalZone
    
    @ObjectRendererRegistry.register(TraversalZone)
    def render_traversal_zone(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel, renderer=None):
        """Render a TraversalZone with distinct styling."""
        if not hasattr(obj, 'bounds'):
            return
        
        bounds = obj.bounds
        
        # Try to discretize for rendering
        if hasattr(bounds, 'discretized'):
            disc = bounds.discretized(8)
            if hasattr(disc, 'get_vertices'):
                vertices = disc.get_vertices()
                if len(vertices) >= 3:
                    pixel_points = [
                        (canvas.to_pixels(p[0]), canvas.to_pixels(p[1]))
                        for p in vertices
                    ]
                    arcade.draw_polygon_filled(pixel_points, theme.zone_fill)
                    arcade.draw_polygon_outline(pixel_points, theme.zone_outline, 2)
except ImportError:
    pass


# =============================================================================
# Safety Padding (Expanded Bounds) - List Handler
# =============================================================================

def render_safety_padding(boundaries: list, canvas: Canvas, theme: Theme, display_level: DisplayLevel):
    """Render expanded obstacle bounds as semi-transparent solid fill."""
    for boundary in boundaries:
        if hasattr(boundary, 'get_vertices'):
            vertices = boundary.get_vertices()
            if len(vertices) >= 3:
                pixel_points = [
                    (canvas.to_pixels(p[0]), canvas.to_pixels(p[1]))
                    for p in vertices
                ]
                # Solid semi-transparent fill for safety padding
                arcade.draw_polygon_filled(pixel_points, theme.safety_padding)
