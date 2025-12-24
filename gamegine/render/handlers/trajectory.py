"""
Trajectory rendering handlers for Gamegine.

Handles rendering of trajectories with display level support.
"""

from typing import Any
import math

import arcade
from arcade.types import Color as ArcadeColor

from gamegine.render.renderer import ObjectRendererRegistry, DisplayLevel
from gamegine.render.canvas import Canvas
from gamegine.render.style import Theme


# =============================================================================
# Trajectory Handler
# =============================================================================

try:
    from gamegine.analysis.trajectory.lib.TrajGen import Trajectory
    
    @ObjectRendererRegistry.register(Trajectory)
    def render_trajectory(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel, renderer=None):
        """Render a Trajectory."""
        if not hasattr(obj, 'points') or len(obj.points) < 2:
            return
        
        points = obj.points
        
        # Register as selectable with hit test
        if renderer is not None:
            def hit_test(wx, wy):
                """Check if point is near the trajectory."""
                for i in range(0, len(points), max(1, len(points) // 20)):
                    pt = points[i]
                    px = pt.x.to(Meter) if hasattr(pt.x, 'to') else float(pt.x)
                    py = pt.y.to(Meter) if hasattr(pt.y, 'to') else float(pt.y)
                    if math.sqrt((wx - px)**2 + (wy - py)**2) < 0.3:  # 30cm hit radius
                        return True
                return False
            renderer.register_selectable(obj, hit_test)
        
        # Check hover/selection state
        is_hovered = renderer is not None and renderer.hovered_object is obj
        is_selected = renderer is not None and renderer.selected_object is obj
        
        # Determine line color and width
        line_color = theme.trajectory_line
        line_width = 3
        
        if is_selected:
            line_color = ArcadeColor(65, 105, 225, 255)  # Royal blue
            line_width = 5
        elif is_hovered:
            line_color = ArcadeColor(100, 149, 237, 255)  # Cornflower blue (lighter)
            line_width = 4

        # Draw trajectory line
        for i in range(len(points) - 1):
            p1, p2 = points[i], points[i + 1]
            arcade.draw_line(
                canvas.to_pixels(p1.x),
                canvas.to_pixels(p1.y),
                canvas.to_pixels(p2.x),
                canvas.to_pixels(p2.y),
                line_color,
                line_width,
            )
        
        # In debug mode, show waypoints
        if display_level == DisplayLevel.DEBUG:
            for pt in points:
                arcade.draw_circle_filled(
                    canvas.to_pixels(pt.x),
                    canvas.to_pixels(pt.y),
                    4,
                    theme.trajectory_point,
                )
except ImportError:
    pass


# =============================================================================
# SwerveTrajectory Handler
# =============================================================================

try:
    from gamegine.analysis.trajectory.lib.TrajGen import SwerveTrajectory
    from gamegine.utils.NCIM.Dimensions.angular import Radian
    from gamegine.utils.NCIM.Dimensions.spatial import Meter
    
    @ObjectRendererRegistry.register(SwerveTrajectory)
    def render_swerve_trajectory(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel, renderer=None):
        """Render a SwerveTrajectory with optional debug info."""
        if not hasattr(obj, 'points') or len(obj.points) < 2:
            return
        
        points = obj.points
        
        # Register as selectable with hit test
        if renderer is not None:
            def hit_test(wx, wy):
                """Check if point is near the trajectory."""
                for i in range(0, len(points), max(1, len(points) // 20)):
                    pt = points[i]
                    px = pt.x.to(Meter) if hasattr(pt.x, 'to') else float(pt.x)
                    py = pt.y.to(Meter) if hasattr(pt.y, 'to') else float(pt.y)
                    if math.sqrt((wx - px)**2 + (wy - py)**2) < 0.3:  # 30cm hit radius
                        return True
                return False
            renderer.register_selectable(obj, hit_test)
        
        # Check hover/selection state
        is_hovered = renderer is not None and renderer.hovered_object is obj
        is_selected = renderer is not None and renderer.selected_object is obj
        
        # Determine line color and width
        line_color = theme.trajectory_line
        line_width = 3
        
        if is_selected:
            line_color = ArcadeColor(65, 105, 225, 255)  # Royal blue
            line_width = 5
        elif is_hovered:
            line_color = ArcadeColor(100, 149, 237, 255)  # Cornflower blue (lighter)
            line_width = 4
        
        # Draw trajectory line
        for i in range(len(points) - 1):
            p1, p2 = points[i], points[i + 1]
            arcade.draw_line(
                canvas.to_pixels(p1.x),
                canvas.to_pixels(p1.y),
                canvas.to_pixels(p2.x),
                canvas.to_pixels(p2.y),
                line_color,
                line_width,
            )
        
        # Selection indicator - glow effect at start
        if is_selected:
            start = points[0]
            sx = canvas.to_pixels(start.x)
            sy = canvas.to_pixels(start.y)
            arcade.draw_circle_filled(sx, sy, 15, ArcadeColor(65, 105, 225, 60))
            arcade.draw_circle_outline(sx, sy, 12, ArcadeColor(65, 105, 225, 200), 2)
        
        # Debug mode: show swerve module positions
        if display_level == DisplayLevel.DEBUG:
            _draw_swerve_debug(obj, canvas, theme)

    def _draw_swerve_debug(obj: Any, canvas: Canvas, theme: Theme):
        """Draw debug visualization for swerve trajectory."""
        # Sample every N points to show waypoints
        step = max(1, len(obj.points) // 15)
        
        for i in range(0, len(obj.points), step):
            pt = obj.points[i]
            px = canvas.to_pixels(pt.x)
            py = canvas.to_pixels(pt.y)
            
            # Draw waypoint marker
            arcade.draw_circle_filled(px, py, 4, theme.trajectory_point)

except ImportError:
    pass


# =============================================================================
# Path Handler
# =============================================================================

try:
    from gamegine.analysis.pathfinding import Path
    
    @ObjectRendererRegistry.register(Path)
    def render_path(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel, renderer=None):
        """Render an A* path."""
        points = obj.get_points() if hasattr(obj, 'get_points') else obj.path
        
        if len(points) < 2:
            return
        
        for i in range(len(points) - 1):
            p1, p2 = points[i], points[i + 1]
            arcade.draw_line(
                canvas.to_pixels(p1[0]),
                canvas.to_pixels(p1[1]),
                canvas.to_pixels(p2[0]),
                canvas.to_pixels(p2[1]),
                theme.path_line,
                2,
            )
except ImportError:
    pass
