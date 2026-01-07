"""
Robot rendering handlers for Gamegine.

Handles rendering of RobotState, SwerveRobot, and trajectory states.
"""

import math
from typing import Any

import arcade
from arcade.types import Color as ArcadeColor

from gamegine.render.renderer import ObjectRendererRegistry, DisplayLevel
from gamegine.render.canvas import Canvas
from gamegine.render.style import Theme


# =============================================================================
# RobotState Handler
# =============================================================================

try:
    from gamegine.simulation.robot import RobotState
    from gamegine.utils.NCIM.Dimensions.spatial import Meter
    from gamegine.utils.NCIM.Dimensions.angular import Radian
    
    @ObjectRendererRegistry.register(RobotState)
    def render_robot_state(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel, renderer=None):
        """Render a RobotState as a robot on the field."""
        # Get position
        x = canvas.to_pixels(obj.x.get())
        y = canvas.to_pixels(obj.y.get())
        heading = obj.heading.get().to(Radian)
        
        # Robot size (approximate 30 inch square robot)
        from gamegine.utils.NCIM.Dimensions.spatial import Inch
        robot_size = canvas.to_pixels(Inch(30))
        half_size = robot_size / 2
        
        # Create rotated rectangle corners
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        
        corners = [
            (-half_size, -half_size),
            (half_size, -half_size),
            (half_size, half_size),
            (-half_size, half_size),
        ]
        
        rotated = [
            (x + c[0] * cos_h - c[1] * sin_h, y + c[0] * sin_h + c[1] * cos_h)
            for c in corners
        ]
        
        # Draw robot body
        arcade.draw_polygon_filled(rotated, theme.robot_fill)
        arcade.draw_polygon_outline(rotated, theme.robot_outline, 2)
        
        # Draw heading indicator (arrow pointing forward)
        arrow_len = half_size * 0.8
        end_x = x + cos_h * arrow_len
        end_y = y + sin_h * arrow_len
        arcade.draw_line(x, y, end_x, end_y, theme.robot_heading, 3)
        
        # Draw small arrowhead
        arrow_head_len = half_size * 0.3
        angle1 = heading + math.pi * 0.8
        angle2 = heading - math.pi * 0.8
        arcade.draw_line(
            end_x, end_y,
            end_x + math.cos(angle1) * arrow_head_len,
            end_y + math.sin(angle1) * arrow_head_len,
            theme.robot_heading, 3
        )
        arcade.draw_line(
            end_x, end_y,
            end_x + math.cos(angle2) * arrow_head_len,
            end_y + math.sin(angle2) * arrow_head_len,
            theme.robot_heading, 3
        )
        
except ImportError:
    pass


# =============================================================================
# SwerveTrajectoryState Handler
# =============================================================================

try:
    from gamegine.analysis.trajectory.lib.trajectoryStates import SwerveTrajectoryState
    from gamegine.utils.NCIM.Dimensions.spatial import Meter, Inch
    from gamegine.utils.NCIM.Dimensions.angular import Radian
    
    @ObjectRendererRegistry.register(SwerveTrajectoryState)
    def render_trajectory_state(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel, renderer=None):
        """Render a SwerveTrajectoryState as robot position along trajectory."""
        # Get position from trajectory state
        x = canvas.to_pixels(obj.x)
        y = canvas.to_pixels(obj.y)
        
        # Get world coordinates for hit test
        wx = obj.x.to(Meter) if hasattr(obj.x, 'to') else float(obj.x)
        wy = obj.y.to(Meter) if hasattr(obj.y, 'to') else float(obj.y)
        
        # Get heading (theta)
        if hasattr(obj, 'theta'):
            heading = obj.theta.to(Radian) if hasattr(obj.theta, 'to') else float(obj.theta)
        else:
            heading = 0
        
        # Robot size (18 pixels half-size like spline_traj_playground)
        ROBOT_HALF_SIZE = 18
        ROBOT_WORLD_SIZE = 0.4  # 40cm half-size in world coords
        
        # Register as selectable with hit test
        if renderer is not None:
            def hit_test(test_x, test_y):
                """Check if point is within robot bounds."""
                return math.sqrt((test_x - wx)**2 + (test_y - wy)**2) < ROBOT_WORLD_SIZE
            renderer.register_selectable(obj, hit_test)
        
        # Check hover/selection state
        is_hovered = renderer is not None and renderer.hovered_object is obj
        is_selected = renderer is not None and renderer.selected_object is obj
        
        # Adjust colors for hover/selection
        fill_color = theme.robot_fill
        outline_color = theme.robot_outline
        outline_width = 2
        
        if is_selected:
            fill_color = ArcadeColor(65, 105, 225, 180)  # Royal blue
            outline_width = 4
        elif is_hovered:
            fill_color = ArcadeColor(100, 149, 237, 180)  # Cornflower blue
            outline_width = 3
        
        # Create rotated rectangle corners
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        
        corners = []
        for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
            rx = x + (dx * cos_h - dy * sin_h) * ROBOT_HALF_SIZE
            ry = y + (dx * sin_h + dy * cos_h) * ROBOT_HALF_SIZE
            corners.append((rx, ry))
        
        # Draw solid robot body
        arcade.draw_polygon_filled(corners, fill_color)
        arcade.draw_polygon_outline(corners, outline_color, outline_width)
        
        # Draw extended heading arrow (1.3x half_size like spline_traj_playground)
        arrow_len = ROBOT_HALF_SIZE * 1.3
        end_x = x + cos_h * arrow_len
        end_y = y + sin_h * arrow_len
        arcade.draw_line(x, y, end_x, end_y, outline_color, 3)

except ImportError:
    pass


# =============================================================================
# TrajectoryState Handler (base class fallback)
# =============================================================================

try:
    from gamegine.analysis.trajectory.lib.trajectoryStates import TrajectoryState
    from gamegine.utils.NCIM.Dimensions.spatial import Meter, Inch
    from gamegine.utils.NCIM.Dimensions.angular import Radian
    
    @ObjectRendererRegistry.register(TrajectoryState)
    def render_base_trajectory_state(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel, renderer=None):
        """Render a base TrajectoryState as robot position along trajectory."""
        # Get position from trajectory state
        x = canvas.to_pixels(obj.x)
        y = canvas.to_pixels(obj.y)
        
        # Get heading (theta)
        if hasattr(obj, 'theta'):
            heading = obj.theta.to(Radian) if hasattr(obj.theta, 'to') else float(obj.theta)
        else:
            heading = 0
        
        # Robot size (18 pixels half-size like spline_traj_playground)
        ROBOT_HALF_SIZE = 18
        
        # Create rotated rectangle corners
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        
        corners = []
        for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
            rx = x + (dx * cos_h - dy * sin_h) * ROBOT_HALF_SIZE
            ry = y + (dx * sin_h + dy * cos_h) * ROBOT_HALF_SIZE
            corners.append((rx, ry))
        
        # Draw solid robot body
        arcade.draw_polygon_filled(corners, theme.robot_fill)
        arcade.draw_polygon_outline(corners, theme.robot_outline, 2)
        
        # Draw heading arrow
        arrow_len = ROBOT_HALF_SIZE * 1.3
        end_x = x + cos_h * arrow_len
        end_y = y + sin_h * arrow_len
        arcade.draw_line(x, y, end_x, end_y, theme.robot_outline, 3)

except ImportError:
    pass
