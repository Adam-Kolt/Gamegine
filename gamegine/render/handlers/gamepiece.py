"""
Gamepiece rendering handlers for Gamegine.

Renders gamepieces based on their state (on field vs held by robot).
"""

from typing import Any

import arcade

from gamegine.render.renderer import ObjectRendererRegistry, DisplayLevel
from gamegine.render.canvas import Canvas
from gamegine.render.style import Theme


# =============================================================================
# GamepieceState Handler - Renders pieces on field or on robot
# =============================================================================

try:
    from gamegine.simulation.gamepiece import GamepieceState
    from gamegine.utils.NCIM.Dimensions.spatial import Meter
    
    # Colors for gamepieces
    GAMEPIECE_ON_FIELD = (255, 200, 50, 255)  # Yellow/gold
    GAMEPIECE_HELD = (100, 200, 100, 255)      # Green when held
    GAMEPIECE_OUTLINE = (80, 80, 80, 255)      # Dark outline
    
    @ObjectRendererRegistry.register(GamepieceState)
    def render_gamepiece_state(
        obj: GamepieceState, 
        canvas: Canvas, 
        theme: Theme, 
        display_level: DisplayLevel, 
        renderer=None
    ):
        """Render a gamepiece based on its state.
        
        If on field: render at x, y position
        If held by robot: render atop the robot (requires renderer to look up robot position)
        """
        from gamegine.utils.NCIM.Dimensions.spatial import Inch
        
        # Gamepiece visual size
        radius = Inch(6)  # 6 inch radius for visualization
        
        if obj.is_on_field():
            # Render at field position
            x = canvas.to_pixels(obj.x.value)
            y = canvas.to_pixels(obj.y.value)
            color = GAMEPIECE_ON_FIELD
        else:
            # Piece is held - try to get robot position from renderer
            if renderer is not None and hasattr(renderer, 'get_robot_position'):
                robot_pos = renderer.get_robot_position(obj.owner)
                if robot_pos:
                    x = canvas.to_pixels(robot_pos[0])
                    y = canvas.to_pixels(robot_pos[1])
                    # Offset slightly to show stacked pieces
                    held_count = getattr(renderer, '_held_count', {})
                    offset = held_count.get(obj.owner, 0) * 8
                    held_count[obj.owner] = held_count.get(obj.owner, 0) + 1
                    renderer._held_count = held_count
                    y += offset
                    color = GAMEPIECE_HELD
                else:
                    return  # Can't render - robot not found
            else:
                return  # No renderer context
        
        # Draw gamepiece as filled circle with outline
        r_px = canvas.to_pixels(radius)
        arcade.draw_circle_filled(x, y, r_px, color)
        arcade.draw_circle_outline(x, y, r_px, GAMEPIECE_OUTLINE, 2)
        
        # Draw indicator dot in center
        arcade.draw_circle_filled(x, y, r_px * 0.3, GAMEPIECE_OUTLINE)

except ImportError:
    pass


# =============================================================================
# GamepieceManager Handler - Renders all managed pieces
# =============================================================================

try:
    from gamegine.simulation.gamepiece import GamepieceManager
    
    @ObjectRendererRegistry.register(GamepieceManager)
    def render_gamepiece_manager(
        obj: GamepieceManager, 
        canvas: Canvas, 
        theme: Theme, 
        display_level: DisplayLevel, 
        renderer=None
    ):
        """Render all gamepieces managed by this manager."""
        # Reset held count for proper stacking
        if renderer is not None:
            renderer._held_count = {}
        
        for piece_id, state in obj.pieces.items():
            render_gamepiece_state(state, canvas, theme, display_level, renderer)

except ImportError:
    pass
