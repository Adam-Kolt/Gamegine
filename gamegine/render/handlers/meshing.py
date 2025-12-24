"""
Meshing rendering handlers for Gamegine.

Handles rendering of navigation meshes (TriangulatedGraph).
"""

from typing import Any

import arcade

from gamegine.render.renderer import ObjectRendererRegistry, DisplayLevel
from gamegine.render.canvas import Canvas
from gamegine.render.style import Theme


# =============================================================================
# TriangulatedGraph Handler
# =============================================================================

try:
    from gamegine.analysis.meshing import TriangulatedGraph
    
    @ObjectRendererRegistry.register(TriangulatedGraph)
    def render_triangulated_graph(obj: Any, canvas: Canvas, theme: Theme, display_level: DisplayLevel):
        """Render a TriangulatedGraph (nav mesh)."""
        # Only show in debug mode
        if display_level != DisplayLevel.DEBUG:
            return
        
        # Draw edges
        for edge in obj.edges:
            p1 = edge.nodes[0]
            p2 = edge.nodes[1]
            arcade.draw_line(
                canvas.to_pixels(p1.x),
                canvas.to_pixels(p1.y),
                canvas.to_pixels(p2.x),
                canvas.to_pixels(p2.y),
                theme.mesh_edge,
                1,
            )
        
        # Draw nodes
        for node in obj.nodes:
            arcade.draw_circle_filled(
                canvas.to_pixels(node.x),
                canvas.to_pixels(node.y),
                2,
                theme.mesh_node,
            )
except ImportError:
    pass
