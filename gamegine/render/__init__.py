"""
Gamegine Render package.

Just add objects. The renderer handles everything.
"""

from gamegine.render.renderer import (
    Renderer,
    ObjectRendererRegistry,
    DisplayLevel,
    AlertType,
    run,
)
from gamegine.render.canvas import Canvas, CanvasSettings, get_canvas, set_canvas
from gamegine.render.style import (
    Theme,
    DarkTheme,
    Palette,
    Opacity,
    get_theme,
    set_theme,
    ROYAL_BLUE,
)

# Import handlers to auto-register them
from gamegine.render import handlers

__all__ = [
    # Core
    "Renderer",
    "ObjectRendererRegistry",
    "DisplayLevel",
    "AlertType",
    "run",
    # Canvas
    "Canvas",
    "CanvasSettings",
    "get_canvas",
    "set_canvas",
    # Styles
    "Theme",
    "DarkTheme",
    "Palette",
    "Opacity",
    "get_theme",
    "set_theme",
    "ROYAL_BLUE",
]
