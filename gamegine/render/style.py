"""
Style module for Gamegine rendering.

Provides theming, color palettes, and styling utilities for consistent
and aesthetically pleasing visualizations.

Brand identity: White background, Royal Blue (#4169E1) primary.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Tuple, Dict

import arcade
from arcade.types import Color as ArcadeColor


# =============================================================================
# Gamegine Brand Colors
# =============================================================================

# Royal Blue - Primary brand color
ROYAL_BLUE = ArcadeColor(65, 105, 225, 255)       # #4169E1
ROYAL_BLUE_LIGHT = ArcadeColor(135, 165, 245, 255)
ROYAL_BLUE_DARK = ArcadeColor(45, 75, 175, 255)
ROYAL_BLUE_FAINT = ArcadeColor(65, 105, 225, 40)

# Whites and grays
WHITE = ArcadeColor(255, 255, 255, 255)
OFF_WHITE = ArcadeColor(250, 250, 252, 255)
GRAY_FAINT = ArcadeColor(240, 240, 244, 255)
GRAY_LIGHT = ArcadeColor(200, 200, 210, 255)
GRAY = ArcadeColor(130, 135, 145, 255)
GRAY_DARK = ArcadeColor(70, 75, 85, 255)
BLACK = ArcadeColor(20, 20, 25, 255)


# =============================================================================
# Color Palettes
# =============================================================================

class Palette(Enum):
    """Modern color palette for rendering."""
    
    # Brand
    ROYAL_BLUE = (65, 105, 225, 255)
    
    # Primary colors
    WHITE = (255, 255, 255, 255)
    BLACK = (20, 20, 25, 255)
    
    # Blues
    BLUE = (59, 130, 246, 255)
    BLUE_LIGHT = (191, 219, 254, 255)
    BLUE_DARK = (29, 78, 216, 255)
    
    # Greens
    GREEN = (34, 197, 94, 255)
    GREEN_LIGHT = (187, 247, 208, 255)
    GREEN_DARK = (21, 128, 61, 255)
    
    # Reds
    RED = (239, 68, 68, 255)
    RED_LIGHT = (254, 202, 202, 255)
    RED_DARK = (185, 28, 28, 255)
    
    # Yellows
    YELLOW = (234, 179, 8, 255)
    YELLOW_LIGHT = (254, 240, 138, 255)
    YELLOW_DARK = (161, 98, 7, 255)
    
    # Oranges
    ORANGE = (249, 115, 22, 255)
    ORANGE_LIGHT = (254, 215, 170, 255)
    ORANGE_DARK = (194, 65, 12, 255)
    
    # Grays
    GRAY = (107, 114, 128, 255)
    GRAY_LIGHT = (229, 231, 235, 255)
    GRAY_DARK = (55, 65, 81, 255)
    
    # Cyans
    CYAN = (6, 182, 212, 255)
    CYAN_LIGHT = (165, 243, 252, 255)
    CYAN_DARK = (14, 116, 144, 255)
    
    # Purples
    PURPLE = (168, 85, 247, 255)
    PURPLE_LIGHT = (233, 213, 255, 255)
    PURPLE_DARK = (126, 34, 206, 255)
    
    # Pinks
    PINK = (236, 72, 153, 255)
    PINK_LIGHT = (251, 207, 232, 255)
    PINK_DARK = (190, 24, 93, 255)
    
    def to_arcade(self) -> ArcadeColor:
        """Convert to arcade Color type."""
        return ArcadeColor(*self.value)
    
    def with_alpha(self, alpha: int) -> ArcadeColor:
        """Return the color with a modified alpha value (0-255)."""
        r, g, b, _ = self.value
        return ArcadeColor(r, g, b, alpha)


class Opacity(Enum):
    """Opacity levels."""
    OPAQUE = 255
    SEMI = 180
    TRANSLUCENT = 128
    FAINT = 64
    GHOST = 32


# =============================================================================
# Themes
# =============================================================================

@dataclass
class Theme:
    """Base theme class defining colors for different elements."""
    
    name: str = "Default"
    
    # Background
    background: ArcadeColor = field(default_factory=lambda: WHITE)
    
    # Grid
    grid_line: ArcadeColor = field(default_factory=lambda: GRAY_FAINT)
    
    # Field elements
    field_border: ArcadeColor = field(default_factory=lambda: GRAY_LIGHT)
    
    # Obstacles
    obstacle_fill: ArcadeColor = field(default_factory=lambda: GRAY_DARK)
    obstacle_outline: ArcadeColor = field(default_factory=lambda: GRAY)
    safety_padding: ArcadeColor = field(default_factory=lambda: ArcadeColor(255, 100, 100, 40))
    
    # Trajectories (Royal Blue!)
    trajectory_line: ArcadeColor = field(default_factory=lambda: ROYAL_BLUE)
    trajectory_point: ArcadeColor = field(default_factory=lambda: ROYAL_BLUE_DARK)
    path_line: ArcadeColor = field(default_factory=lambda: ROYAL_BLUE_FAINT)
    
    # Robot
    robot_fill: ArcadeColor = field(default_factory=lambda: ROYAL_BLUE)
    robot_outline: ArcadeColor = field(default_factory=lambda: ROYAL_BLUE_DARK)
    robot_heading: ArcadeColor = field(default_factory=lambda: BLACK)
    
    # Swerve modules (debug mode)
    module_colors: Tuple[ArcadeColor, ...] = field(default_factory=lambda: (
        ArcadeColor(220, 60, 60, 255),   # Red - Front Left
        ArcadeColor(255, 165, 0, 255),   # Orange - Front Right
        ArcadeColor(65, 105, 225, 255),  # Blue - Back Right
        ArcadeColor(255, 215, 0, 255),   # Yellow - Back Left
    ))
    
    # Ghost/Preview
    ghost_fill: ArcadeColor = field(default_factory=lambda: ArcadeColor(65, 105, 225, 60))
    ghost_outline: ArcadeColor = field(default_factory=lambda: ArcadeColor(65, 105, 225, 120))
    
    # UI
    ui_primary: ArcadeColor = field(default_factory=lambda: ROYAL_BLUE)
    ui_text: ArcadeColor = field(default_factory=lambda: GRAY_DARK)
    ui_text_light: ArcadeColor = field(default_factory=lambda: GRAY)
    tooltip_background: ArcadeColor = field(default_factory=lambda: ArcadeColor(255, 255, 255, 245))
    tooltip_text: ArcadeColor = field(default_factory=lambda: GRAY_DARK)
    tooltip_border: ArcadeColor = field(default_factory=lambda: ROYAL_BLUE_LIGHT)
    
    # Mesh/Graph
    mesh_edge: ArcadeColor = field(default_factory=lambda: ROYAL_BLUE_FAINT)
    mesh_node: ArcadeColor = field(default_factory=lambda: ROYAL_BLUE_LIGHT)
    
    # Hover highlight
    hover_highlight: ArcadeColor = field(default_factory=lambda: ArcadeColor(65, 105, 225, 80))


# The default Gamegine theme
GamegineTheme = Theme


@dataclass
class DarkTheme(Theme):
    """Dark theme with high contrast colors."""
    
    name: str = "Dark"
    background: ArcadeColor = field(default_factory=lambda: ArcadeColor(18, 18, 22, 255))
    grid_line: ArcadeColor = field(default_factory=lambda: ArcadeColor(35, 35, 45, 255))
    field_border: ArcadeColor = field(default_factory=lambda: GRAY)
    
    obstacle_fill: ArcadeColor = field(default_factory=lambda: ArcadeColor(60, 60, 70, 255))
    obstacle_outline: ArcadeColor = field(default_factory=lambda: ArcadeColor(90, 90, 105, 255))
    
    trajectory_line: ArcadeColor = field(default_factory=lambda: ROYAL_BLUE_LIGHT)
    robot_fill: ArcadeColor = field(default_factory=lambda: ROYAL_BLUE_LIGHT)
    
    ui_text: ArcadeColor = field(default_factory=lambda: ArcadeColor(220, 220, 230, 255))
    tooltip_background: ArcadeColor = field(default_factory=lambda: ArcadeColor(30, 30, 40, 245))
    tooltip_text: ArcadeColor = field(default_factory=lambda: ArcadeColor(220, 220, 230, 255))


# =============================================================================
# Theme Management
# =============================================================================

_current_theme: Theme = Theme()


def get_theme() -> Theme:
    """Get the current active theme."""
    return _current_theme


def set_theme(theme: Theme):
    """Set the active theme."""
    global _current_theme
    _current_theme = theme


# =============================================================================
# Utility Functions
# =============================================================================

def lerp_color(color1: ArcadeColor, color2: ArcadeColor, t: float) -> ArcadeColor:
    """Linearly interpolate between two colors."""
    t = max(0.0, min(1.0, t))
    r = int(color1[0] + (color2[0] - color1[0]) * t)
    g = int(color1[1] + (color2[1] - color1[1]) * t)
    b = int(color1[2] + (color2[2] - color1[2]) * t)
    a1 = color1[3] if len(color1) > 3 else 255
    a2 = color2[3] if len(color2) > 3 else 255
    a = int(a1 + (a2 - a1) * t)
    return ArcadeColor(r, g, b, a)


def with_alpha(color: ArcadeColor, alpha: int) -> ArcadeColor:
    """Return a color with modified alpha."""
    return ArcadeColor(color[0], color[1], color[2], alpha)
