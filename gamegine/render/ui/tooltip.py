"""
Tooltip UI component.

Provides hover tooltips for rendered objects.
"""

from dataclasses import dataclass
from typing import Any, Optional, Tuple, Callable
import math

import arcade
from arcade.types import Color as ArcadeColor

from gamegine.render.style import Theme, get_theme, Palette


@dataclass
class TooltipConfig:
    """Configuration for tooltip appearance."""
    padding: int = 8
    margin: int = 10
    font_size: int = 12
    max_width: int = 250
    corner_radius: int = 4
    line_height: float = 1.4


class Tooltip:
    """
    A tooltip that displays information when hovering over objects.
    """
    
    def __init__(self, config: Optional[TooltipConfig] = None):
        self._config = config or TooltipConfig()
        self._visible = False
        self._text: str = ""
        self._x: float = 0
        self._y: float = 0
        self._target_obj: Any = None
    
    @property
    def visible(self) -> bool:
        return self._visible
    
    def show(self, text: str, x: float, y: float, target: Any = None):
        """Show the tooltip at the given position."""
        self._visible = True
        self._text = text
        self._x = x
        self._y = y
        self._target_obj = target
    
    def hide(self):
        """Hide the tooltip."""
        self._visible = False
        self._target_obj = None
    
    def draw(self, theme: Optional[Theme] = None):
        """Draw the tooltip."""
        if not self._visible or not self._text:
            return
        
        theme = theme or get_theme()
        config = self._config
        
        # Calculate text dimensions
        lines = self._text.split('\n')
        line_height = config.font_size * config.line_height
        text_height = len(lines) * line_height
        text_width = max(len(line) * config.font_size * 0.6 for line in lines)
        text_width = min(text_width, config.max_width)
        
        # Background dimensions
        bg_width = text_width + config.padding * 2
        bg_height = text_height + config.padding * 2
        
        # Position adjustment (keep on screen)
        x = self._x + config.margin
        y = self._y + config.margin
        
        # Draw background
        arcade.draw_lrbt_rectangle_filled(
            left=x,
            right=x + bg_width,
            bottom=y,
            top=y + bg_height,
            color=theme.tooltip_background,
        )
        
        # Draw border
        arcade.draw_lrbt_rectangle_outline(
            left=x,
            right=x + bg_width,
            bottom=y,
            top=y + bg_height,
            color=theme.tooltip_border,
            border_width=1,
        )
        
        # Draw text
        text_x = x + config.padding
        text_y = y + bg_height - config.padding - config.font_size
        
        for line in lines:
            arcade.draw_text(
                line,
                text_x,
                text_y,
                theme.tooltip_text,
                config.font_size,
                anchor_x="left",
                anchor_y="top",
            )
            text_y -= line_height


class TooltipManager:
    """
    Manages tooltips for multiple objects.
    
    Provides automatic tooltip display based on hover detection.
    """
    
    def __init__(self):
        self._tooltip = Tooltip()
        self._info_providers: dict = {}  # type -> Callable[[Any], str]
        self._hover_delay: float = 0.3  # seconds before showing tooltip
        self._hover_time: float = 0.0
        self._last_hover_obj: Any = None
    
    def register_info_provider(self, obj_type: type, provider: Callable[[Any], str]):
        """
        Register a function that provides tooltip text for a given object type.
        
        Args:
            obj_type: The type of object
            provider: A function that takes an object and returns tooltip text
        """
        self._info_providers[obj_type] = provider
    
    def get_info(self, obj: Any) -> Optional[str]:
        """Get tooltip info for an object."""
        obj_type = type(obj)
        
        # Check exact type
        if obj_type in self._info_providers:
            return self._info_providers[obj_type](obj)
        
        # Check base classes
        for base in obj_type.__mro__:
            if base in self._info_providers:
                return self._info_providers[base](obj)
        
        # Default: try __str__ or repr
        if hasattr(obj, '__str__'):
            return str(obj)
        
        return None
    
    def update(self, delta_time: float, hovered_obj: Any, mouse_x: float, mouse_y: float):
        """Update tooltip state based on hover."""
        if hovered_obj is None:
            self._tooltip.hide()
            self._hover_time = 0.0
            self._last_hover_obj = None
            return
        
        if hovered_obj != self._last_hover_obj:
            self._hover_time = 0.0
            self._last_hover_obj = hovered_obj
            self._tooltip.hide()
        else:
            self._hover_time += delta_time
        
        if self._hover_time >= self._hover_delay:
            info = self.get_info(hovered_obj)
            if info:
                self._tooltip.show(info, mouse_x, mouse_y, hovered_obj)
    
    def draw(self, theme: Optional[Theme] = None):
        """Draw the active tooltip."""
        self._tooltip.draw(theme)


# Global tooltip manager
_tooltip_manager: Optional[TooltipManager] = None


def get_tooltip_manager() -> TooltipManager:
    """Get the global tooltip manager."""
    global _tooltip_manager
    if _tooltip_manager is None:
        _tooltip_manager = TooltipManager()
    return _tooltip_manager
