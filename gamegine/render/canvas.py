"""
Canvas module for Gamegine rendering.

This module provides a hardware-accelerated drawing abstraction layer using arcade.
All drawing primitives route through this module to enable consistent styling,
coordinate transforms, and potential future backend swaps.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional, TYPE_CHECKING
import math

import arcade
from arcade.types import Color as ArcadeColor

if TYPE_CHECKING:
    from gamegine.utils.NCIM.ncim import SpatialMeasurement


@dataclass
class CanvasSettings:
    """Global settings for canvas rendering."""
    render_scale: float = 100.0  # pixels per meter
    line_width: float = 2.0
    antialiasing: bool = True


class Canvas:
    """
    Hardware-accelerated drawing canvas using arcade.
    
    Provides drawing primitives that automatically handle coordinate conversion
    from game units (SpatialMeasurement) to screen pixels.
    """
    
    _settings: CanvasSettings = field(default_factory=CanvasSettings)
    
    def __init__(self, settings: Optional[CanvasSettings] = None):
        self._settings = settings or CanvasSettings()
        self._shape_list: Optional[arcade.ShapeElementList] = None
    
    @property
    def render_scale(self) -> float:
        return self._settings.render_scale
    
    @render_scale.setter
    def render_scale(self, value: float):
        self._settings.render_scale = value
    
    def to_pixels(self, value: "SpatialMeasurement") -> float:
        """Convert a SpatialMeasurement to pixels using the current render scale."""
        from gamegine.utils.NCIM.Dimensions.spatial import Meter
        return float(value.to(Meter)) * self._settings.render_scale
    
    def begin_shape_batch(self):
        """Begin a batched shape drawing session for better performance."""
        self._shape_list = arcade.ShapeElementList()
    
    def end_shape_batch(self):
        """End the batched shape drawing and render all shapes."""
        if self._shape_list:
            self._shape_list.draw()
            self._shape_list = None
    
    # -------------------------------------------------------------------------
    # Drawing Primitives
    # -------------------------------------------------------------------------
    
    def draw_circle(
        self,
        x: "SpatialMeasurement",
        y: "SpatialMeasurement",
        radius: "SpatialMeasurement",
        color: ArcadeColor,
        filled: bool = True,
        line_width: float = 2.0,
    ):
        """Draw a circle at the given position."""
        px = self.to_pixels(x)
        py = self.to_pixels(y)
        pr = self.to_pixels(radius)
        
        if filled:
            arcade.draw_circle_filled(px, py, pr, color)
        else:
            arcade.draw_circle_outline(px, py, pr, color, line_width)
    
    def draw_rectangle(
        self,
        x: "SpatialMeasurement",
        y: "SpatialMeasurement",
        width: "SpatialMeasurement",
        height: "SpatialMeasurement",
        color: ArcadeColor,
        filled: bool = True,
        line_width: float = 2.0,
        tilt_angle: float = 0.0,
    ):
        """Draw a rectangle. x,y is the center."""
        px = self.to_pixels(x)
        py = self.to_pixels(y)
        pw = self.to_pixels(width)
        ph = self.to_pixels(height)
        
        if filled:
            arcade.draw_rectangle_filled(px, py, pw, ph, color, tilt_angle)
        else:
            arcade.draw_rectangle_outline(px, py, pw, ph, color, line_width, tilt_angle)
    
    def draw_polygon(
        self,
        points: List[Tuple["SpatialMeasurement", "SpatialMeasurement"]],
        color: ArcadeColor,
        filled: bool = True,
        line_width: float = 2.0,
    ):
        """Draw a polygon from a list of points."""
        pixel_points = [(self.to_pixels(x), self.to_pixels(y)) for x, y in points]
        
        if filled:
            arcade.draw_polygon_filled(pixel_points, color)
        else:
            arcade.draw_polygon_outline(pixel_points, color, line_width)
    
    def draw_line(
        self,
        x1: "SpatialMeasurement",
        y1: "SpatialMeasurement",
        x2: "SpatialMeasurement",
        y2: "SpatialMeasurement",
        color: ArcadeColor,
        line_width: float = 2.0,
    ):
        """Draw a line between two points."""
        arcade.draw_line(
            self.to_pixels(x1),
            self.to_pixels(y1),
            self.to_pixels(x2),
            self.to_pixels(y2),
            color,
            line_width,
        )
    
    def draw_lines(
        self,
        points: List[Tuple["SpatialMeasurement", "SpatialMeasurement"]],
        color: ArcadeColor,
        line_width: float = 2.0,
    ):
        """Draw connected line segments through a list of points."""
        pixel_points = [(self.to_pixels(x), self.to_pixels(y)) for x, y in points]
        arcade.draw_lines(pixel_points, color, line_width)
    
    def draw_point(
        self,
        x: "SpatialMeasurement",
        y: "SpatialMeasurement",
        color: ArcadeColor,
        size: float = 5.0,
    ):
        """Draw a single point."""
        arcade.draw_point(self.to_pixels(x), self.to_pixels(y), color, size)
    
    def draw_text(
        self,
        text: str,
        x: "SpatialMeasurement",
        y: "SpatialMeasurement",
        color: ArcadeColor = arcade.color.BLACK,
        font_size: int = 12,
        anchor_x: str = "left",
        anchor_y: str = "baseline",
    ):
        """Draw text at the given position."""
        arcade.draw_text(
            text,
            self.to_pixels(x),
            self.to_pixels(y),
            color,
            font_size,
            anchor_x=anchor_x,
            anchor_y=anchor_y,
        )
    
    def draw_arc(
        self,
        x: "SpatialMeasurement",
        y: "SpatialMeasurement",
        radius: "SpatialMeasurement",
        color: ArcadeColor,
        start_angle: float,
        end_angle: float,
        line_width: float = 2.0,
    ):
        """Draw an arc."""
        arcade.draw_arc_outline(
            self.to_pixels(x),
            self.to_pixels(y),
            self.to_pixels(radius) * 2,  # arcade uses width/height, not radius
            self.to_pixels(radius) * 2,
            color,
            start_angle,
            end_angle,
            line_width,
        )


# Global canvas instance for convenience
_default_canvas: Optional[Canvas] = None


def get_canvas() -> Canvas:
    """Get the default global canvas instance."""
    global _default_canvas
    if _default_canvas is None:
        _default_canvas = Canvas()
    return _default_canvas


def set_canvas(canvas: Canvas):
    """Set the default global canvas instance."""
    global _default_canvas
    _default_canvas = canvas
