"""Reusable drawing helpers that encapsulate pygame boilerplate."""

from gamegine.representation.bounds import Circle, DiscreteBoundary, Polygon, Rectangle
import pygame

from gamegine.utils.NCIM.ncim import RatioOf, SpatialMeasurement
from .style import get_color, Palette, Shade, Opacity

# TODO: Some of this is kinda cooked and should be configured elsewhere


def draw_fancy_circle(circle: Circle, color: Palette, render_scale: SpatialMeasurement):
    """Draw a stylised circle with a subtle gradient for depth."""
    # Outer Circle
    pygame.draw.circle(
        pygame.display.get_surface(),
        get_color(color, Shade.LIGHT, Opacity.TRANSPARENTISH),
        (RatioOf(circle.x, render_scale), RatioOf(circle.y, render_scale)),
        RatioOf(circle.radius, render_scale),
    )

    # Inner Circle
    pygame.draw.circle(
        pygame.display.get_surface(),
        get_color(color, Shade.NORMAL, Opacity.TRANSLUCENT),
        (RatioOf(circle.x, render_scale), RatioOf(circle.y, render_scale)),
        RatioOf(circle.radius, render_scale) - 5,
    )


def draw_fancy_rectangle(
    rectangle: Rectangle, color: Palette, render_scale: SpatialMeasurement
):
    """Render a rectangle with a two-tone fill so obstacles pop onscreen."""
    border_width = 5
    border_radius = 5

    # Outer Rectangle
    pygame.draw.rect(
        pygame.display.get_surface(),
        get_color(color, Shade.LIGHT, Opacity.TRANSPARENTISH),
        pygame.Rect(
            RatioOf(rectangle.x, render_scale),
            RatioOf(rectangle.y, render_scale),
            RatioOf(rectangle.width, render_scale),
            RatioOf(rectangle.height, render_scale),
        ),
        border_radius=border_radius,
    )

    # Inner Rectangle
    pygame.draw.rect(
        pygame.display.get_surface(),
        get_color(color, Shade.NORMAL, Opacity.TRANSPARENTISH),
        pygame.Rect(
            RatioOf(rectangle.x, render_scale) + border_width,
            RatioOf(rectangle.y, render_scale) + border_width,
            RatioOf(rectangle.width, render_scale) - border_width * 2,
            RatioOf(rectangle.height, render_scale) - border_width * 2,
        ),
        border_radius=border_radius,
    )


def draw_fancy_polygon(
    polygon: Polygon, color: Palette, render_scale: SpatialMeasurement
):
    """Paint a filled polygon honouring the configured colour palette."""
    # Outer Polygon
    pygame.draw.polygon(
        pygame.display.get_surface(),
        get_color(color, Shade.LIGHT, Opacity.TRANSPARENTISH),
        [
            (RatioOf(point[0], render_scale), RatioOf(point[1], render_scale))
            for point in polygon.points
        ],
    )


def draw_fancy_discrete_boundary(
    discrete_boundary, color: Palette, render_scale: SpatialMeasurement
):
    """Render polygonal boundaries with the same styling as other shapes."""
    pygame.draw.polygon(
        pygame.display.get_surface(),
        get_color(color, Shade.LIGHT, Opacity.TRANSPARENTISH),
        [
            (RatioOf(point[0], render_scale), RatioOf(point[1], render_scale))
            for point in discrete_boundary.get_vertices()
        ],
    )


def draw_point(
    x: SpatialMeasurement,
    y: SpatialMeasurement,
    radius: SpatialMeasurement,
    color: Palette,
    render_scale: SpatialMeasurement,
):
    """Draw a filled point marker respecting the NCIM render scale."""
    pygame.draw.circle(
        pygame.display.get_surface(),
        get_color(color, Shade.LIGHT, Opacity.TRANSPARENTISH),
        (RatioOf(x, render_scale), RatioOf(y, render_scale)),
        RatioOf(radius, render_scale),
    )


def draw_line(
    x1: SpatialMeasurement,
    y1: SpatialMeasurement,
    x2: SpatialMeasurement,
    y2: SpatialMeasurement,
    thickness: SpatialMeasurement,
    color: Palette,
    render_scale: SpatialMeasurement,
):
    """Draw a straight segment using NCIM-aware measurements."""
    pygame.draw.line(
        pygame.display.get_surface(),
        get_color(color, Shade.LIGHT, Opacity.TRANSPARENTISH),
        (RatioOf(x1, render_scale), RatioOf(y1, render_scale)),
        (RatioOf(x2, render_scale), RatioOf(y2, render_scale)),
        width=int(RatioOf(thickness, render_scale)),
    )


def draw_fancy_boundary(bounds, color: Palette, render_scale: SpatialMeasurement):
    """Dispatch helper that chooses the correct renderer for a boundary."""
    if isinstance(bounds, Circle):
        draw_fancy_circle(bounds, color, render_scale)
    elif isinstance(bounds, Rectangle):
        draw_fancy_rectangle(bounds, color, render_scale)
    elif isinstance(bounds, Polygon):
        draw_fancy_polygon(bounds, color, render_scale)
    elif issubclass(bounds.__class__, DiscreteBoundary):
        draw_fancy_discrete_boundary(bounds, color, render_scale)
    else:
        raise ValueError("Unknown boundary type")


def draw_obstacle(obstacle, render_scale: SpatialMeasurement):
    """Draw an obstacle using the default palette configuration."""

    draw_fancy_boundary(obstacle.bounds, Palette.RED, render_scale)
