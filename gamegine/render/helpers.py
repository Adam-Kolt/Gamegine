from gamegine.representation.bounds import Circle, DiscreteBoundary, Polygon, Rectangle
import pygame

from gamegine.utils.NCIM.ncim import RatioOf, SpatialMeasurement
from .style import get_color, Palette, Shade, Opacity

# TODO: Some of this is kinda cooked and should be configured elsewhere


def draw_fancy_circle(circle: Circle, color: Palette, render_scale: SpatialMeasurement):
    """Draws a fancy circle with a light outer circle and a darker inner circle.

    :param circle: The circle to draw.
    :type circle: :class:`Circle`
    :param color: The color of the circle.
    :type color: :class:`Palette`
    :param render_scale: The scale at which to render the circle.
    :type render_scale: :class:`SpatialMeasurement`
    """
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
    """Draws a fancy rectangle with a light outer rectangle and a darker inner rectangle.

    :param rectangle: The rectangle to draw.
    :type rectangle: :class:`Rectangle`
    :param color: The color of the rectangle.
    :type color: :class:`Palette`
    :param render_scale: The scale at which to render the rectangle.
    :type render_scale: :class:`SpatialMeasurement`
    """
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
    """Draws a polygon.

    :param polygon: The polygon to draw.
    :type polygon: :class:`Polygon`
    :param color: The color of the polygon.
    :type color: :class:`Palette`
    :param render_scale: The scale at which to render the polygon.
    :type render_scale: :class:`SpatialMeasurement`
    """
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
    """Draws a discrete boundary.

    :param discrete_boundary: The discrete boundary to draw.
    :type discrete_boundary: :class:`DiscreteBoundary`
    :param color: The color of the discrete boundary.
    :type color: :class:`Palette`
    :param render_scale: The scale at which to render the discrete boundary.
    :type render_scale: :class:`SpatialMeasurement`
    """
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
    """Draws a point.

    :param x: The x-coordinate of the point.
    :type x: :class:`SpatialMeasurement`
    :param y: The y-coordinate of the point.
    :type y: :class:`SpatialMeasurement`
    :param radius: The radius of the point.
    :type radius: :class:`SpatialMeasurement`
    :param color: The color of the point.
    :type color: :class:`Palette`
    :param render_scale: The scale at which to render the point.
    :type render_scale: :class:`SpatialMeasurement`
    """
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
    """Draws a line.

    :param x1: The x-coordinate of the start of the line.
    :type x1: :class:`SpatialMeasurement`
    :param y1: The y-coordinate of the start of the line.
    :type y1: :class:`SpatialMeasurement`
    :param x2: The x-coordinate of the end of the line.
    :type x2: :class:`SpatialMeasurement`
    :param y2: The y-coordinate of the end of the line.
    :type y2: :class:`SpatialMeasurement`
    :param thickness: The thickness of the line.
    :type thickness: :class:`SpatialMeasurement`
    :param color: The color of the line.
    :type color: :class:`Palette`
    :param render_scale: The scale at which to render the line.
    :type render_scale: :class:`SpatialMeasurement`
    """
    pygame.draw.line(
        pygame.display.get_surface(),
        get_color(color, Shade.LIGHT, Opacity.TRANSPARENTISH),
        (RatioOf(x1, render_scale), RatioOf(y1, render_scale)),
        (RatioOf(x2, render_scale), RatioOf(y2, render_scale)),
        width=int(RatioOf(thickness, render_scale)),
    )


def draw_fancy_boundary(bounds, color: Palette, render_scale: SpatialMeasurement):
    """Draws a fancy boundary.

    :param bounds: The boundary to draw.
    :type bounds: :class:`Circle`, :
    :param color: The color of the boundary.
    :type color: :class:`Palette`
    :param render_scale: The scale at which to render the boundary.
    :type render_scale: :class:`SpatialMeasurement`
    """
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
    """Draws an obstacle.

    :param obstacle: The obstacle to draw.
    :type obstacle: :class:`Obstacle`
    :param render_scale: The scale at which to render the obstacle.
    :type render_scale: :class:`SpatialMeasurement`
    """

    draw_fancy_boundary(obstacle.bounds, Palette.RED, render_scale)
