
import pint
from gamegine.representation.bounds import Circle, Polygon, Rectangle
import pygame

from gamegine.utils.unit import RatioOf
from .style import get_color, Palette, Shade, Opacity
# TODO: Some of this is kinda cooked and should be configured elsewhere

def draw_fancy_circle(circle: Circle, color: Palette, render_scale: pint.Quantity):
    # Outer Circle
    pygame.draw.circle(
        pygame.display.get_surface(),
        get_color(color, Shade.LIGHT, Opacity.TRANSPARENTISH),
        (circle.x, circle.y),
        circle.radius
    )

    # Inner Circle
    pygame.draw.circle(
        pygame.display.get_surface(),
        get_color(color, Shade.NORMAL, Opacity.TRANSLUCENT),
        (circle.x, circle.y),
        circle.radius * 0.8
    )


def draw_fancy_rectangle(rectangle: Rectangle, color: Palette, render_scale: pint.Quantity):
    # Outer Rectangle
    pygame.draw.rect(
        pygame.display.get_surface(),
        get_color(color, Shade.LIGHT, Opacity.TRANSPARENTISH),
        pygame.Rect(RatioOf(rectangle.x, render_scale), RatioOf(rectangle.y, render_scale), RatioOf(rectangle.width, render_scale), RatioOf(rectangle.height, render_scale)
    ))

    # Inner Rectangle
    pygame.draw.rect(
        pygame.display.get_surface(),
        get_color(color, Shade.NORMAL, Opacity.TRANSLUCENT),
        pygame.Rect(RatioOf(rectangle.x, render_scale) + RatioOf(rectangle.width, render_scale) * 0.1, RatioOf(rectangle.y, render_scale) + RatioOf(rectangle.height, render_scale) * 0.1, RatioOf(rectangle.width, render_scale) * 0.8, RatioOf(rectangle.height, render_scale) * 0.8)
    )
    



def draw_fancy_polygon(polygon: Polygon, color: Palette, render_scale: pint.Quantity):
    # Outer Polygon
    pygame.draw.polygon(
        pygame.display.get_surface(),
        get_color(color, Shade.LIGHT, Opacity.TRANSPARENTISH),
        polygon.points
    )

    # Inner Polygon
    pygame.draw.polygon(
        pygame.display.get_surface(),
        get_color(color, Shade.NORMAL, Opacity.TRANSLUCENT),
        [(point[0] + (polygon.points[0][0] - point[0]) * 0.1, point[1] + (polygon.points[0][1] - point[1]) * 0.1) for point in polygon.points]
    )

def draw_fancy_boundary(bounds, color: Palette, render_scale: pint.Quantity):
    if isinstance(bounds, Circle):
        draw_fancy_circle(bounds, color, render_scale)
    elif isinstance(bounds, Rectangle):
        draw_fancy_rectangle(bounds, color, render_scale)
    elif isinstance(bounds, Polygon):
        draw_fancy_polygon(bounds, color, render_scale)
    else:
        raise ValueError("Unknown boundary type")
    
def draw_obstacle(obstacle, render_scale: pint.Quantity):
    draw_fancy_boundary(obstacle.bounds, Palette.RED, render_scale)
