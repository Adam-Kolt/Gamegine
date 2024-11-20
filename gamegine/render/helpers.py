import pygame

from gamegine.render.renderer import Renderer
from gamegine.representation.boundary.shape2D import Circle, Polygon, Rectangle
from gamegine.utils.NCIM.ncim import RatioOf, SpatialMeasurement, Centimeter
from .style import Base, Palette


def draw_line(
    x1: SpatialMeasurement,
    y1: SpatialMeasurement,
    x2: SpatialMeasurement,
    y2: SpatialMeasurement,
    color: Base = Palette.BLACK,
    thickness: SpatialMeasurement = Centimeter(1),
    opacity=1.0,
) -> None:
    pygame.draw.line(
        pygame.display.get_surface(),
        color.get_color().get_pygame_color(),
        (RatioOf(x1, Renderer.render_scale), RatioOf(y1, Renderer.render_scale)),
        (RatioOf(x2, Renderer.render_scale), RatioOf(y2, Renderer.render_scale)),
        RatioOf(thickness, Renderer.render_scale),
    )


def draw_point(
    x: SpatialMeasurement,
    y: SpatialMeasurement,
    radius: SpatialMeasurement,
    color: Base = Palette.BLACK,
    render_scale: SpatialMeasurement = Centimeter(10),
    opacity=1.0,
) -> None:
    pygame.draw.circle(
        pygame.display.get_surface(),
        color.get_color().get_pygame_color(),
        (
            RatioOf(x, render_scale),
            RatioOf(y, render_scale),
        ),
        RatioOf(radius, render_scale),
    )


def draw_circle(
    x: SpatialMeasurement,
    y: SpatialMeasurement,
    radius: SpatialMeasurement,
    color: Base = Palette.BLACK,
    thickness: SpatialMeasurement = Centimeter(1),
    opacity=1.0,
) -> None:
    pygame.draw.circle(
        pygame.display.get_surface(),
        color.get_color(-0.1).get_pygame_color(),
        (RatioOf(x, Renderer.render_scale), RatioOf(y, Renderer.render_scale)),
        RatioOf(radius, Renderer.render_scale),
    )

    pygame.draw.circle(
        pygame.display.get_surface(),
        color.get_color().get_pygame_color(),
        (RatioOf(x, Renderer.render_scale), RatioOf(y, Renderer.render_scale)),
        RatioOf(radius - thickness, Renderer.render_scale),
    )


def draw_rectangle(
    x: SpatialMeasurement,
    y: SpatialMeasurement,
    width: SpatialMeasurement,
    height: SpatialMeasurement,
    color: Base = Palette.BLACK,
    thickness: SpatialMeasurement = Centimeter(1),
    opacity=1.0,
) -> None:
    pygame.draw.rect(
        pygame.display.get_surface(),
        color.get_color().get_pygame_color(),
        (
            RatioOf(x, Renderer.render_scale),
            RatioOf(y, Renderer.render_scale),
            RatioOf(width, Renderer.render_scale),
            RatioOf(height, Renderer.render_scale),
        ),
    )

    pygame.draw.rect(
        pygame.display.get_surface(),
        color.get_color(-0.1).get_pygame_color(),
        (
            RatioOf(x, Renderer.render_scale),
            RatioOf(y, Renderer.render_scale),
            RatioOf(width, Renderer.render_scale),
            RatioOf(height, Renderer.render_scale),
        ),
        RatioOf(thickness, Renderer.render_scale),
    )


def draw_polygon(
    vertices: list[tuple[SpatialMeasurement, SpatialMeasurement]],
    color: Base = Palette.BLACK,
    thickness: SpatialMeasurement = Centimeter(1),
    opacity=1.0,
) -> None:
    pygame.draw.polygon(
        pygame.display.get_surface(),
        color.get_color().get_pygame_color(),
        [
            (
                RatioOf(vertex[0], Renderer.render_scale),
                RatioOf(vertex[1], Renderer.render_scale),
            )
            for vertex in vertices
        ],
    )

    pygame.draw.polygon(
        pygame.display.get_surface(),
        color.get_color(-0.1).get_pygame_color(),
        [
            (
                RatioOf(vertex[0], Renderer.render_scale),
                RatioOf(vertex[1], Renderer.render_scale),
            )
            for vertex in vertices
        ],
        RatioOf(thickness, Renderer.render_scale),
    )
