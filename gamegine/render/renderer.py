"""Minimal pygame based renderer used by examples and debugging tools."""

import pygame


from gamegine.render.drawable import Drawable
from gamegine.render.helpers import draw_obstacle
from gamegine.representation.game import Game
from gamegine.utils.NCIM.ncim import Centimeter, RatioOf, SpatialMeasurement

pygame.init()


class Renderer:
    """Singleton wrapper around a pygame surface for quick visualisation."""

    renderer = None
    render_scale = Centimeter(10)
    clock = pygame.time.Clock()
    ANIMATION_TICKS = 500

    def __init__(self):
        """Create the renderer singleton and bootstrap pygame state."""
        if Renderer.renderer is not None:
            raise Exception("Renderer already exists")
        Renderer.renderer = self
        self.game: Game = None
        self.last_animation_loop = pygame.time.get_ticks()

    def set_render_scale(self, scale: SpatialMeasurement):
        """Set the number of field units that correspond to a single pixel."""
        Renderer.render_scale = scale
        self.init_display()

    def set_game(self, game: Game):
        """Assign the active :class:`~gamegine.representation.game.Game`."""
        self.game = game

    def __calculate_current_tick(self):  # Used for easier animation, 100 ticks per second
        """Return the animation tick counter used when interpolating motion."""
        ticks = int((pygame.time.get_ticks() - self.last_animation_loop) / 10)
        if ticks > self.ANIMATION_TICKS:
            self.last_animation_loop = pygame.time.get_ticks()
        return ticks

    @staticmethod
    def to_pixels(value: SpatialMeasurement) -> int:
        """Convert an NCIM measurement to the active pixel scale."""
        return int(RatioOf(value, Renderer.render_scale))

    def init_display(self):
        """Create the pygame window and size it according to the selected game."""
        if self.game is None:
            raise Exception("Game not set")
        dimensions = self.game.field_size
        pygame.display.set_caption(self.game.name)
        pygame.display.set_mode(
            (self.to_pixels(dimensions[0]), self.to_pixels(dimensions[1]))
        )
        # TODO: Logo kinda bad, make better

    #        logo = pygame.image.load("gamegine/render/assets/logo.png")
    # pygame.display.set_icon(logo)

    def draw_static_elements(self):
        """Render all static obstacles and interactables for the active game."""
        if self.game is None:
            raise Exception("Game not set")

        for obstacle in self.game.static_obstacles.values():
            if obstacle.isVisible():
                draw_obstacle(obstacle, Renderer.render_scale)

        for interactable in self.game.interactables.values():

            interactable.draw(Renderer.render_scale)

    def draw_element(self, element: Drawable):
        """Render a single drawable entity using the current pixel scale."""
        element.draw(Renderer.render_scale)

    def draw_elements(self, elements: list[Drawable]):
        """Iterate over and draw a sequence of drawable entities."""
        for element in elements:
            self.draw_element(element)

    def loop(self) -> bool:
        """Process window events and clear the buffer ready for drawing."""
        if self.game is None:
            raise Exception("Game not set")

        # Draw Background
        pygame.display.get_surface().fill((230, 230, 230))

        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                return False

        return events

    def render_frame(self):
        """Swap the pygame buffers so the latest frame is displayed."""
        pygame.display.flip()
