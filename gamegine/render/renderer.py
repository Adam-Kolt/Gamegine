import pygame


from gamegine.render.drawable import Drawable
from gamegine.render.helpers import draw_obstacle
from gamegine.representation.game import Game
from gamegine.utils.NCIM.ncim import Centimeter, RatioOf, SpatialMeasurement

pygame.init()


class Renderer:
    """Class for rendering the game field and elements on the screen."""

    renderer = None
    render_scale = Centimeter(10)
    clock = pygame.time.Clock()
    ANIMATION_TICKS = 500

    def __init__(self):
        if Renderer.renderer is not None:
            raise Exception("Renderer already exists")
        Renderer.renderer = self
        self.game: Game = None
        self.last_animation_loop = pygame.time.get_ticks()

    def set_render_scale(self, scale: SpatialMeasurement):
        """Sets the scale at which to render the game field. This is defined as the number of units of measurement per pixel.

        :param scale: The scale at which to render the game field.
        :type scale: :class:`SpatialMeasurement`
        """
        Renderer.render_scale = scale
        self.init_display()

    def set_game(self, game: Game):
        """Sets the game to render.

        :param game: The game to render.
        :type game: :class:`Game`
        """
        self.game = game

    def __calculate_current_tick(
        self,
    ):  # Used for easier animation, 100 ticks per second
        ticks = int((pygame.time.get_ticks() - self.last_animation_loop) / 10)
        if ticks > self.ANIMATION_TICKS:
            self.last_animation_loop = pygame.time.get_ticks()
        return ticks

    @staticmethod
    def to_pixels(value: SpatialMeasurement) -> int:
        """Converts a value in the game's units of measurement to pixels.

        :param value: The value to convert to pixels.
        :type value: :class:`SpatialMeasurement`
        :return: The value in pixels.
        :rtype: int
        """
        return int(RatioOf(value, Renderer.render_scale))

    def init_display(self):
        if self.game is None:
            raise Exception("Game not set")
        dimensions = self.game.field_size
        pygame.display.set_caption(self.game.name)
        pygame.display.set_mode(
            (self.to_pixels(dimensions[0]), self.to_pixels(dimensions[1]))
        )
        # TODO: Logo kinda bad, make better
        logo = pygame.image.load("gamegine/render/assets/logo.png")
        pygame.display.set_icon(logo)

    def draw_static_elements(self):
        """Draws the static elements of the game field."""
        if self.game is None:
            raise Exception("Game not set")

        for obstacle in self.game.static_obstacles.values():
            if obstacle.isVisible():
                draw_obstacle(obstacle, Renderer.render_scale)

        for interactable in self.game.interactables.values():

            interactable.draw(Renderer.render_scale)

    def draw_element(self, element: Drawable):
        """Draws a drawable element on the screen.

        :param element: The element to draw.
        :type element: :class:`Drawable`
        """
        element.draw(Renderer.render_scale)

    def draw_elements(self, elements: list[Drawable]):
        """Draws a list of drawable elements on the screen.

        :param elements: The elements to draw.
        :type elements: List[:class:`Drawable`]
        """
        for element in elements:
            self.draw_element(element)

    def loop(self) -> bool:
        """The main loop for the renderer."""
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
        """Renders a frame of the game and updates the display."""
        pygame.display.flip()
