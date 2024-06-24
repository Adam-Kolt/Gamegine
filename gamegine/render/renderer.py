import pygame
import pint

from gamegine.render.helpers import draw_obstacle
from gamegine.representation.game import Game
from gamegine.utils.unit import Centimeter, RatioOf

pygame.init()

class Renderer:
    renderer = None
    render_scale = Centimeter(10)

    def __init__(self):
        if Renderer.renderer is not None:
            raise Exception("Renderer already exists")
        Renderer.renderer = self
        self.game: Game = None

    def set_render_scale(self, scale: pint.Quantity):
        Renderer.render_scale = scale
        self.init_display()
    
    def set_game(self, game: Game):
        self.game = game

    @staticmethod
    def to_pixels(value: pint.Quantity) -> int:
        return int(RatioOf(value, Renderer.render_scale))

    def init_display(self):
        if self.game is None:
            raise Exception("Game not set")
        dimensions = self.game.field_size
        pygame.display.set_caption(self.game.name)
        pygame.display.set_mode(
            (
                self.to_pixels(dimensions[0]),
                self.to_pixels(dimensions[1])
            )
        )

    def draw_static_elements(self):
        if self.game is None:
            raise Exception("Game not set")
        
        for obstacle in self.game.obstacles.values():
            draw_obstacle(obstacle, Renderer.render_scale)



    def loop(self) -> bool:
        if self.game is None:
            raise Exception("Game not set")


        # Draw Background
        pygame.display.get_surface().fill((230, 230, 230))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False


        return True
    

    def render_frame(self):
        pygame.display.flip()
