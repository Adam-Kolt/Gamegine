

from typing import List, Tuple
import pint
import pygame
from gamegine.analysis.meshing import Map
from gamegine.render.drawable import Drawable
from gamegine.render.renderer import Renderer


class PathDisplay(Drawable):
        
        def __init__(self, path: List[Tuple[pint.Quantity, pint.Quantity]]):
            self.path = path
            self.map = map
    
        def z_index(self):
            return 1
        
        def draw(self, render_scale: pint.Quantity):
            for i in range(len(self.path) - 1):
                node1 = self.path[i]
                node2 = self.path[i + 1]
                pygame.draw.line(pygame.display.get_surface(), (255, 0, 0), (Renderer.to_pixels(node1[0]), Renderer.to_pixels(node1[1])), (Renderer.to_pixels(node2[0]), Renderer.to_pixels(node2[1])), width=2)