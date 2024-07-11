

from typing import List, Tuple
import pint
import pygame
from gamegine.analysis.meshing import Map
from gamegine.render.drawable import Drawable
from gamegine.render.renderer import Renderer


class MapDisplay(Drawable):
    
    def __init__(self, map: Map):
        self.map = map
        self.nodes = map.get_all_nodes()
        self.connections = map.get_all_unique_connections()

    def z_index(self):
        return 0
    
    def draw(self, render_scale: pint.Quantity):
        
        for node in self.nodes:
            pygame.draw.circle(pygame.display.get_surface(), (0, 255, 0), (Renderer.to_pixels(node[1][0]), Renderer.to_pixels(node[1][1])), 5)

        for connection in self.connections:
            one, two = connection[0][0], connection[0][1]
            node1 = self.map.decode_coordinates(one)
            node2 = self.map.decode_coordinates(two)
            color = (0, 255, 0) if connection[2] else (255, 0, 0)
            pygame.draw.line(pygame.display.get_surface(), color, (Renderer.to_pixels(node1[0]), Renderer.to_pixels(node1[1])), (Renderer.to_pixels(node2[0]), Renderer.to_pixels(node2[1])), width=1)

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