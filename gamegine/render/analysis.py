

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
            if len(connection) != 2:
                print(f"WTH! {connection} Connection is not a pair of nodes.")
                continue
            # Literally no clue whether this is faster or slower than just converting the set to a list and indexing it
            one, two = connection.pop(), connection.pop()
            node1 = self.map.decode_coordinates(one)
            node2 = self.map.decode_coordinates(two)
            connection.add(one)
            connection.add(two)
            
            pygame.draw.line(pygame.display.get_surface(), (0, 255, 0), (Renderer.to_pixels(node1[0]), Renderer.to_pixels(node1[1])), (Renderer.to_pixels(node2[0]), Renderer.to_pixels(node2[1])), width=1)