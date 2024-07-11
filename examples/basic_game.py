

from gamegine.analysis.meshing import Map, VisibilityGraph, TriangulatedGraph
from gamegine.render.analysis import MapDisplay
from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import Circle, ExpandedObjectBounds, Point, SymmetricalX
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Circular, Rectangular
from gamegine.utils.unit import Meter, Centimeter, Feet, Inch
import time


test_game = Game("Test Game")
print("Name:", test_game.name)
test_game.set_field_size(Feet(54)+Inch(3.25), Feet(26) + Inch(11.25))
objs = SymmetricalX([
    Rectangular("Test Rectangle", Feet(1), Feet(1), Feet(2), Feet(2)),
    Circular("Test Circle", Feet(4), Feet(4), Feet(1)),


    ], test_game.half_field_x(), "Red ", "Blue ")


test_game.add_obstacles(objs)
test_game.enable_field_border_obstacles()

map = Map("Test Map")

# Mesh 
map.add_node(Feet(1), Feet(23))
map.add_node(Feet(9), Feet(2))
map.add_node(Feet(4.3), Feet(12))
map.add_node(Feet(20), Feet(23))
map.connect_all_nodes()


obstacles = ExpandedObjectBounds(test_game.get_obstacles())
visibility_map = TriangulatedGraph(obstacles, Feet(3), test_game.get_field_size())


display = MapDisplay(visibility_map)


renderer = Renderer()

renderer.set_game(test_game)
renderer.set_render_scale(Centimeter(1.5))
renderer.init_display()
print("Game set and display initialized")

while renderer.loop():
    renderer.draw_static_elements()
    renderer.draw_element(display)
    time.sleep(0.1)
    renderer.render_frame()

