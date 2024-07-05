

from gamegine.analysis.meshing import VisibilityGraph
from gamegine.render.analysis import MapDisplay, PathDisplay
from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import Circle, Point, SymmetricalX, CircularPattern
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Circular, Polygonal, Rectangular
from gamegine.utils.unit import Degree, Meter, Centimeter, Feet, Inch
from gamegine.analysis import pathfinding
import time


test_game = Game("FRC Crescendo 2024")
print("Name:", test_game.name)
test_game.set_field_size(Feet(54)+Inch(3.25), Feet(26) + Inch(11.25))
objs = SymmetricalX([
    *CircularPattern(
        [Circular("Stage Leg", Inch(133), Inch(161.62), Inch(7))],
        (Inch(133)+Inch(59.771), Inch(161.62)), 
        Degree(360),
        3,
        lambda i : str(i)
    ),
    Polygonal("Subwoofer", [
        (Inch(0), Inch(64.081)),
        (Inch(0), Inch(64.081) + Inch(82.645)),
        (Inch(35.695), Inch(64.081) + Inch(82.645) - Inch(20.825)),
        (Inch(35.695), Inch(64.081) + Inch(20.825)),]),

    Polygonal("Source", [
        (Inch(0), Inch(281.5)),
        (Inch(0), test_game.full_field_y()),
        (Inch(72.111), test_game.full_field_y())
    ]),



    ], test_game.half_field_x(), "Red ", "Blue ")


test_game.add_obstacles(objs)

starting_points = [    
    Point(Inch(49), Inch(29.64)),
    Point(Inch(49), Inch(66)),
    Point(Inch(49), Inch(132)),
    Point(Inch(49), Inch(198)),
    ]
points = [point.get_vertices()[0] for point in starting_points]

map = VisibilityGraph(test_game.get_obstacles(), points, test_game.field_size)

path = pathfinding.findPath(map, (Inch(30),Inch(20)), (Feet(50), Feet(9)), pathfinding.AStar, pathfinding.InitialConnectionPolicy.SnapToClosest)
path_display = PathDisplay(path)

map_visual = MapDisplay(map)

renderer = Renderer()

renderer.set_game(test_game)
renderer.set_render_scale(Centimeter(1.5))
renderer.init_display()
print("Game set and display initialized")

while renderer.loop():
    renderer.draw_static_elements()
    renderer.draw_element(map_visual)
    renderer.draw_element(path_display)
    time.sleep(0.1)
    renderer.render_frame()

