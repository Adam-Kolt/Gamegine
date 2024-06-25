

from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import Circle, SymmetricalX
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Circular, Rectangular
from gamegine.utils.unit import Meter, Centimeter, Feet, Inch
import time


test_game = Game("FRC Crescendo 2024")
print("Name:", test_game.name)
test_game.set_field_size(Feet(54)+Inch(3.25), Feet(26) + Inch(11.25))
objs = SymmetricalX([
    Rectangular("Test Rectangle", Feet(1), Feet(1), Feet(2), Feet(2)),
    Circular("Stage Leg 1", Inch(133), Inch(161.62), Inch(7)),


    ], test_game.half_field_x(), "Red ", "Blue ")


test_game.add_obstacles(objs)



renderer = Renderer()

renderer.set_game(test_game)
renderer.set_render_scale(Centimeter(1.5))
renderer.init_display()
print("Game set and display initialized")

while renderer.loop():
    renderer.draw_static_elements()
    time.sleep(0.1)
    renderer.render_frame()

