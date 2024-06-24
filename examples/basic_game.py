

from gamegine.render.renderer import Renderer
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Rectangular
from gamegine.utils.unit import Meter, Centimeter, Feet, Inch
import time


test_game = Game("Test Game")
print("Name:", test_game.name)
test_game.add_obstacle(Rectangular("Test Rectangle", Feet(1) + Inch(1), Meter(0.5), Meter(4), Meter(4)))


renderer = Renderer()

renderer.set_game(test_game)
renderer.set_render_scale(Centimeter(2))
renderer.init_display()
print("Game set and display initialized")

while renderer.loop():
    renderer.draw_static_elements()
    time.sleep(0.1)
    renderer.render_frame()

