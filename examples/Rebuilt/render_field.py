from Rebuilt import create_rebuilt_game
from gamegine.render.renderer import Renderer, run
from gamegine.utils.NCIM.ncim import Inch
from gamegine.first.alliance import Alliance


game = create_rebuilt_game()
# Create renderer with game - this sets up dimensions automatically
renderer = Renderer.create(game=game)

# Add all obstacles and zones from the game to the renderer
renderer.add_game_objects(game)


# Run the render loop
run()
