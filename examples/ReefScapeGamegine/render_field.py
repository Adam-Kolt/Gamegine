from Reefscape import CageLevel, Reefscape, add_cage_obstacles
from gamegine.render.renderer import Renderer, run
from gamegine.utils.NCIM.ncim import Inch
from gamegine.first.alliance import Alliance

# Add cage obstacles before rendering
add_cage_obstacles(
    [CageLevel.DEEP, CageLevel.SHALLOW, CageLevel.SHALLOW], Alliance.BLUE
)
add_cage_obstacles([CageLevel.SHALLOW, CageLevel.DEEP, CageLevel.DEEP], Alliance.RED)

# Create renderer with game - this sets up dimensions automatically
renderer = Renderer.create(game=Reefscape)

# Add all obstacles from the game to the renderer
renderer.add_obstacles(Reefscape.get_obstacles())

# Run the render loop
run()
