from typing import Callable
from gamegine.render import helpers
from gamegine.render.style import Palette
from gamegine.representation.bounds import Circle
from gamegine.representation.gamepiece import Gamepiece, GamepiecePhysicalProperties
from gamegine.utils.NCIM.Dimensions.mass import Gram, MassMeasurement, Ounce
from gamegine.utils.NCIM.Dimensions.spatial import Inch, SpatialMeasurement


class Note(Gamepiece):
    name = "Note"
    bounds = Circle(Inch(0), Inch(0), Inch(7)).get_3d(Inch(0), Inch(1))
    physical_properties = GamepiecePhysicalProperties(Ounce(8.3), 0.5)

    # AndyMark Note Dimensions
    INNER_RADIUS = Inch(5)
    OUTER_RADIUS = Inch(7)

    @classmethod
    def display(
        cls,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        render_scale: SpatialMeasurement,
    ):
        helpers.draw_point(x, y, cls.OUTER_RADIUS, Palette.ORANGE, render_scale)
