import pygame
from gamegine.representation.gamepiece import Gamepiece, GamepiecePhysicalProperties
from gamegine.representation.bounds import Cylinder, Transform3D
from gamegine.utils.NCIM.ncim import Inch, Degree, Centimeter, Pound, SpatialMeasurement


class Coral(Gamepiece):
    name = "Coral"
    bounds = Cylinder(Inch(2), Centimeter(30), Transform3D())
    physical_properties = GamepiecePhysicalProperties(
        mass=Pound(1.5), friction_coefficient=0.5
    )

    def display(
        x: SpatialMeasurement, y: SpatialMeasurement, render_scale: SpatialMeasurement
    ):
        # draw centers pink rectangle sideways in pygame 11 7/8 in long by 4 inch wide
        pygame.draw.rect(
            pygame.display.get_surface(),
            (255, 192, 203),
            (
                x / render_scale - (Inch(11.875) / 2 / render_scale),
                y / render_scale - (Inch(4) / 2 / render_scale),
                Inch(11.875) / render_scale,
                Inch(4) / render_scale,
            ),
        )


class Algae(Gamepiece):
    name = "Algae"
    bounds = Cylinder(Inch(8), Centimeter(10), Transform3D())
    physical_properties = GamepiecePhysicalProperties(
        mass=Pound(0.5), friction_coefficient=0.3
    )
