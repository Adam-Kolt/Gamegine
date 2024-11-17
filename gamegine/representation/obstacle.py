from typing import List, Tuple

from gamegine.representation.boundary.boundary import Boundary, BoundedObject
from gamegine.utils.NCIM.ncim import SpatialMeasurement


class Obstacle(BoundedObject):
    def __init__(self, name: str, bounds: Boundary) -> None:
        super().__init__(bounds, name)
        self.rendering_visibility = True
        pass

    def __str__(self):
        return f"{self.name} Obstacle with bounds: {self.bounds}"

    def invisible(self) -> "Obstacle":
        self.rendering_visibility = False
        return self

    def isVisible(self) -> bool:
        return self.rendering_visibility

    def set_rendering_visibility(self, visibility: bool) -> "Obstacle":
        self.rendering_visibility = visibility
        return self

    # TODO: Move rendering visibility and settings to Drawable
