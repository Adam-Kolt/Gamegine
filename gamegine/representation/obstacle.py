from typing import List, Tuple
from gamegine.representation.bounds import (
    Boundary,
    BoundedObject,
    Line,
    Polygon,
    Rectangle,
    Circle,
)
from gamegine.utils.unit import SpatialMeasurement


class Obstacle(BoundedObject):
    def __init__(self, name: str, bounds: Boundary) -> None:
        super().__init__(bounds)
        self.name = name
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


class Rectangular(Obstacle):
    def __init__(
        self,
        name: str,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        width: SpatialMeasurement,
        height: SpatialMeasurement,
    ) -> None:
        super().__init__(name, Rectangle(x, y, width, height))


class Circular(Obstacle):
    def __init__(
        self,
        name: str,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        radius: SpatialMeasurement,
    ) -> None:
        super().__init__(name, Circle(x, y, radius))


class Polygonal(Obstacle):
    def __init__(
        self, name: str, points: List[Tuple[SpatialMeasurement, SpatialMeasurement]]
    ) -> None:
        super().__init__(name, Polygon(points))
