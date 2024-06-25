

from typing import List, Tuple
from gamegine.representation.bounds import Boundary, BoundedObject, Polygon, Rectangle, Circle

class Obstacle(BoundedObject): 
    def __init__(self, name: str, bounds: Boundary) -> None:
        super().__init__(bounds)
        self.name = name
        pass

    def __str__(self):
        return f"{self.name} Obstacle with bounds: {self.bounds}"
    
class Rectangular(Obstacle):
    def __init__(self, name: str, x: float, y: float, width: float, height: float) -> None:
        super().__init__(name, Rectangle(x, y, width, height))

class Circular(Obstacle):
    def __init__(self, name: str, x: float, y: float, radius: float) -> None:
        super().__init__(name, Circle(x, y, radius))

class Polygonal(Obstacle):
    def __init__(self, name: str, points: List[Tuple[float, float]]) -> None:
        super().__init__(name, Polygon(points))


