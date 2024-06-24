from typing import List, Tuple

import pint


class Boundary(object):
    pass

class Rectangle(Boundary):
    def __init__(self, x: pint.Quantity, y: pint.Quantity, width: pint.Quantity, height: pint.Quantity):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def __str__(self):
        return f"Rectangle(x={self.x}, y={self.y}, width={self.width}, height={self.height})"
    
class Circle(Boundary):
    def __init__(self, x: pint.Quantity, y: pint.Quantity, radius: pint.Quantity):
        self.x = x
        self.y = y
        self.radius = radius

    def __str__(self):
        return f"Circle(x={self.x}, y={self.y}, radius={self.radius})"
    
class Polygon(Boundary):
    def __init__(self, points: List[Tuple[pint.Quantity, pint.Quantity]]):
        self.points = points

    def __str__(self):
        return f"Polygon(points={self.points})"
    
class Line(Boundary):
    def __init__(self, x1: pint.Quantity, y1: pint.Quantity, x2: pint.Quantity, y2: pint.Quantity):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def __str__(self):
        return f"Line(x1={self.x1}, y1={self.y1}, x2={self.x2}, y2={self.y2}"
    
