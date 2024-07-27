from typing import Callable, List, Tuple
from abc import ABC, abstractmethod
import copy

import pint
import math
from gamegine.representation.base import NamedObject
from gamegine.utils.matematika import ReflectValue1D, RotateAboutOrigin
import shapely.geometry as sg

from gamegine.utils.unit import Inch, List2Std, StdMag, StdMagTuple, Tuple2Std



class Boundary(ABC):

    @abstractmethod
    def translate(self, x: pint.Quantity, y: pint.Quantity) -> 'Boundary':
        pass

    @abstractmethod
    def scale(self, factor: pint.Quantity) -> 'Boundary':
        pass

    @abstractmethod
    def reflect_x(self, axis: pint.Quantity) -> 'Boundary':
        pass

    @abstractmethod
    def reflect_y(self, axis: pint.Quantity) -> 'Boundary':
        pass

    @abstractmethod
    def discretized(self, curve_segments: int = 5) -> 'DiscreteBoundary':
        pass

class DiscreteBoundary(Boundary):
    @abstractmethod
    def get_vertices(self) -> List[Tuple[pint.Quantity, pint.Quantity]]:
        pass

    def discretized(self, curve_segments: int = 5) -> 'DiscreteBoundary':
        return self
    
    def __recompute_plain_points(self):
        self.plain_points = [StdMagTuple(point) for point in self.get_vertices()]
    
    def intersects_line(self, x1: pint.Quantity, y1: pint.Quantity, x2: pint.Quantity, y2: pint.Quantity) -> bool:
        self.__recompute_plain_points()
        return sg.Polygon(self.plain_points).intersects(sg.LineString([StdMagTuple((x1, y1)), StdMagTuple((x2, y2))]))
    
    def intersects_rectangle(self, x: pint.Quantity, y: pint.Quantity, width: pint.Quantity, height: pint.Quantity) -> bool:
        self.__recompute_plain_points()
        return sg.Polygon(self.plain_points).intersects(sg.box(x, y, x + width, y + height))

    def contains_point(self, x: pint.Quantity, y: pint.Quantity) -> bool:
        self.__recompute_plain_points()
        return sg.Polygon(self.plain_points).contains(sg.Point(StdMagTuple((x, y))))
    
    def __convert_coordinate_sequence(self, coord_sequence) -> List[Tuple[float, float]]:
        return [Tuple2Std((x,y)) for x,y in coord_sequence]


    def buffered(self, distance: pint.Quantity) -> 'DiscreteBoundary': # Efficiency is cooked here...but its easy
        self.__recompute_plain_points()
        coords = self.__convert_coordinate_sequence(sg.Polygon(self.plain_points).buffer(StdMag(distance), quad_segs=1).exterior.coords)
        return Polygon(coords)


class Rectangle(DiscreteBoundary):
    def __init__(self, x: pint.Quantity, y: pint.Quantity, width: pint.Quantity, height: pint.Quantity):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def __str__(self):
        return f"Rectangle(x={self.x}, y={self.y}, width={self.width}, height={self.height})"
    
    def translate(self, x: pint.Quantity, y: pint.Quantity) -> 'Rectangle':
        self.x += x
        self.y += y
        return self
    
    def scale(self, factor: pint.Quantity) -> 'Rectangle':
        self.width *= factor
        self.height *= factor
        return self
    
    def reflect_x(self, axis: pint.Quantity) -> 'Rectangle':
        self.x = ReflectValue1D(self.x, axis)
        self.x -= self.width
        return self
    
    def reflect_y(self, axis: pint.Quantity) -> 'Rectangle':
        self.y = ReflectValue1D(self.y, axis)
        self.y -= self.height
        return self
    
    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary:
        return self
    
    def get_vertices(self) -> List[Tuple[pint.Quantity, pint.Quantity]]:
        return [(self.x, self.y), (self.x + self.width, self.y), (self.x + self.width, self.y + self.height), (self.x, self.y + self.height)]
    
class Circle(Boundary):
    def __init__(self, x: pint.Quantity, y: pint.Quantity, radius: pint.Quantity):
        self.x = x
        self.y = y
        self.radius = radius

    def __str__(self):
        return f"Circle(x={self.x}, y={self.y}, radius={self.radius})"
    
    def translate(self, x: pint.Quantity, y: pint.Quantity) -> 'Circle':
        self.x += x
        self.y += y
        return self
    
    def scale(self, factor: pint.Quantity) -> 'Circle':
        self.radius *= factor
        return self
    
    def reflect_x(self, axis: pint.Quantity) -> 'Circle':
        self.x = ReflectValue1D(self.x, axis)
        return self
    
    def reflect_y(self, axis: pint.Quantity) -> 'Circle':
        self.y = 2 * axis - self.y
        return self
    
    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary:
        points = []
        step = 2 * math.pi / curve_segments
        for i in range(curve_segments):
            angle = step*i
            points.append((math.cos(angle)*self.radius + self.x, math.sin(angle)*self.radius + self.y))
        return Polygon(points)

    
class Polygon(DiscreteBoundary):
    def __init__(self, points: List[Tuple[pint.Quantity, pint.Quantity]]):
        self.points = points

    def __str__(self):
        return f"Polygon(points={self.points})"
    
    def translate(self, x: pint.Quantity, y: pint.Quantity) -> 'Polygon':
        self.points = [(x + point[0], y + point[1]) for point in self.points]
        return self
    
    def scale(self, factor: pint.Quantity) -> 'Polygon':
        self.points = [(factor * point[0], factor * point[1]) for point in self.points]
        return self
    
    def reflect_x(self, axis: pint.Quantity) -> 'Polygon':
        self.points = [(ReflectValue1D(point[0], axis), point[1]) for point in self.points]
        return self
    
    def reflect_y(self, axis: pint.Quantity) -> 'Polygon':
        self.points = [(point[0], ReflectValue1D(point[1], axis)) for point in self.points]
        return self
    
    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary:
        return self
    
    def get_vertices(self) -> List[Tuple[pint.Quantity, pint.Quantity]]:
        return self.points

    
    
class Line(DiscreteBoundary):
    def __init__(self, x1: pint.Quantity, y1: pint.Quantity, x2: pint.Quantity, y2: pint.Quantity):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def __str__(self):
        return f"Line(x1={self.x1}, y1={self.y1}, x2={self.x2}, y2={self.y2}"
    
    def translate(self, x: pint.Quantity, y: pint.Quantity) -> 'Line':
        self.x1 += x
        self.y1 += y
        self.x2 += x
        self.y2 += y
        return self
    
    def scale(self, factor: pint.Quantity) -> 'Line':
        self.x1 *= factor
        self.y1 *= factor
        self.x2 *= factor
        self.y2 *= factor
        return self
    
    def reflect_y(self, axis: pint.Quantity) -> 'Line':
        self.y1 = ReflectValue1D(self.y1, axis)
        self.y2 = ReflectValue1D(self.y2, axis)
        return self
    
    def reflect_x(self, axis: pint.Quantity) -> 'Line':
        self.x1 = ReflectValue1D(self.x1, axis)
        self.x2 = ReflectValue1D(self.x2, axis)
        return self
    
    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary:
        return self
    
    def get_vertices(self) -> List[Tuple[pint.Quantity, pint.Quantity]]:
        return [(self.x1, self.y1), (self.x2, self.y2)]
    
class Point(DiscreteBoundary):
    def __init__(self, x: pint.Quantity, y: pint.Quantity):
        self.x = x
        self.y = y

    def __str__(self):
        return f"Point(x={self.x}, y={self.y})"
    
    def translate(self, x: pint.Quantity, y: pint.Quantity) -> 'Point':
        self.x += x
        self.y += y
        return self
    
    def scale(self, factor: pint.Quantity) -> 'Point':
        self.x *= factor
        self.y *= factor
        return self
    
    def reflect_y(self, axis: pint.Quantity) -> 'Point':
        self.y = ReflectValue1D(self.y, axis)
        return self
    
    def reflect_x(self, axis: pint.Quantity) -> 'Point':
        self.x = ReflectValue1D(self.x, axis)
        return self
    
    def discretized(self, curve_segments: int = 5) -> DiscreteBoundary:
        return self
    
    def get_vertices(self) -> List[Tuple[pint.Quantity, pint.Quantity]]:
        return [(self.x, self.y)]
    
class BoundedObject(NamedObject):
    def __init__(self, bounds: Boundary) -> None:
        self.bounds = bounds

    def mirrored_over_horizontal(self, axis: pint.Quantity) -> 'BoundedObject':
        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.reflect_y(axis)
        return obj
    
    def mirrored_over_horizontal_ip(self, axis: pint.Quantity) -> None:
        self.bounds = self.bounds.reflect_y(axis)
    
    def mirrored_over_vertical(self, axis: pint.Quantity) -> 'BoundedObject':
        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.reflect_x(axis)
        return obj
    
    def mirrored_over_vertical_ip(self, axis: pint.Quantity) -> 'BoundedObject':
        self.bounds = self.bounds.reflect_x(axis)
        return self
    
    def scaled(self, factor: pint.Quantity) -> 'BoundedObject':
        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.scale(factor)
        return obj
    
    def translated(self, x: pint.Quantity, y: pint.Quantity) -> 'BoundedObject':
        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.translate(x, y)
        return obj
    
    def translate_ip(self, x: pint.Quantity, y: pint.Quantity) -> 'BoundedObject':
        self.bounds = self.bounds.translate(x, y)
        return self
    
    def discretized(self, curve_segments: int = 5) -> 'BoundedObject':
        obj = copy.deepcopy(self)
        obj.bounds = obj.bounds.discretized(curve_segments)
        return obj
    
    def discretize_ip(self, curve_segments: int = 5) -> 'BoundedObject':
        self.bounds = self.bounds.discretized(curve_segments)
        return self

def LineIntersectsAnyBound(bounds: List[DiscreteBoundary], x1: pint.Quantity, y1: pint.Quantity, x2: pint.Quantity, y2: pint.Quantity) -> bool:
    for bound in bounds:
        if not isinstance(bound, DiscreteBoundary):
            print(f"Discretizing continous bound {bound}")
        bound.discretized() # If not already discretized
        if bound.intersects_line(x1, y1, x2, y2):
            return True
    return False

def CircularPattern(objects: List[BoundedObject], center: Tuple[pint.Quantity, pint.Quantity], angle: pint.Quantity, num_objects: int, prefix_func: Callable[[int], str]) -> List[BoundedObject]:
    if num_objects <= 1:
        raise Exception("Number of objects must be greater than 1")
    out = []
    angle_increment = angle / num_objects
    for object in objects:
        centerToObject = [object.bounds.x - center[0], object.bounds.y - center[1]]
        objectToCenter = [centerToObject[0] * -1, centerToObject[1] * -1]
        
        for i in range(1, num_objects):
            angle = angle_increment * i
            new_vector = RotateAboutOrigin(centerToObject, -angle)
            out.append(object.translated(objectToCenter[0], objectToCenter[1]).translate_ip(new_vector[0], new_vector[1]).prefix(prefix_func(i)))
    return objects + out


def SymmetricalX(objects: List[BoundedObject], axis: pint.Quantity, prefix: str, prefix_og: str = "") -> List[BoundedObject]:
    return [obj.prefix(prefix_og) for obj in objects] + [obj.mirrored_over_vertical(axis).prefix(prefix) for obj in objects]

def SymmetricalY(objects: List[BoundedObject], axis: pint.Quantity, prefix: str, prefix_og: str = "") -> List[BoundedObject]:
    return [obj.prefix(prefix_og) for obj in objects] + [obj.mirrored_over_horizontal(axis).prefix(prefix) for obj in objects]

def SymmetricalXY(objects: List[BoundedObject], axis_x: pint.Quantity, axis_y: pint.Quantity, prefix: str, prefix_og: str = "") -> List[BoundedObject]:
    return [obj.prefix(prefix_og) for obj in objects] + [obj.mirrored_over_vertical(axis_y).mirrored_over_horizontal_ip(axis_x).prefix(prefix) for obj in objects]

def ExpandedObjectBounds(objects: List[BoundedObject], robot_radius: pint.Quantity = Inch(21), discretization_quality=4) -> List[DiscreteBoundary]:
    return [object.bounds.discretized(discretization_quality).buffered(robot_radius) for object in objects]

def ExpandedBounds(bounds: List[Boundary], robot_radius: pint.Quantity = Inch(21), discretization_quality=4) -> List[DiscreteBoundary]:
    return [bound.discretized(discretization_quality).buffered(robot_radius) for bound in bounds]