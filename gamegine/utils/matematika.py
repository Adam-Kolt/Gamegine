
import numpy as np
import pint
import math

from gamegine.utils.logging import Debug
from gamegine.utils.unit import AngularMeasurement, AngularUnits

def Clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def ReflectPoint(point, normal):
    return point - 2 * np.dot(point, normal) * normal

def ReflectOverLine(point, line):
    normal = np.array([line[1], -line[0]])
    return ReflectPoint(point, normal)

def ReflectValue1D(value, axis):
    return 2 * axis - value

def GetDistanceBetween(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

def CoordinateInRectangle(point, rectangle):
    return rectangle[0] <= point[0] <= rectangle[2] and rectangle[1] <= point[1] <= rectangle[3]

def RotateAboutOrigin(vector, angle: AngularMeasurement):
    Debug(angle)
    rads = angle.to(AngularUnits.Radian)
    x = vector[0] * math.cos(rads) - vector[1] * math.sin(rads)
    y = vector[0] * math.sin(rads) + vector[1] * math.cos(rads)
    return [x, y]

def AngleBetweenVectors(vector1, vector2):
    ratio = Clamp(np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2)), -1, 1)
    return math.acos(ratio)
