
import numpy as np
import pint
import math


def ReflectPoint(point, normal):
    return point - 2 * np.dot(point, normal) * normal

def ReflectOverLine(point, line):
    normal = np.array([line[1], -line[0]])
    return ReflectPoint(point, normal)

def ReflectValue1D(value, axis):
    return 2 * axis - value

def GetDistanceBetween(point1, point2):
    return (math.sqrt(point1[0]**2 + point2[0]**2), math.sqrt(point1[1]**2 + point2[1]**2))

def RotateAboutOrigin(vector, angle: pint.Quantity):
    rads = angle.to("rad")
    x = vector[0] * math.cos(rads) - vector[1] * math.sin(rads)
    y = vector[0] * math.sin(rads) + vector[1] * math.cos(rads)
    return [x, y]
