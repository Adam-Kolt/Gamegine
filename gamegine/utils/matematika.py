import numpy as np
import pint
import math

from gamegine.utils.logging import Debug
from gamegine.utils.NCIM.ncim import AngularMeasurement
from gamegine.utils.NCIM.Dimensions import angular


def Clamp(value, min_value, max_value):
    """Clamps a value between a minimum and maximum value.

    :param value: The value to clamp.
    :type value: float
    :param min_value: The minimum value.
    :type min_value: float
    :param max_value: The maximum value.
    :type max_value: float
    :return: The clamped value.
    :rtype: float
    """
    return max(min(value, max_value), min_value)


def ReflectPoint(point, normal):
    """Reflects a point over a normal vector.

    :param point: The point to reflect.
    :type point: list
    :param normal: The normal vector to reflect over.
    :type normal: list
    :return: The reflected point.
    :rtype: list
    """
    return point - 2 * np.dot(point, normal) * normal


def ReflectOverLine(point, line):
    """Reflects a point over a line.

    :param point: The point to reflect.
    :type point: list
    :param line: The line to reflect over.
    :type line: list
    :return: The reflected point.
    :rtype: list
    """
    normal = np.array([line[1], -line[0]])
    return ReflectPoint(point, normal)


def ReflectValue1D(value, axis):
    """Reflects a value over an axis.

    :param value: The value to reflect.
    :type value: float
    :param axis: The axis to reflect over.
    :type axis: float
    :return: The reflected value.
    :rtype: float
    """
    return 2 * axis - value


def GetDistanceBetween(point1, point2):
    """Returns the distance between two points defined by (x, y) pairs.

    :param point1: The first point.
    :type point1: list
    :param point2: The second point.
    :type point2: list
    :return: The distance between the two points.
    :rtype: float
    """
    return np.linalg.norm(np.array(point1) - np.array(point2))


def CoordinateInRectangle(point, rectangle):
    """Determines if a coordinate is within a rectangle, defined by two points (opposite corners).

    :param point: The point to check.
    :type point: list
    :param rectangle: The rectangle to check.
    :type rectangle: list
    :return: Whether the point is within the rectangle.
    :rtype: bool
    """
    return (
        rectangle[0] <= point[0] <= rectangle[2]
        and rectangle[1] <= point[1] <= rectangle[3]
    )


def RotateAboutOrigin(vector, angle: AngularMeasurement):
    """Rotates a vector by an angle about the origin.

    :param vector: The vector to rotate.
    :type vector: list
    :param angle: The angle to rotate by.
    :type angle: :class:`AngularMeasurement`
    :return: The rotated vector.
    :rtype: list
    """
    Debug(f"Raw: {float(angle)} Angle: {angle}")
    rads = angle.to(angular.Radian)
    Debug(f"Rads: {rads}")
    x = vector[0] * math.cos(rads) - vector[1] * math.sin(rads)
    y = vector[0] * math.sin(rads) + vector[1] * math.cos(rads)
    return [x, y]


def AngleBetweenVectors(vector1, vector2):
    """Returns the angle between two vectors.

    :param vector1: The first vector.
    :type vector1: list
    :param vector2: The second vector.
    :type vector2: list
    :return: The angle between the two vectors.
    :rtype: float
    """
    ratio = Clamp(
        np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2)),
        -1,
        1,
    )
    return math.acos(ratio)
