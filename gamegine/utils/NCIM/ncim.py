# Import Units for Easy Access
from enum import Enum
from gamegine.utils.NCIM.Dimensions.spatial import *
from gamegine.utils.NCIM.Dimensions.temporal import *
from gamegine.utils.NCIM.Dimensions.angular import *
from gamegine.utils.NCIM.Dimensions.mass import *
from gamegine.utils.NCIM.Dimensions.force import *
from gamegine.utils.NCIM.ComplexDimensions.torque import *
from gamegine.utils.NCIM.ComplexDimensions.velocity import *
from gamegine.utils.NCIM.ComplexDimensions.acceleration import *
from gamegine.utils.NCIM.ComplexDimensions.MOI import *
from gamegine.utils.NCIM.ComplexDimensions.omega import *
from gamegine.utils.NCIM.basic import *


def RatioOf(a, b):
    """Returns the ratio of two values. Primarily for backwards compatibility with functions which already use this function.

    :param a: The numerator of the ratio.
    :type a: float
    :param b: The denominator of the ratio.
    :type b: float
    :return: The ratio of the two values.
    :rtype: float
    """
    return a / b


def Zero():
    """Returns a zero spatial measurement.

    :return: A zero value of the default dimension.
    :rtype: :class:`Dimension`
    """
    return Meter(0)
