from gamegine.utils.NCIM.dimension import Dimension
from gamegine.utils.NCIM.basic import Measurement, Unit


class MassUnit(Unit):
    """A class representing a mass unit of measurement, in addition to creating a :class:`MassMeasurement` of the same type when called. Inherits from the :class:`Unit` class.

    :param scale: The scale of the unit.
    :type scale: float, optional
    :param display: The display name of the unit.
    :type display: str, optional
    :param shift: The shift of the unit.
    :type shift: float, optional
    """

    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Mass, scale, display, shift)

    def __call__(self, magnitude: float):
        """Creates a new :class:`MassMeasurement` with the given magnitude.

        :param magnitude: The magnitude of the measurement.
        :type magnitude: float
        :return: A new mass measurement
        :rtype: :class:`MassMeasurement`
        """

        return MassMeasurement(magnitude, self)


class MassMeasurement(Measurement):
    """A class representing a mass measurement

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`MassUnit`
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __init__(self, magnitude: float, unit: MassUnit, base_magnitude=None) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Gram = MassUnit(1, "g")
Kilogram = MassUnit(1000, "kg")
Milligram = MassUnit(0.001, "mg")
Pound = MassUnit(453.592, "lb")
Ounce = MassUnit(28.3495, "oz")
