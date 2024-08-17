from gamegine.utils.NCIM.basic import Measurement, Unit
from gamegine.utils.NCIM.dimension import Dimension


class AngularUnit(Unit):
    """A class representing an angular unit of measurement, in addition to creating a :class:`AngularMeasurement` of the same type when called. Inherits from the :class:`Unit` class.

    :param scale: The scale of the unit.
    :type scale: float, optional
    :param display: The display name of the unit.
    :type display: str, optional
    :param shift: The shift of the unit.
    :type shift: float, optional
    """

    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Angular, scale, display, shift)

    def __call__(self, magnitude: float):
        """Creates a new :class:`AngularMeasurement` with the given magnitude.

        :param magnitude: The magnitude of the measurement.
        :type magnitude: float
        :return: A new angular measurement.
        :rtype: :class:`AngularMeasurement`
        """
        return AngularMeasurement(magnitude, self)


class AngularMeasurement(Measurement):
    """A class representing an angular measurement. Inherits from the :class:`Measurement` class.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`AngularUnit`
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __init__(
        self, magnitude: float, unit: AngularUnit, base_magnitude=None
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Radian = AngularUnit(1, "rad")
Degree = AngularUnit(0.0174533, "°")
Grad = AngularUnit(0.01570796, "grad")
