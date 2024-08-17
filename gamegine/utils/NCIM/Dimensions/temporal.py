from gamegine.utils.NCIM.basic import Measurement, Unit
from gamegine.utils.NCIM.dimension import Dimension


class TemporalUnit(Unit):
    """A class representing a temporal unit of measurement, in addition to creating a :class:`TemporalMeasurement` of the same type when called. Inherits from the :class:`Unit` class.

    :param scale: The scale of the unit.
    :type scale: float, optional
    :param display: The display name of the unit.
    :type display: str, optional
    :param shift: The shift of the unit.
    :type shift: float, optional
    """

    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Temporal, scale, display, shift)

    def __call__(self, magnitude: float):
        """Creates a new :class:`TemporalMeasurement` with the given magnitude.

        :param magnitude: The magnitude of the measurement.
        :type magnitude: float
        :return: A new temporal measurement.
        :rtype: :class:`TemporalMeasurement`
        """
        return TemporalMeasurement(magnitude, self)


class TemporalMeasurement(Measurement):
    """A class representing a temporal measurement. Inherits from the :class:`Measurement` class.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`TemporalUnit`
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __init__(
        self, magnitude: float, unit: TemporalUnit, base_magnitude=None
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Second = TemporalUnit(1, "s")
Minute = TemporalUnit(60, "min")
Hour = TemporalUnit(3600, "h")
Day = TemporalUnit(86400, "d")
Week = TemporalUnit(604800, "wk")
Month = TemporalUnit(2628000, "mo")
Year = TemporalUnit(31536000, "yr")
