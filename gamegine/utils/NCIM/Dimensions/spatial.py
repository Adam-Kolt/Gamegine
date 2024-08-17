from gamegine.utils.NCIM.basic import Measurement, Unit
from gamegine.utils.NCIM.dimension import Dimension


class SpatialUnit(Unit):
    """A class representing a spatial unit of measurement, in addition to creating a :class:`SpatialMeasurement` of the same type when called. Inherits from the :class:`Unit` class.

    :param scale: The scale of the unit.
    :type scale: float, optional
    :param display: The display name of the unit.
    :type display: str, optional
    :param shift: The shift of the unit.
    :type shift: float, optional
    """

    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Spatial, scale, display, shift)

    def __call__(self, magnitude: float):
        """Creates a new :class:`SpatialMeasurement` with the given magnitude.

        :param magnitude: The magnitude of the measurement.
        :type magnitude: float
        :return: A new spatial measurement.
        :rtype: :class:`SpatialMeasurement`
        """

        return SpatialMeasurement(magnitude, self)


class SpatialMeasurement(Measurement):
    """A class representing a spatial measurement

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`SpatialUnit`
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __init__(
        self, magnitude: float, unit: SpatialUnit, base_magnitude=None
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Meter = SpatialUnit(10000, "m")
Centimeter = SpatialUnit(100, "cm")
Feet = SpatialUnit(3048, "ft")
Inch = SpatialUnit(254, "in")
Yard = SpatialUnit(9144, "yd")
Kilometer = SpatialUnit(10000000, "km")
Mile = SpatialUnit(16093400, "mi")
NauticalMile = SpatialUnit(18520000, "nmi")
MicroMeter = SpatialUnit(0.01, "μm")
