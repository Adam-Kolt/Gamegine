from gamegine.utils.NCIM.Dimensions.spatial import Kilometer, Meter, SpatialUnit
from gamegine.utils.NCIM.Dimensions.temporal import Hour, Second, TemporalUnit
from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit


class AccelerationUnit(ComplexUnit):
    """A class representing an acceleration complex unit, composed of a spatial and temporal element in the form spatial^1 / temporal^2, in addition to creating a :class:`Acceleration` of the same type when called. Inherits from the :class:`ComplexUnit` class.

    :param spatial: The spatial unit of the acceleration.
    :type spatial: :class:`SpatialUnit`
    :param time: The temporal unit of the acceleration.
    :type time: :class:`TemporalUnit`
    """

    def __init__(self, spatial: SpatialUnit, time: TemporalUnit) -> None:
        super().__init__({spatial: 1, time: -2})

    def __call__(self, magnitude: float):
        """Creates a new :class:`Acceleration` with the given magnitude.

        :param magnitude: The magnitude of the measurement.
        :type magnitude: float
        :return: A new acceleration measurement.
        :rtype: :class:`Acceleration`
        """
        return Acceleration(magnitude, self)


class Acceleration(ComplexMeasurement):
    """A class representing an acceleration complex measurement, composed of a spatial and temporal element in the form spatial^1 / temporal^2. Inherits from the :class:`ComplexMeasurement` class.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`AccelerationUnit`
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __new__(
        cls,
        magnitude: float,
        unit: AccelerationUnit,
        base_magnitude=None,
    ):
        return ComplexMeasurement.__new__(cls, magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: AccelerationUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


MeterPerSecondSquared = AccelerationUnit(Meter, Second)
KilometerPerHourSquared = AccelerationUnit(Kilometer, Hour)
