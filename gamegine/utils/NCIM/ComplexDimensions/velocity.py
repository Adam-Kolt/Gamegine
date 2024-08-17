from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit
from gamegine.utils.NCIM.Dimensions.spatial import (
    Feet,
    Kilometer,
    Meter,
    Mile,
    SpatialUnit,
)
from gamegine.utils.NCIM.Dimensions.temporal import Hour, Second, TemporalUnit


class VelocityUnit(ComplexUnit):
    """A class representing a velocity complex unit, composed of a spatial and temporal element in the form spatial^1 / temporal^1, in addition to creating a :class:`Velocity` of the same type when called. Inherits from the :class:`ComplexUnit` class.

    :param spatial: The spatial unit of the velocity.
    :type spatial: :class:`SpatialUnit`
    :param time: The temporal unit of the velocity.
    :type time: :class:`TemporalUnit`
    """

    def __init__(self, spatial: SpatialUnit, time: TemporalUnit) -> None:
        super().__init__({spatial: 1, time: -1})

    def __call__(self, magnitude: float):
        """Creates a new :class:`Velocity` with the given magnitude.

        :param magnitude: The magnitude of the measurement.
        :type magnitude: float
        :return: A new velocity measurement.
        :rtype: :class:`Velocity`
        """
        return Velocity(magnitude, self)


class Velocity(ComplexMeasurement):
    """A class representing a velocity complex measurement, composed of a spatial and temporal element in the form spatial^1 / temporal^1. Inherits from the :class:`ComplexMeasurement` class.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`VelocityUnit`
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __new__(
        cls,
        magnitude: float,
        unit: VelocityUnit,
        base_magnitude=None,
    ):
        return ComplexMeasurement.__new__(cls, magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: VelocityUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


MetersPerSecond = VelocityUnit(Meter, Second)
KilometersPerHour = VelocityUnit(Kilometer, Hour)
MilesPerHour = VelocityUnit(Mile, Hour)
FeetPerSecond = VelocityUnit(Feet, Second)
