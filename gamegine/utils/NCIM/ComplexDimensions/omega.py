from gamegine.utils.NCIM.Dimensions.angular import AngularUnit, Degree, Radian, Rotation
from gamegine.utils.NCIM.Dimensions.temporal import Second, TemporalUnit
from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit


class OmegaUnit(ComplexUnit):
    """A class representing an angular velocity complex unit, composed of an angular and temporal element in the form angular^1 / temporal^1, in addition to creating a :class:`Omega` of the same type when called. Inherits from the :class:`ComplexUnit` class.

    :param angles: The angular unit of the angular velocity.
    :type angles: :class:`AngularUnit`
    :param time: The temporal unit of the angular velocity.
    :type time: :class:`TemporalUnit`
    """

    def __init__(self, angles: AngularUnit, time: TemporalUnit) -> None:
        super().__init__({angles: 1, time: -1})

    def __call__(self, magnitude: float):
        """Creates a new :class:`Omega` object with the given magnitude and this unit.

        :param magnitude: The magnitude of the measurement.
        :type magnitude: float
        :return: A new :class:`Omega` object.
        :rtype: :class:`Omega`
        """
        return Omega(magnitude, self)


class Omega(ComplexMeasurement):
    """A class representing an angular velocity complex measurement, composed of an angular and temporal element in the form angular^1 / temporal^1. Inherits from the :class:`ComplexMeasurement` class.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`OmegaUnit`
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __new__(
        cls,
        magnitude: float,
        unit: OmegaUnit,
        base_magnitude=None,
    ):
        return ComplexMeasurement.__new__(cls, magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: OmegaUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


RadiansPerSecond = OmegaUnit(Radian, Second)
DegreesPerSecond = OmegaUnit(Degree, Second)
RotationsPerSecond = OmegaUnit(Rotation, Second)
