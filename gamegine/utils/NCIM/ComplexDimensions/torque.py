from gamegine.utils.NCIM.Dimensions.force import ForceUnit, Newton
from gamegine.utils.NCIM.Dimensions.spatial import Meter, SpatialUnit
from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit


class TorqueUnit(ComplexUnit):
    """A class representing a torque complex unit, composed of a spatial and force element in the form spatial^1 * force^1, in addition to creating a :class:`Torque` of the same type when called. Inherits from the :class:`ComplexUnit` class.

    :param spatial: The spatial unit of the torque.
    :type spatial: :class:`SpatialUnit`
    :param force: The force unit of the torque.
    :type force: :class:`ForceUnit`
    """

    def __init__(self, spatial: SpatialUnit, force: ForceUnit) -> None:
        super().__init__({spatial: 1, force: 1})

    def __call__(self, magnitude: float):
        """Creates a new :class:`Torque` with the given magnitude.

        :param magnitude: The magnitude of the measurement.
        :type magnitude: float
        :return: A new torque measurement.
        :rtype: :class:`Torque`
        """
        return Torque(magnitude, self)


class Torque(ComplexMeasurement):
    """A class representing a torque complex measurement, composed of a spatial and force element in the form spatial^1 * force^1. Inherits from the :class:`ComplexMeasurement` class.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`TorqueUnit`
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __new__(
        cls,
        magnitude: float,
        unit: TorqueUnit,
        base_magnitude=None,
    ):
        return ComplexMeasurement.__new__(cls, magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: TorqueUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


NewtonMeter = TorqueUnit(Meter, Newton)
