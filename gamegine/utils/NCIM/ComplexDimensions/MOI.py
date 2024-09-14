from gamegine.utils.NCIM.Dimensions.mass import Kilogram, MassUnit, Pound
from gamegine.utils.NCIM.Dimensions.spatial import Inch, Meter, SpatialUnit
from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit


class MOIUnit(ComplexUnit):
    """A class representing a moment of inertia complex unit, composed of a spatial and mass element in the form spatial^2 * mass^1, in addition to creating a :class:`MOI` of the same type when called. Inherits from the :class:`ComplexUnit` class.

    :param mass: The mass unit of the
    :type mass: :class:`MassUnit`
    :param spatial: The spatial unit of the
    :type spatial: :class:`SpatialUnit
    """

    def __init__(self, mass: MassUnit, spatial: SpatialUnit) -> None:
        super().__init__({spatial: 2, mass: 1})

    def __call__(self, magnitude: float):
        """Creates a new :class:`MOI` with the given magnitude.

        :param magnitude: The magnitude of the measurement.
        :type magnitude: float
        :return: A new moment of inertia measurement.
        :rtype: :class:`MOI`
        """
        return MOI(magnitude, self)


class MOI(ComplexMeasurement):
    """A class representing a moment of inertia complex measurement, composed of a spatial and mass element in the form spatial^2 * mass^1. Inherits from the :class:`ComplexMeasurement` class.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`MOIUnit`
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __new__(
        cls,
        magnitude: float,
        unit: MOIUnit,
        base_magnitude=None,
    ):
        return ComplexMeasurement.__new__(cls, magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: MOIUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


KilogramMetersSquared = MOIUnit(Kilogram, Meter)
PoundsInchesSquared = MOIUnit(Pound, Inch)
