from gamegine.utils.NCIM.Dimensions.current import Ampere, CurrentUnit
from gamegine.utils.NCIM.Dimensions.mass import Kilogram, MassUnit
from gamegine.utils.NCIM.Dimensions.spatial import Meter, SpatialUnit
from gamegine.utils.NCIM.Dimensions.temporal import Second, TemporalUnit
from gamegine.utils.NCIM.basic import ComplexMeasurement, ComplexUnit


class ElectricPotUnit(ComplexUnit):
    """A class representing an electric potential complex unit, composed kg m^2 / A s^3, in addition to creating a :class:`ElectricPot` of the same type when called. Inherits from the :class:`ComplexUnit` class.

    :param mass: The mass unit of the electric potential.
    :type mass: :class:`MassUnit`
    :param spatial: The spatial unit of the electric potential.
    :type spatial: :class:`SpatialUnit`
    :param current: The current unit of the electric potential.
    :type current: :class:`CurrentUnit`
    :param time: The temporal unit of the electric potential.
    :type time: :class:`TemporalUnit`
    """

    def __init__(
        self,
        mass: MassUnit,
        spatial: SpatialUnit,
        current: CurrentUnit,
        time: TemporalUnit,
    ) -> None:
        super().__init__({mass: 1, spatial: 2, current: -1, time: -3})

    def __call__(self, magnitude: float):
        return ElectricPot(magnitude, self)


class ElectricPot(ComplexMeasurement):
    """A class representing an electric potential complex measurement, composed of kg m^2 / A s^3. Inherits from the :class:`ComplexMeasurement` class.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`ElectricPot
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __new__(
        cls,
        magnitude: float,
        unit: ElectricPotUnit,
        base_magnitude=None,
    ):
        return ComplexMeasurement.__new__(cls, magnitude, unit, base_magnitude)

    def __init__(
        self,
        magnitude: float,
        unit: ElectricPotUnit,
        base_magnitude=None,
    ) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Volt = ElectricPotUnit(Kilogram, Meter, Ampere, Second)
