from gamegine.utils.NCIM.basic import Measurement, Unit
from gamegine.utils.NCIM.dimension import Dimension


# TODO: Technically, force is not an SI base units, but it is a derived unit from mass and acceleration.
class ForceUnit(Unit):
    """A class representing a force unit of measurement, in addition to creating a :class:`ForceMeasurement` of the same type when called. Inherits from the :class:`Unit` class.

    :param scale: The scale of the unit.
    :type scale: float, optional
    :param display: The display name of the unit.
    :type display: str, optional
    :param shift: The shift of the unit.
    :type shift: float, optional
    """

    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Force, scale, display, shift)

    def __call__(self, magnitude: float):
        """Creates a new class:`ForceMeasurement` with the given magnitude.

        :param magnitude: The magnitude of the measurement.
        :type magnitude: float
        :return: A new force measurement.
        :rtype: :class:`ForceMeasurement`
        """
        return ForceMeasurement(magnitude, self)


class ForceMeasurement(Measurement):
    """A class representing a force measurement. Inherits from the :class:`Measurement` class.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`ForceUnit`
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __init__(self, magnitude: float, unit: ForceUnit, base_magnitude=None) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Newton = ForceUnit(1, "N")
Dyne = ForceUnit(0.00001, "dyn")
PoundForce = ForceUnit(4.44822, "lbf")
KilogramForce = ForceUnit(9.80665, "kgf")
