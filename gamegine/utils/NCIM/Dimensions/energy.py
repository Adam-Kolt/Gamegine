from gamegine.utils.NCIM.basic import Measurement, Unit
from gamegine.utils.NCIM.dimension import Dimension


class EnergyUnit(Unit):
    """A class representing an energy unit of measurement, in addition to creating a :class:`EnergyMeasurement` of the same type when called. Inherits from the :class:`Unit` class.

    :param scale: The scale of the unit.
    :type scale: float, optional
    :param display: The display name of the unit.
    :type display: str, optional
    :param shift: The shift of the unit.
    :type shift: float, optional
    """

    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Energy, scale, display, shift)

    def __call__(self, magnitude: float):
        """Creates a new energy measurement with the given magnitude.

        :param magnitude: The magnitude of the measurement.
        :type magnitude: float
        :return: A new energy measurement.
        :rtype: :class:`EnergyMeasurement`
        """
        return EnergyMeasurement(magnitude, self)


class EnergyMeasurement(Measurement):
    """A class representing an :class:`EnergyMeasurement`. Inherits from the :class:`Measurement` class.

    :param magnitude: The magnitude of the measurement.
    :type magnitude: float
    :param unit: The unit of the measurement.
    :type unit: :class:`EnergyUnit`
    :param base_magnitude: The base magnitude of the measurement, used for internal creation. Defaults to None.
    :type base_magnitude: float, optional
    """

    def __init__(self, magnitude: float, unit: EnergyUnit, base_magnitude=None) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Joule = EnergyUnit(1, "J")
Kilojoule = EnergyUnit(1000, "kJ")
Calorie = EnergyUnit(4.184, "cal")
Kilocalorie = EnergyUnit(4184, "kcal")
WattHour = EnergyUnit(3600, "Wh")
KilowattHour = EnergyUnit(3600000, "kWh")
ElectronVolt = EnergyUnit(1.60218e-19, "eV")
