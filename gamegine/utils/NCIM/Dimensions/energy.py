from gamegine.utils.NCIM.basic import Measurement, Unit
from gamegine.utils.NCIM.dimension import Dimension


class EnergyUnit(Unit):
    def __init__(self, scale: float = 0, display="", shift: float = 0) -> None:
        super().__init__(Dimension.Energy, scale, display, shift)

    def __call__(self, magnitude: float):
        return EnergyMeasurement(magnitude, self)


class EnergyMeasurement(Measurement):
    def __init__(self, magnitude: float, unit: EnergyUnit, base_magnitude=None) -> None:
        super().__init__(magnitude, unit, base_magnitude)


Joule = EnergyUnit(1, "J")
Kilojoule = EnergyUnit(1000, "kJ")
Calorie = EnergyUnit(4.184, "cal")
Kilocalorie = EnergyUnit(4184, "kcal")
WattHour = EnergyUnit(3600, "Wh")
KilowattHour = EnergyUnit(3600000, "kWh")
ElectronVolt = EnergyUnit(1.60218e-19, "eV")
