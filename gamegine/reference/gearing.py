from dataclasses import dataclass
from enum import Enum


@dataclass
class Gear:
    """Dataclass used to store the number of teeth on a gear."""

    teeth: int

    def __add__(self, other: "Gear") -> "GearSeries":
        """Adds two Gear objects together, returning a GearRatio object with the ratio of the two gears.

        :param other: The other Gear object to add.
        :type other: Gear
        :return: A new GearRatio object.
        :rtype: GearRatio
        """
        return GearSeries(self, other)

    def get_input_gear(self) -> "Gear":
        """Returns the input gear of the gear system."""
        return self

    def get_output_gear(self) -> "Gear":
        """Returns the output gear of the gear system."""
        return self

    def get_ratio(self):
        return 1

    def __div__(self, other):
        return self.teeth / other.teeth

    def __rdiv__(self, other):
        return other.teeth / self.teeth

    def __truediv__(self, other):
        return self.teeth / other.teeth

    def __rtruediv__(self, other):
        return other.teeth / self.teeth

    def __mul__(self, other):
        return self.teeth * other.teeth

    def __rmul__(self, other):
        return self.teeth * other.teeth


class GearSeries:
    """Class used to store the gear ratio of a gear system. Used in order to create compound gears. To apply the gear ratio to a value, multiply or divide the value by the GearSeries object.

    :param input_gear: The input gear of the gear system.
    :type input_gear: Gear
    :param output_gear: The output gear of the gear system.
    :type output_gear: Gear
    :param ratio: The gear ratio of the gear system.
    :type ratio: float
    """

    def __init__(self, input_gear: Gear, output_gear: Gear, ratio=None) -> None:
        self.input_gear = input_gear
        self.output_gear = output_gear
        if ratio is not None:
            self.ratio = ratio
        else:
            self.ratio = output_gear / input_gear

    def get_input_gear(self) -> Gear:
        """Returns the input gear of the gear system.

        :return: The input gear of the gear system.
        :rtype: Gear
        """
        return self.input_gear

    def get_output_gear(self) -> Gear:
        """Returns the output gear of the gear system.

        :return: The output gear of the gear system.
        :rtype: Gear
        """
        return self.output_gear

    def get_ratio(self) -> float:
        """Returns the gear ratio of the gear system.

        :return: The gear ratio of the gear system.
        :rtype: float
        """
        return self.ratio

    def __mul__(self, other):
        """Multiplies the object by the gear ratio of the gear system."""
        return other / self.get_ratio()

    def __rmul__(self, other):
        """Multiplies the object by the gear ratio of the gear system."""
        return other / self.get_ratio()

    def __truediv__(self, other):
        """Divides the object by the gear ratio of the gear system."""
        return other * self.get_ratio()

    def __rtruediv__(self, other):
        """Divides the object by the gear ratio of the gear system."""
        return other * self.get_ratio()

    def __add__(self, other) -> "GearSeries":
        """Adds the object by the gear ratio of the gear system."""

        if isinstance(other, GearSeries):
            ratio = self.get_ratio() * other.get_ratio()
        else:
            ratio = (
                other.get_input_gear()
                / self.get_output_gear()
                * self.get_ratio()
                * other.get_ratio()
            )

        return GearSeries(self.get_input_gear(), other.get_output_gear(), ratio)

    def forward_input(self, other):
        return self * other

    def reverse_input(self, other):
        return other / self


class MK4I:
    """Class for storing the gear ratios of the MK4I Swerve."""

    __STAGE1A = Gear(14) + Gear(50)
    __STAGE1B = Gear(16) + Gear(50)
    __STAGE2A = Gear(25) + Gear(19)
    __STAGE2B = Gear(27) + Gear(17)
    __STAGE2C = Gear(28) + Gear(16)
    __STAGE3A = Gear(15) + Gear(45)
    L1 = __STAGE1A + __STAGE2A + __STAGE3A
    L2 = __STAGE1A + __STAGE2B + __STAGE3A
    L3 = __STAGE1A + __STAGE2C + __STAGE3A

class ThriftySwerve:
    """Class for storing the gear ratios of the Thrifty Swerve."""

    __BASE_STAGE = GearSeries(None, None, ratio=4.5)
    

    class Pinions(Enum):
        """Enum for the pinion gears of the Thrifty Swerve."""
        P12 = Gear(12)
        P13 = Gear(13)
        P14 = Gear(14)
    
    class Stage2(Enum):
        """Enum for the stage 2 gears of the Thrifty Swerve."""
        S18 = Gear(18)
        S16 = Gear(16)

    @classmethod
    def get_gear_ratio(cls, pinion: "ThriftySwerve.Pinions", stage2: "ThriftySwerve.Stage2") -> GearSeries:
        """Returns the gear ratio of the Thrifty Swerve for the given pinion and stage 2 gears.

        :param pinion: The pinion gear to use.
        :type pinion: ThriftySwerve.Pinions
        :param stage2: The stage 2 gear to use.
        :type stage2: ThriftySwerve.Stage2
        :return: The gear ratio of the Thrifty Swerve.
        :rtype: GearSeries
        """
      
        return cls.__BASE_STAGE + (pinion.value + stage2.value)
      