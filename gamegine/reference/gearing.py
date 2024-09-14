from dataclasses import dataclass


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

    def get_input_gear(self):
        return self

    def get_output_gear(self):
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


# TODO: Fix to actually make it make sense
class GearSeries:
    """Dataclass used to store the gear ratio of a gear system."""

    def __init__(self, input_gear: Gear, output_gear: Gear, ratio=None) -> None:
        self.input_gear = input_gear
        self.output_gear = output_gear
        if ratio is not None:
            self.ratio = ratio
        else:
            self.ratio = output_gear / input_gear

    def get_input_gear(self):
        return self.input_gear

    def get_output_gear(self):
        return self.output_gear

    def get_ratio(self) -> float:
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
    __STAGE1A = Gear(14) + Gear(50)
    __STAGE1B = Gear(16) + Gear(50)
    __STAGE2A = Gear(25) + Gear(19)
    __STAGE2B = Gear(27) + Gear(17)
    __STAGE2C = Gear(28) + Gear(16)
    __STAGE3A = Gear(15) + Gear(45)
    L1 = __STAGE1A + __STAGE2A + __STAGE3A
    L2 = __STAGE1A + __STAGE2B + __STAGE3A
    L3 = __STAGE1A + __STAGE2C + __STAGE3A
