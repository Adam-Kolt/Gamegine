from dataclasses import dataclass


@dataclass
class Gear:
    """Dataclass used to store the number of teeth on a gear."""

    teeth: int

    def __add__(self, other: "Gear") -> "GearRatio":
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

    def ratio(self):
        return 1


# TODO: Fix to actually make it make sense
class GearSeries:
    """Dataclass used to store the gear ratio of a gear system."""

    def __init__(self, input_gear: Gear, output_gear: Gear) -> None:
        self.input_gear = input_gear
        self.output_gear = output_gear
        self.ratio = output_gear / input_gear

    def get_input_gear(self):
        return self.input_gear

    def get_output_gear(self):
        return self.output_gear

    def ratio(self):
        return self.output_gear / self.input_gear

    def __mul__(self, other):
        """Multiplies the object by the gear ratio of the gear system."""
        return other * self.ratio()

    def __rmul__(self, other):
        """Multiplies the object by the gear ratio of the gear system."""
        return other * self.ratio()

    def __truediv__(self, other):
        """Divides the object by the gear ratio of the gear system."""
        return other / self.ratio()

    def __rtruediv__(self, other):
        """Divides the object by the gear ratio of the gear system."""
        return other / self.ratio()

    def __add__(self, other):
        """Adds the object by the gear ratio of the gear system."""
        if isinstance(other, GearSeries):
            self.ratio *= other.ratio()
            self.output_gear = other.get_output_gear()
            return self
        self.ratio = (
            other.get_input_gear()
            / self.get_output_gear()
            * self.ratio()
            * other.ratio()
        )
        self.output_gear = other.get_output_gear()
        return self


class MK4:
    L1 = (Gear(14) + Gear(50)) + (Gear(25) + Gear(19)) + (Gear(15) + Gear(45))
