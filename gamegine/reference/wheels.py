from dataclasses import dataclass

from gamegine.utils.NCIM.Dimensions.mass import Kilogram, MassMeasurement, Pound
from gamegine.utils.NCIM.Dimensions.spatial import Inch, SpatialMeasurement


@dataclass
class Tread:
    """Dataclass used to store the coefficient of friction of a wheel tread."""

    coefficientOfFriction: float
    sideCoF: float = 0.0

    def __post_init__(self):
        if self.coefficientOfFriction < 0:
            raise ValueError("Coefficient of friction must be non-negative.")

        if self.sideCoF == 0:
            self.sideCoF = self.coefficientOfFriction

    def __str__(self) -> str:
        return f"Tread(coefficientOfFriction={self.coefficientOfFriction})"

    def __repr__(self) -> str:
        return str(self)


@dataclass
class Wheel:
    """Dataclass used to store the diameter of a wheel."""

    diameter: SpatialMeasurement
    tread: Tread
    width: SpatialMeasurement = Inch(1)
    mass: MassMeasurement = Pound(0.3)

    def __post_init__(self):
        if self.diameter <= 0:
            raise ValueError("Wheel diameter must be positive.")

    def circumference(self) -> SpatialMeasurement:
        """Calculate the circumference of the wheel."""
        return self.diameter * 3.14159

    def grip(self) -> float:
        """Calculate the grip of the wheel."""
        return self.tread.coefficientOfFriction

    def __str__(self) -> str:
        return f"Wheel(diameter={self.diameter}, tread={self.tread})"

    def __repr__(self) -> str:
        return str(self)


class TreadDB:
    # VexPro wheels
    vexPro_traction_tires = Tread(1.1)
    vexPro_versawheel = Tread(1.2)
    vexPro_versawheel_DT = Tread(1.0)
    vexPro_mecanum_wheels = Tread(1.0)
    vexPro_omni_wheels = Tread(1.1)

    # Colson Performa
    colson_performa_wheels = Tread(1.0)

    # AM wheels
    am_pebbletop_tread = Tread(1.24)
    am_blue_nitrile_roughtop_tread = Tread(1.19)
    am_8_pneumatic_wheel = Tread(1.27, 0.93)
    am_hiGrip_wheels_blue = Tread(0.97)
    am_hiGrip_wheels_black = Tread(0.77)
    am_hiGrip_wheels_white = Tread(1.0, 0.95)

    # AM FIRST wheels
    am_2008_FIRST_wheels = Tread(0.9)
    am_2010_FIRST_wheels = Tread(1.0)

    # AM Stealth wheels
    am_stealth_wheels_grey = Tread(0.92)
    am_stealth_wheels_black = Tread(0.87)
    am_stealth_wheels_blue = Tread(1.14)

    # AM Mecanum wheels (all sizes)
    am_mecanum_wheels = Tread(0.7, 0.6)

    # AM Dualie Omni wheels
    am_dualie_omni_wheels = Tread(1.0, 0.2)
