import pytest


def test_create_units():
    from gamegine.utils.NCIM.ncim import Feet, Inch, Meter, Centimeter
    from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement

    assert isinstance(Feet(1), SpatialMeasurement)
    assert isinstance(Inch(1), SpatialMeasurement)
    assert isinstance(Meter(1), SpatialMeasurement)
    assert isinstance(Centimeter(1), SpatialMeasurement)


def test_add_units():
    from gamegine.utils.NCIM.ncim import Feet, Inch, Meter, Centimeter, Yard

    assert Feet(1) + Feet(2) == Yard(1)
    assert Inch(1) + Inch(1) == Inch(2)
    assert Meter(1) + Meter(1) == Meter(2)
    assert Centimeter(99) + Centimeter(1) == Meter(1)


def test_sub_units():
    from gamegine.utils.NCIM.ncim import Feet, Inch, Meter, Centimeter, Yard

    assert Yard(1) - Feet(2) == Feet(1)
    assert Inch(2) - Inch(1) == Inch(1)
    assert Meter(2) - Meter(1) == Meter(1)
    assert Meter(1) - Centimeter(99) == Centimeter(1)


def test_mul_units():
    from gamegine.utils.NCIM.ncim import Feet, Inch, Meter, Centimeter

    assert Feet(2) * 2 == Feet(4)
    assert Inch(2) * 2 == Inch(4)
    assert Meter(2) * 2 == Meter(4)
    assert Centimeter(2) * 2 == Centimeter(4)


def test_div_units():
    from gamegine.utils.NCIM.ncim import Feet, Inch, Meter, Centimeter

    assert Feet(4) / 2 == Feet(2)
    assert Inch(4) / 2 == Inch(2)
    assert Meter(4) / 2 == Meter(2)
    assert Centimeter(4) / 2 == Centimeter(2)
    assert Feet(1) / Inch(1) == 12


def test_complex_measurement_creation():
    pass
