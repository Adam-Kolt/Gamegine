import pytest
from pytest import approx

def test_create_units():
    from gamegine.utils.NCIM.ncim import Feet, Inch, Meter, Centimeter
    from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement

    assert isinstance(Feet(1), SpatialMeasurement)
    assert isinstance(Inch(1), SpatialMeasurement)
    assert isinstance(Meter(1), SpatialMeasurement)
    assert isinstance(Centimeter(1), SpatialMeasurement)


def test_add_units():
    from gamegine.utils.NCIM.ncim import Feet, Inch, Meter, Centimeter, Yard

    assert float(Feet(1) + Feet(2)) == approx(float(Yard(1)))
    assert float(Inch(1) + Inch(1)) == approx(float(Inch(2)))
    assert float(Meter(1) + Meter(1)) == approx(float(Meter(2)))
    assert float(Centimeter(99) + Centimeter(1)) == approx(float(Meter(1)))


def test_sub_units():
    from gamegine.utils.NCIM.ncim import Feet, Inch, Meter, Centimeter, Yard

    assert float(Yard(1) - Feet(2)) == approx(float(Feet(1)))
    assert float(Inch(2) - Inch(1)) == approx(float(Inch(1)))
    assert float(Meter(2) - Meter(1)) == approx(float(Meter(1)))
    assert float(Meter(1) - Centimeter(99)) == approx(float(Centimeter(1)))


def test_mul_units():
    from gamegine.utils.NCIM.ncim import Feet, Inch, Meter, Centimeter

    assert float(Feet(2) * 2) == approx(float(Feet(4)))
    assert float(Inch(2) * 2) == approx(float(Inch(4)))
    assert float(Meter(2) * 2) == approx(float(Meter(4)))
    assert float(Centimeter(2) * 2) == approx(float(Centimeter(4)))


def test_div_units():
    from gamegine.utils.NCIM.ncim import Feet, Inch, Meter, Centimeter

    assert float(Feet(4) / 2) == approx(float(Feet(2)))
    assert float(Inch(4) / 2) == approx(float(Inch(2)))
    assert float(Meter(4) / 2) == approx(float(Meter(2)))
    assert float(Centimeter(4) / 2) == approx(float(Centimeter(2)))
    
    # Division of measurements results in scalar (or dimensionless measurement)
    # Feet(1) / Inch(1) -> 0.3048 / 0.0254 = 12.0
    val = Feet(1) / Inch(1)
    if hasattr(val, 'unit'): # If it returns Measurement
        assert float(val) == approx(12.0)
    else:
        assert val == approx(12.0)


def test_complex_measurement_creation():
    pass
