
import pytest
import numpy as np
from gamegine.utils.NCIM.core import Unit, Measurement, Dimension

# Helper to create dummy units
def create_unit_params(dims, scale=1.0, symbol="", offset=0.0):
    return dims, scale, symbol, offset

@pytest.fixture
def meter_unit():
    dims = np.zeros(len(Dimension))
    dims[Dimension.Spatial.value] = 1
    return Unit(dims, 1.0, "m")

@pytest.fixture
def second_unit():
    dims = np.zeros(len(Dimension))
    dims[Dimension.Temporal.value] = 1
    return Unit(dims, 1.0, "s")

@pytest.fixture
def velocity_unit(meter_unit, second_unit):
    return meter_unit / second_unit

class TestUnit:
    def test_initialization(self, meter_unit):
        assert meter_unit.scale == 1.0
        assert meter_unit.symbol == "m"
        assert meter_unit.offset == 0.0
        assert meter_unit.dimensions[Dimension.Spatial.value] == 1

    def test_equality(self, meter_unit):
        dims = np.zeros(len(Dimension))
        dims[Dimension.Spatial.value] = 1
        other_meter = Unit(dims, 1.0, "m")
        assert meter_unit == other_meter
        
        # Different scale
        inch = Unit(dims, 0.0254, "in")
        assert meter_unit != inch

    def test_multiplication(self, meter_unit, second_unit):
        ms = meter_unit * second_unit
        assert ms.dimensions[Dimension.Spatial.value] == 1
        assert ms.dimensions[Dimension.Temporal.value] == 1
        assert ms.symbol == "m*s"

    def test_division(self, meter_unit, second_unit):
        ms_inv = meter_unit / second_unit
        assert ms_inv.dimensions[Dimension.Spatial.value] == 1
        assert ms_inv.dimensions[Dimension.Temporal.value] == -1
        assert ms_inv.symbol == "m/s"

    def test_power(self, meter_unit):
        m2 = meter_unit ** 2
        assert m2.dimensions[Dimension.Spatial.value] == 2
        assert m2.symbol == "m^2"

class TestMeasurement:
    def test_initialization(self, meter_unit):
        m = Measurement(10, meter_unit)
        assert float(m) == 10.0 # Base value
        assert m.unit == meter_unit

    def test_conversion(self, meter_unit):
        # Create km unit
        dims = np.zeros(len(Dimension))
        dims[Dimension.Spatial.value] = 1
        km_unit = Unit(dims, 1000.0, "km")
        
        m = Measurement(1, km_unit) # 1 km
        assert float(m) == 1000.0 # 1000 meters base
        
        m_as_meters = m.to(meter_unit)
        assert float(m_as_meters) == 1000.0
        
        # Check string representation (which uses from_base)
        assert "1.0000 km" in str(m)
        assert "1000.0000 m" in str(m_as_meters)

    def test_add_same_unit(self, meter_unit):
        m1 = Measurement(10, meter_unit)
        m2 = Measurement(5, meter_unit)
        m3 = m1 + m2
        assert float(m3) == 15.0
        assert m3.unit == meter_unit

    def test_add_compatible_unit(self, meter_unit):
        dims = np.zeros(len(Dimension))
        dims[Dimension.Spatial.value] = 1
        km_unit = Unit(dims, 1000.0, "km")
        
        m1 = Measurement(1000, meter_unit) # 1000 m
        m2 = Measurement(1, km_unit) # 1 km = 1000 m
        
        m3 = m1 + m2
        assert float(m3) == 2000.0
        assert m3.unit == meter_unit

    def test_sub(self, meter_unit):
        m1 = Measurement(10, meter_unit)
        m2 = Measurement(5, meter_unit)
        m3 = m1 - m2
        assert float(m3) == 5.0

    def test_mul_scalar(self, meter_unit):
        m1 = Measurement(10, meter_unit)
        m2 = m1 * 2
        assert float(m2) == 20.0
        assert m2.unit == meter_unit
        
        m3 = 3 * m1
        assert float(m3) == 30.0

    def test_mul_measurement(self, meter_unit, second_unit):
        m = Measurement(10, meter_unit)
        s = Measurement(2, second_unit)
        
        # 10m * 2s = 20 m*s
        res = m * s
        assert float(res) == 20.0
        assert res.unit.dimensions[Dimension.Spatial.value] == 1
        assert res.unit.dimensions[Dimension.Temporal.value] == 1

    def test_div_measurement(self, meter_unit, second_unit):
        m = Measurement(10, meter_unit)
        s = Measurement(2, second_unit)
        
        # 10m / 2s = 5 m/s
        res = m / s
        assert float(res) == 5.0
        assert res.unit.dimensions[Dimension.Spatial.value] == 1
        assert res.unit.dimensions[Dimension.Temporal.value] == -1

    def test_incompatible_addition(self, meter_unit, second_unit):
        m = Measurement(1, meter_unit)
        s = Measurement(1, second_unit)
        with pytest.raises(ValueError):
            _ = m + s
