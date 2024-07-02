
def test_create_units_magnitude():
    from gamegine.utils.unit import Meter, Centimeter, Feet, Inch, Second, Radian, Degree, GetRegistry
    assert Meter(1).magnitude == 1
    assert Centimeter(1).magnitude == 1
    assert Feet(1).magnitude == 1
    assert Inch(1).magnitude == 1
    assert Second(1).magnitude == 1
    assert Radian(1).magnitude == 1
    assert Degree(1).magnitude == 1

def test_create_units_unit():
    from gamegine.utils.unit import Meter, Centimeter, Feet, Inch, Second, Radian, Degree, GetRegistry
    assert Meter(1).units == GetRegistry().meter
    assert Centimeter(1).units == GetRegistry().centimeter
    assert Feet(1).units == GetRegistry().foot
    assert Inch(1).units == GetRegistry().inch
    assert Second(1).units == GetRegistry().second
    assert Radian(1).units == GetRegistry().radian
    assert Degree(1).units == GetRegistry().degree

def test_convert_to_standard_magnitude():
    from gamegine.utils.unit import StdMag, Meter, Centimeter, Feet, Inch, Second, Radian, Degree, GetRegistry
    from gamegine import Q_
    assert StdMag(Meter(1)) == Q_(1, GetRegistry().meter).to_base_units().magnitude
    assert StdMag(Centimeter(1)) == Q_(1, GetRegistry().centimeter).to_base_units().magnitude
    assert StdMag(Feet(1)) == Q_(1, GetRegistry().foot).to_base_units().magnitude
    assert StdMag(Inch(1)) == Q_(1, GetRegistry().inch).to_base_units().magnitude



def test_convert_magnitude_to_standard():
    from gamegine.utils.unit import ToStd, Meter, Centimeter, Feet, Inch, Second, Radian, Degree, GetRegistry, StandardUnit
    from gamegine import Q_
    assert ToStd(1) == Q_(1, StandardUnit)
    assert ToStd(1).magnitude == 1
    
def test_convert_list_to_standard():
    from gamegine.utils.unit import List2Std, Meter, Centimeter, Feet, Inch, Second, Radian, Degree, GetRegistry, StandardUnit
    from gamegine import Q_
    assert List2Std([1, 2, 3]) == [Q_(1, StandardUnit), Q_(2, StandardUnit), Q_(3, StandardUnit)]
    assert List2Std([1, 2, 3])[0].magnitude == 1

def test_convert_tuple_to_standard():
    from gamegine.utils.unit import Tuple2Std, Meter, Centimeter, Feet, Inch, Second, Radian, Degree, GetRegistry, StandardUnit
    from gamegine import Q_
    assert Tuple2Std((1, 2, 3)) == (Q_(1, StandardUnit), Q_(2, StandardUnit), Q_(3, StandardUnit))
    assert Tuple2Std((1, 2, 3))[0].magnitude == 1
    