def test_reflect_point_simple():
    import gamegine.utils.matematika as m
    import numpy as np
    point = np.array([1, 1])
    normal = np.array([0, 1])
    reflected = m.ReflectPoint(point, normal)
    assert reflected[0] == 1
    assert reflected[1] == -1

def test_reflect_1d_simple():
    import gamegine.utils.matematika as m
    reflected = m.ReflectValue1D(1, 0)
    assert reflected == -1

def test_reflect_1d_complex():
    import gamegine.utils.matematika as m
    reflected = m.ReflectValue1D(58, 60)
    assert reflected == 62

def test_reflect_over_line_simple():
    import gamegine.utils.matematika as m
    import numpy as np
    point = np.array([1, 1])
    line = np.array([0, 1])
    reflected = m.ReflectOverLine(point, line)
    assert reflected[0] == -1
    assert reflected[1] == 1

def test_rotate_about_origin_simple():
    import gamegine.utils.matematika as m
    import numpy as np
    import pint
    import gamegine.utils.unit as u

    point = [1, 0]
    angle = u.Degree(90)
    rotated = m.RotateAboutOrigin(point, angle)
    assert round(rotated[0], 3) == 0
    assert round(rotated[1], 3) == 1

