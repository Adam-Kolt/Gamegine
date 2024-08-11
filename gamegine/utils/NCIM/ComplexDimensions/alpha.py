from gamegine.utils.NCIM.Dimensions.angular import AngularUnit
from gamegine.utils.NCIM.Dimensions.temporal import TemporalUnit
from gamegine.utils.NCIM.complex import ComplexUnit


class AlphaUnit(ComplexUnit):
    def __init__(self, angles: AngularUnit, time: TemporalUnit) -> None:
        super().__init__({angles: 1, time: -2})
