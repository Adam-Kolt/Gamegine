from enum import Enum


class Dimension(Enum):
    """Enum for the different dimensions of the NCIM system"""

    Spatial = 0
    Mass = 1
    Force = 2
    Energy = 3
    Temporal = 4
    Angular = 5
