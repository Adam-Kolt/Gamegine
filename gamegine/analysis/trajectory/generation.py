from abc import ABC, abstractmethod
from typing import List, Tuple
from dataclasses import dataclass
import pint

from gamegine.render.drawable import Drawable
from gamegine.representation.bounds import DiscreteBoundary
from gamegine.utils.unit import SpatialMeasurement

@dataclass
class TrajectoryParameters: # All values are in m/s, m/s^2, m/s^3,
    max_speed: float 
    max_acceleration: float
    max_jerk: float
    max_curvature: float = 0.0


class TrajectoryGenerator(ABC):
    @abstractmethod
    def calculate_trajectory(self, path: List[Tuple[SpatialMeasurement, SpatialMeasurement]], parameters):
        pass

class Trajectory(Drawable):
    @abstractmethod
    def get_discrete_trajectory(self, poll_step: pint.Quantity) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        pass
