from abc import ABC, abstractmethod
from typing import List, Tuple

import pint

from gamegine.render.drawable import Drawable
from gamegine.utils.unit import SpatialMeasurement

class TrajectoryGenerator(ABC):
    @abstractmethod
    def calculate_trajectory(self, path: List[Tuple[SpatialMeasurement, SpatialMeasurement]], parameters):
        pass


class Trajectory(ABC):

    @abstractmethod
    def get_discrete_trajectory(self, poll_step: SpatialMeasurement) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        pass

    @abstractmethod
    def get_display_object(self, debug: bool = False) -> Drawable:
        pass
