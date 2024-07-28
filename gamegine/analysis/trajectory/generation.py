from abc import ABC, abstractmethod
from typing import List, Tuple
from dataclasses import dataclass
import pint

from gamegine.analysis.pathfinding import Path
from gamegine.render.drawable import Drawable
from gamegine.representation.bounds import DiscreteBoundary
from gamegine.utils.unit import SpatialMeasurement


@dataclass
class HolonomicTrajectoryParameters:
    max_speed: SpatialMeasurement
    max_acceleration: SpatialMeasurement


class TrajectoryGenerator(ABC):
    @abstractmethod
    def calculate_trajectory(
        self,
        path: Path,
        parameters: HolonomicTrajectoryParameters,
        obstacles: List[DiscreteBoundary],
    ):
        pass


class Trajectory(Drawable):
    @abstractmethod
    def get_discrete_trajectory(
        self, poll_step: SpatialMeasurement
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        pass
