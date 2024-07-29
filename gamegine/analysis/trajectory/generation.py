from abc import ABC, abstractmethod
from typing import List, Tuple
from dataclasses import dataclass
import pint

from gamegine.analysis.pathfinding import Path
from gamegine.render.drawable import Drawable
from gamegine.representation.bounds import DiscreteBoundary
from gamegine.utils.unit import AngularMeasurement, SpatialMeasurement


@dataclass
class HolonomicTrajectoryParameters:  # TODO: Refine this to be cleaner
    max_translational_speed: SpatialMeasurement
    max_translational_acceleration: SpatialMeasurement
    max_translational_jerk: SpatialMeasurement

    max_rotational_speed: AngularMeasurement
    max_rotational_acceleration: AngularMeasurement
    max_rotational_jerk: AngularMeasurement

    def get_translational_parameters(
        self,
    ) -> Tuple[SpatialMeasurement, SpatialMeasurement, SpatialMeasurement]:
        return (
            self.max_translational_speed,
            self.max_translational_acceleration,
            self.max_translational_jerk,
        )

    def get_rotational_parameters(
        self,
    ) -> Tuple[AngularMeasurement, AngularMeasurement, AngularMeasurement]:
        return (
            self.max_rotational_speed,
            self.max_rotational_acceleration,
            self.max_rotational_jerk,
        )


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
