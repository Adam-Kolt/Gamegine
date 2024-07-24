from abc import ABC, abstractmethod
from typing import List, Tuple

import pint

from gamegine.render.drawable import Drawable

class TrajectoryGenerator(ABC):
    @abstractmethod
    def calculate_trajectory(self, path: List[Tuple[pint.Quantity, pint.Quantity]], parameters):
        pass


class Trajectory(ABC):

    @abstractmethod
    def get_discrete_trajectory(self, poll_step: pint.Quantity) -> List[Tuple[pint.Quantity, pint.Quantity]]:
        pass

    @abstractmethod
    def get_display_object(self, debug: bool = False) -> Drawable:
        pass
