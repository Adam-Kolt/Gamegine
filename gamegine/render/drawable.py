from abc import ABC, abstractmethod

import pint

from gamegine.utils.NCIM.ncim import SpatialMeasurement


class Drawable(ABC):
    @abstractmethod
    def draw(self, render_scale: SpatialMeasurement):
        pass
