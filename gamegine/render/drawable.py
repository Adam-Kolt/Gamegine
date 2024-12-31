from abc import ABC, abstractmethod


from gamegine.utils.NCIM.ncim import SpatialMeasurement


class Drawable(ABC):
    """Abstract class for objects which can be drawn on the field screen."""

    @abstractmethod
    def draw(self, render_scale: SpatialMeasurement):
        pass
