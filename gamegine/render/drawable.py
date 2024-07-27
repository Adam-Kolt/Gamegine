from abc import ABC, abstractmethod

import pint

from gamegine.utils.unit import SpatialMeasurement

class Drawable(ABC):
    @abstractmethod
    def z_index(self):
        pass

    @abstractmethod
    def draw(self, render_scale: SpatialMeasurement):
        pass

class Animated(Drawable):
    @abstractmethod
    def animate(self, render_scale: SpatialMeasurement, animation_tick: int, delta_time: int):
        pass