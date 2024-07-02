from abc import ABC, abstractmethod

import pint

class Drawable(ABC):
    @abstractmethod
    def z_index(self):
        pass

    @abstractmethod
    def draw(self, render_scale: pint.Quantity):
        pass

class Animated(Drawable):
    @abstractmethod
    def animate(self, render_scale: pint.Quantity, animation_tick: int, delta_time: int):
        pass