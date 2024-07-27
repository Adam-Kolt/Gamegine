from abc import ABC, abstractmethod

import pint

class Drawable(ABC):
    @abstractmethod
    def z_index(self):
        pass

    @abstractmethod
    def draw(self, render_scale: pint.Quantity):
        pass
