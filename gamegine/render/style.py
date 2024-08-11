from enum import Enum
from typing import List
import numpy as np
from pygame import Color


class Palette(Enum):
    RED = [200, 0, 0]
    GREEN = [0, 200, 0]
    LIGHT_GREEN = [0, 230, 0]
    DARK_GREEN = [0, 150, 0]
    BLUE = [0, 0, 200]
    YELLOW = [200, 200, 0]
    PINK = [218, 112, 214]

    def get_color_array(self) -> np.ndarray:
        return np.array(self.value)


class Shade(Enum):
    LIGHT = 0.1
    NORMAL = 0
    DARK = -0.1


class Opacity(Enum):
    OPAQUE = 1
    TRANSLUCENT = 0.5
    TRANSPARENTISH = 0.2


def get_color(color: Palette, shade: Shade, opacity: Opacity) -> Color:
    return Color(
        *(
            (color.get_color_array() + color.get_color_array() * shade.value)
            .clip(0, 255)
            .astype(int)
            .tolist()
        ),
        int(opacity.value * 255)
    )


def get_color_from_rgb(rgb: np.array, opacity: Opacity) -> Color:
    if rgb.shape != (3,):
        raise ValueError("RGB must be a 3 element numpy array")
    return Color(*rgb.tolist(), a=opacity.value * 255)
