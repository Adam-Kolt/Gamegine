from enum import Enum
import numpy as np
from pygame import Color

class Palette(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3
    YELLOW = 4

    color_map = {
        RED: np.array([255, 0, 0]),
        GREEN: np.array([0, 255, 0]),
        BLUE: np.array([0, 0, 255]),
        YELLOW: np.array([255, 255, 0])
    }


    def get_color_array(self) -> np.ndarray:
        return np.array([200, 0, 0])
        

class Shade(Enum):
    LIGHT = 0.1
    NORMAL = 0
    DARK = -0.1

class Opacity(Enum):
    OPAQUE = 1
    TRANSLUCENT = 0.5
    TRANSPARENTISH = 0.8

def get_color(color: Palette, shade: Shade, opacity: Opacity) -> Color:
    return Color(*((color.get_color_array() + color.get_color_array() * shade.value).clip(0, 255).astype(int).tolist()), a=int(opacity.value*255))

def get_color_from_rgb(rgb: np.array, opacity: Opacity) -> Color:
    if rgb.shape != (3,):
        raise ValueError("RGB must be a 3 element numpy array")
    return Color(*rgb.tolist(), a=opacity.value*255)

