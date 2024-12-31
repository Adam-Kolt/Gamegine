from enum import Enum
from typing import List
import numpy as np
from pygame import Color


class Palette(Enum):
    """A collection of colors which can be used for consistent color schemes."""

    WHITE = [255, 255, 255]
    RED = [200, 0, 0]
    GREEN = [0, 200, 0]
    LIGHT_GREEN = [0, 230, 0]
    DARK_GREEN = [0, 150, 0]
    BLUE = [0, 0, 200]
    YELLOW = [200, 200, 0]
    PINK = [218, 112, 214]
    ORANGE = [255, 165, 0]

    def get_color_array(self) -> np.ndarray:
        return np.array(self.value)


class Shade(Enum):
    """A collection of shades which can be used to adjust the brightness of a color."""

    LIGHT = 0.1
    NORMAL = 0
    DARK = -0.1


class Opacity(Enum):
    """A collection of opacities which can be used to adjust the transparency of a color."""

    OPAQUE = 1
    TRANSLUCENT = 0.5
    TRANSPARENTISH = 0.2


def get_color(color: Palette, shade: Shade, opacity: Opacity) -> Color:
    """Get a color with a specific shade and opacity.

    :param color: The color to adjust.
    :type color: :class:`Palette`
    :param shade: The shade to apply to the color.
    :type shade: :class:`Shade`
    :param opacity: The opacity to apply to the color.
    :type opacity: :class:`Opacity`
    :return: The adjusted color.
    :rtype: :class:`Color`
    """
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
    """Get a color with a specific shade and opacity.

    :param rgb: The RGB values of the color.
    :type rgb: np.array
    :param opacity: The opacity to apply to the color.
    :type opacity: :class:`Opacity`
    :return: The adjusted color.
    :rtype: :class:`Color`
    """
    if rgb.shape != (3,):
        raise ValueError("RGB must be a 3 element numpy array")
    return Color(*rgb.tolist(), a=opacity.value * 255)
