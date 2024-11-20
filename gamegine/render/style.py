from enum import Enum
from typing import List
import numpy as np
from pygame import Color as PygameColor


class Color:
    def __init__(self, r: int, g: int, b: int, a: int):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

    def get_color_array(self) -> np.ndarray:
        return np.array([self.r, self.g, self.b, self.a])

    def get_color_tuple(self) -> tuple:
        return (self.r, self.g, self.b, self.a)

    def get_color(self) -> "Color":
        return self

    def __eq__(self, other: "Color") -> bool:
        return self.get_color_tuple() == other.get_color_tuple()

    def __ne__(self, other: "Color") -> bool:
        return self.get_color_tuple() != other.get_color_tuple()

    def __repr__(self) -> str:
        return f"Color({self.r}, {self.g}, {self.b}, {self.a})"

    def get_pygame_color(self) -> PygameColor:
        return PygameColor(self.get_color_tuple())


class Base:
    def __init__(self, base_color: Color) -> None:
        self.base_color = base_color

    def get_color(self, shade: float = 0.0, opacity: float = 1.0) -> Color:
        return Color(
            *(
                self.base_color.get_color_array()
                + self.base_color.get_color_array() * shade
            )
            .clip(0, 255)
            .astype(int)
            .tolist(),
            int(opacity * 255),
        )


class Palette:
    RED = Base(Color(255, 0, 0, 255))
    GREEN = Base(Color(0, 255, 0, 255))
    BLUE = Base(Color(0, 0, 255, 255))
    YELLOW = Base(Color(255, 255, 0, 255))
    ORANGE = Base(Color(255, 165, 0, 255))
    PURPLE = Base(Color(128, 0, 128, 255))
    CYAN = Base(Color(0, 255, 255, 255))
    PINK = Base(Color(255, 192, 203, 255))
    WHITE = Base(Color(255, 255, 255, 255))
    BLACK = Base(Color(0, 0, 0, 255))
