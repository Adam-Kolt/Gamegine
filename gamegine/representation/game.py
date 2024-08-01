from typing import List
import pint

from gamegine.utils.logging import Debug
from gamegine.utils.unit import Meter, SpatialMeasurement, Zero
import numpy as np
from gamegine.representation import obstacle


class Game(object):

    def __init__(self, name: str = "FRC Game") -> None:
        Debug(f"Creating new game: {name}")
        self.name = name
        self.field_size = (Meter(15.0), Meter(15.0))

        self.static_obstacles = {}
        self.interactables = {}
        self.zones = {}
        self.global_states = {}
        self.field_borders = False

    def set_field_size(self, width: SpatialMeasurement, height: SpatialMeasurement):
        self.field_size = (width, height)
        if self.field_borders:
            self.__recompute_field_border_obstacles()

    def get_field_size(self):
        return self.field_size

    def half_field_size(self):
        return (self.field_size[0] / 2, self.field_size[1])

    def half_field_x(self):
        return self.field_size[0] / 2

    def half_field_y(self):
        return self.field_size[1] / 2

    def field_size(self):
        return self.field_size

    def full_field_y(self):
        return self.field_size[1]

    def full_field_x(self):
        return self.field_size[0]

    def add_obstacle(self, obstacle: obstacle.Obstacle) -> "Game":
        if obstacle.name in self.static_obstacles:
            raise Exception(
                f"Obstacle {obstacle.name} already exists. Names must be unique."
            )  #  ¯\_(ツ)_/¯

        self.static_obstacles[obstacle.name] = obstacle
        return self

    def modify_obstacle(self, obstacle: obstacle.Obstacle) -> "Game":
        self.static_obstacles[obstacle.name] = obstacle
        return self

    def modify_obstacles(self, obstacles: List[obstacle.Obstacle]) -> "Game":
        for obstacle in obstacles:
            self.modify_obstacle(obstacle)
        return self

    def get_obstacle(self, name: str) -> obstacle.Obstacle:
        return self.static_obstacles[name]

    def add_obstacles(self, obstacles: List[obstacle.Obstacle]) -> "Game":
        for obstacle in obstacles:
            self.add_obstacle(obstacle)
        return self

    def __recompute_field_border_obstacles(self) -> "Game":
        thickness = Meter(0.00001)  # In order to make the border a valid obstacle
        self.modify_obstacles(
            [
                obstacle.Rectangular(
                    "Field Border Top",
                    Zero(),
                    Zero() - thickness,
                    self.field_size[0],
                    thickness,
                ).invisible(),
                obstacle.Rectangular(
                    "Field Border Bottom",
                    Zero(),
                    self.field_size[1],
                    self.field_size[0],
                    thickness,
                ).invisible(),
                obstacle.Rectangular(
                    "Field Border Left",
                    Zero() - thickness,
                    Zero(),
                    thickness,
                    self.field_size[1],
                ).invisible(),
                obstacle.Rectangular(
                    "Field Border Right",
                    self.field_size[0],
                    Zero(),
                    thickness,
                    self.field_size[1],
                ).invisible(),
            ]
        )

    def enable_field_border_obstacles(self) -> "Game":
        self.field_borders = True
        self.__recompute_field_border_obstacles()
        return self

    def get_obstacles(self) -> List[obstacle.Obstacle]:
        return self.static_obstacles.values()

    def add_interactable(self, interactable) -> "Game":
        if interactable.name in self.interactables:
            raise Exception(
                f"Interactable {interactable.name} already exists. Names must be unique."
            )

        self.interactables[interactable.name] = interactable
        return self

    def add_zone(self, zone) -> "Game":
        if zone.name in self.zones:
            raise Exception(f"Zone {zone.name} already exists. Names must be unique.")

        self.zones[zone.name] = zone
        return self
