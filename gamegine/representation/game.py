from typing import List
import pint

from gamegine.utils.unit import Meter
import numpy as np
from . import obstacle


class Game(object):

    def __init__(self, name: str = "FRC Game") -> None:
        self.name = name
        self.field_size = (Meter(15.0), Meter(15.0))

        self.obstacles = {}
        self.interactables = {}
        self.zones = {}
        self.global_states = {}
        

    def set_field_size(self, width: pint.Quantity, height: pint.Quantity):
        self.field_size = (width, height)

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

    def add_obstacle(self, obstacle: obstacle.Obstacle) -> 'Game':
        if obstacle.name in self.obstacles:
            raise Exception(f"Obstacle {obstacle.name} already exists. Names must be unique.")
        
        self.obstacles[obstacle.name] = obstacle
        return self
    
    def add_obstacles(self, obstacles: List[obstacle.Obstacle]) -> 'Game':
        for obstacle in obstacles:
            self.add_obstacle(obstacle)
        return self
    
    def get_obstacles(self) -> List[obstacle.Obstacle]:
        return self.obstacles.values()

    def add_interactable(self, interactable) -> 'Game':
        if interactable.name in self.interactables:
            raise Exception(f"Interactable {interactable.name} already exists. Names must be unique.")

        self.interactables[interactable.name] = interactable
        return self

    def add_zone(self, zone) -> 'Game':
        if zone.name in self.zones:
            raise Exception(f"Zone {zone.name} already exists. Names must be unique.")

        self.zones[zone.name] = zone
        return self

