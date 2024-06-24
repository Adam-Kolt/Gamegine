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

    def add_obstacle(self, obstacle: obstacle.Obstacle):
        self.obstacles[obstacle.name] = obstacle

    def add_interactable(self, interactable):
        self.interactables[interactable.name] = interactable

    def add_zone(self, zone):
        self.zones[zone.name] = zone

