from enum import Enum
from abc import ABC, abstractmethod

# IN PROGRESSS


class RobotInteractable(ABC):
    def __init__(self, name: str) -> None:
        self.name = name
        self.interaction_requirements = []
        pass

    def interact(self, parameters):
        pass
