from enum import Enum

# IN PROGRESSS

class Interactable(object):
    def __init__(self, name: str) -> None:
        self.name = name
        self.interaction_requirements = []
        pass

    def interact(self, parameters):
        pass

    