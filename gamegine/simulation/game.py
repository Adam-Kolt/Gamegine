from gamegine.representation.interactable import RobotInteractable
from gamegine.simulation.robot import RobotState
from gamegine.simulation.state import StateSpace, ValueEntry


class GameState(StateSpace):
    def __init__(self):
        super().__init__()
        self.setValue("score", 0)
        self.setValue("auto_time", 0)
        self.setValue("teleop_time", 0)
        self.setValue("endgame_time", 0)
        self.setValue("current_time", 0)
        self.createSpace("robots")
        self.createSpace("interactables")

    @property
    def score(self) -> ValueEntry[int]:
        return self.getValue("score")

    @property
    def auto_time(self) -> ValueEntry[int]:
        return self.getValue("auto_time")

    @property
    def teleop_time(self) -> ValueEntry[int]:
        return self.getValue("teleop_time")

    @property
    def endgame_time(self) -> ValueEntry[int]:
        return self.getValue("endgame_time")

    @property
    def current_time(self) -> ValueEntry[float]:
        return self.getValue("current_time")

    @property
    def total_time(self) -> float:
        return self.auto_time.get() + self.teleop_time.get()

    def add_robot(self, robot: RobotState):
        self.get("robots").registerSpace(robot.name, robot)

    def get_robot(self, name) -> RobotState:
        return self.get("robots").get(name)

    def add_interactable(self, interactable: RobotInteractable):
        self.get("interactables").registerSpace(
            interactable.name, interactable.initializeInteractableState()
        )

    def get_interactable(self, name) -> StateSpace:
        return self.get("interactables").get(name)
