from gamegine.representation.interactable import RobotInteractable
from gamegine.simulation.robot import RobotState
from gamegine.simulation.state import StateSpace, ValueEntry
from gamegine.first.alliance import Alliance


class GameState(StateSpace):
    def __init__(self):
        super().__init__()
        # Legacy score (for backward compatibility / zero-sum games)
        self.setValue("score", 0)
        # Per-alliance scores for alliance-based training
        self.setValue("red_score", 0)
        self.setValue("blue_score", 0)
        # Time tracking
        self.setValue("auto_time", 0)
        self.setValue("teleop_time", 0)
        self.setValue("endgame_time", 0)
        self.setValue("current_time", 0)
        self.createSpace("robots")
        self.createSpace("interactables")

    @property
    def score(self) -> ValueEntry[int]:
        """Legacy score (backward compatible)."""
        return self.getValue("score")
    
    @property
    def red_score(self) -> ValueEntry[int]:
        """Red alliance score."""
        return self.getValue("red_score")
    
    @property
    def blue_score(self) -> ValueEntry[int]:
        """Blue alliance score."""
        return self.getValue("blue_score")
    
    def get_alliance_score(self, alliance: Alliance) -> ValueEntry[int]:
        """Get score entry for a specific alliance."""
        if alliance == Alliance.RED:
            return self.red_score
        return self.blue_score
    
    def get_alliance_score_value(self, alliance: Alliance) -> int:
        """Get current score value for a specific alliance."""
        return self.get_alliance_score(alliance).get()

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

