from gamegine.simulation.state import StateSpace, ValueEntry


class GameState(StateSpace):
    def __init__(self):
        super().__init__()
        self.setValue("score", 0)
        self.setValue("auto_time", 0)
        self.setValue("teleop_time", 0)
        self.setValue("endgame_time", 0)
        self.setValue("current_time", 0)

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
