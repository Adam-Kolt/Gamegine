from typing import List
from examples.crescendo.gamepieces import Note
from gamegine.render.style import Palette
from gamegine.representation.bounds import DiscreteBoundary, Point
from gamegine.representation.interactable import InteractionOption, RobotInteractable
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.simulation.state import StateSpace, ValueChange, ValueEntry, ValueIncrease
from gamegine.utils.NCIM.Dimensions.spatial import Inch, SpatialMeasurement


class SpeakerState(StateSpace):
    def __init__(self) -> None:
        super().__init__()
        self.setValue("note_count", 0)
        self.setValue("")

    @property
    def notes(self) -> ValueEntry[int]:
        return self.getValue("note_count")


def robotHasNote(
    interactableState: StateSpace, robotState: RobotState, gameState: StateSpace
):
    return robotState.gamepieces[Note] > 0


def __is_auto_or_amplification(gameState: GameState) -> bool:
    return (
        gameState.current_time <= gameState.auto_time
        or gameState.getValue("amplification") > 0
    )


class Speaker(RobotInteractable):
    def __init__(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        z: SpatialMeasurement,
        name: str,
    ) -> None:
        super().__init__(Point(x, y, z), name)

    @staticmethod
    def initializeInteractableState() -> SpeakerState:
        return SpeakerState()

    @staticmethod
    def __shoot_into(
        interactableState: SpeakerState, robotState: RobotState, gameState: GameState
    ) -> List[ValueChange]:
        robotState.gamepieces.get()[Note] -= 1
        interactableState.notes.set(interactableState.notes.get() + 1)
        if __is_auto_or_amplification(gameState):
            return [ValueIncrease(gameState.score, 5)]
        return [ValueIncrease(gameState.score, 3)]

    @staticmethod
    def get_interactions() -> List[InteractionOption]:
        return [
            InteractionOption(
                "ShootNoteInto",
                "Shoot a note into the speaker",
                robotHasNote,
                Speaker.__shoot_into,
            )
        ]

    def draw(self, render_scale: SpatialMeasurement):
        point: Point = self.bounds
        #helpers.draw_point(point.x, point.y, Inch(3), Palette.BLUE, render_scale)
        pass

class AmplifierState(StateSpace):
    def __init__(self) -> None:
        super().__init__()
        self.setValue("current_note_count", 0)
        self.setValue("total_note_count", 0)

    @property
    def notes(self) -> ValueEntry[int]:
        return self.getValue("note_count")

    @property
    def total_notes(self) -> ValueEntry[int]:
        return self.getValue("total_note_count")


class Amplifier(RobotInteractable):
    def __init__(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        z: SpatialMeasurement,
        name: str,
    ) -> None:
        super().__init__(Point(x, y, z), name)

    @staticmethod
    def initializeInteractableState() -> StateSpace:
        return AmplifierState()

    @staticmethod
    def __put_note_into(
        interactableState: StateSpace, robotState: RobotState, gameState: StateSpace
    ) -> List[ValueChange]:
        robotState.gamepieces.get()[Note] -= 1
        interactableState.notes.set(interactableState.notes.get() + 1)
        if __is_auto_or_amplification(gameState):
            return [ValueIncrease(gameState.score, 3)]
        return [ValueIncrease(gameState.score, 2)]

    @staticmethod
    def __contains_notes(
        interactableState: StateSpace, robotState: RobotState, gameState: StateSpace
    ) -> bool:
        return interactableState.notes.get() > 2

    @staticmethod
    def __amplify(
        interactableState: AmplifierState, robotState: RobotState, gameState: StateSpace
    ) -> List[ValueChange]:
        gameState.setValue("amplification", 10)
        interactableState.notes.set(interactableState.notes.get() - 3)
        return []

    @staticmethod
    def get_interactions() -> List[InteractionOption]:
        return [
            InteractionOption(
                "PutNoteInto",
                "Put a note into the amplifier",
                robotHasNote,
                Amplifier.__put_note_into,
            ),
            InteractionOption(
                "Amplify",
                "Begin amplification period",
                Amplifier.__contains_notes,
                Amplifier.__amplify,
            ),
        ]


class StageState(StateSpace):
    def __init__(self) -> None:
        super().__init__()
        self.setValue("chain1", 0)
        self.setValue("chain2", 0)
        self.setValue("chain3", 0)
        self.setValue("spotlight1", False)
        self.setValue("spotlight2", False)
        self.setValue("spotlight3", False)
        self.setValue("trap1", False)
        self.setValue("trap2", False)
        self.setValue("trap3", False)
        self.setValue("stage_points", 0)

    @property
    def climbed(self) -> ValueEntry[bool]:
        return self.getValue("climbed")

    @property
    def chain1(self) -> ValueEntry[int]:
        return self.getValue("chain1")

    @property
    def chain2(self) -> ValueEntry[int]:
        return self.getValue("chain2")

    @property
    def chain3(self) -> ValueEntry[int]:
        return self.getValue("chain3")

    @property
    def spotlight1(self) -> ValueEntry[bool]:
        return self.getValue("spotlight1")

    @property
    def spotlight2(self) -> ValueEntry[bool]:
        return self.getValue("spotlight2")

    @property
    def spotlight3(self) -> ValueEntry[bool]:
        return self.getValue("spotlight3")

    @property
    def trap1(self) -> ValueEntry[bool]:
        return self.getValue("trap1")

    @property
    def trap2(self) -> ValueEntry[bool]:
        return self.getValue("trap2")

    @property
    def trap3(self) -> ValueEntry[bool]:
        return self.getValue("trap3")

    @property
    def stage_points(self) -> ValueEntry[int]:
        return self.getValue("stage_points")


class Stage(RobotInteractable):
    def __init__(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        z: SpatialMeasurement,
        name: str,
    ) -> None:
        super().__init__(Point(x, y, z), name)

    @staticmethod
    def initializeInteractableState() -> StateSpace:
        return StageState()

    @staticmethod
    def __chain_scoring(chain_name: str):

        def _chain(
            interactableState: StageState, robotState: RobotState, gameState: GameState
        ) -> List[ValueChange]:
            chain_value = interactableState.getValue(chain_name)

            chain_value.set(chain_value.get() + 1)

            score = 0
            if chain_value.get() > 1:
                score = 15
            else:
                score = 5

            return [
                ValueIncrease(gameState.score, score),
                ValueIncrease(interactableState.stage_points, score),
            ]

        return _chain

    @staticmethod
    def __trap(trap_name: str):

        def _trap(
            interactableState: StageState, robotState: RobotState, gameState: GameState
        ) -> List[ValueChange]:
            robotState.gamepieces.get()[Note] -= 1
            interactableState.getValue(trap_name).set(True)
            return [ValueIncrease(gameState.score, 10)]

        return _trap

    @staticmethod
    def __spotlight(spotlight_name: str):

        def _spotlight(
            interactableState: StageState, robotState: RobotState, gameState: GameState
        ) -> List[ValueChange]:
            interactableState.getValue(spotlight_name).set(True)
            return []

        return _spotlight

    @staticmethod
    def get_interactions() -> List[InteractionOption]:
        return [
            InteractionOption(
                "Chain1",
                "Climb chain 1",
                lambda *args: True,
                Stage.__chain_scoring("chain1"),
            ),
            InteractionOption(
                "Chain2",
                "Climb chain 2",
                lambda *args: True,
                Stage.__chain_scoring("chain2"),
            ),
            InteractionOption(
                "Chain3",
                "Climb chain 3",
                lambda *args: True,
                Stage.__chain_scoring("chain3"),
            ),
            InteractionOption(
                "Trap1",
                "Activate trap 1",
                robotHasNote,
                Stage.__trap("trap1"),
            ),
            InteractionOption(
                "Trap2",
                "Activate trap 2",
                robotHasNote,
                Stage.__trap("trap2"),
            ),
            InteractionOption(
                "Trap3",
                "Activate trap 3",
                robotHasNote,
                Stage.__trap("trap3"),
            ),
            InteractionOption(
                "Spotlight1",
                "Activate spotlight 1",
                lambda *args: True,
                Stage.__spotlight("spotlight1"),
            ),
            InteractionOption(
                "Spotlight2",
                "Activate spotlight 2",
                lambda *args: True,
                Stage.__spotlight("spotlight2"),
            ),
            InteractionOption(
                "Spotlight3",
                "Activate spotlight 3",
                lambda *args: True,
                Stage.__spotlight("spotlight3"),
            ),
        ]
