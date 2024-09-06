from typing import List
from examples.crescendo.gamepieces import Note
from gamegine.render import helpers
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
    def initializeInteractableState() -> StateSpace:
        return StateSpace()

    @staticmethod
    def __shoot_into(
        interactableState: SpeakerState, robotState: RobotState, gameState: StateSpace
    ) -> List[ValueChange]:
        robotState.gamepieces.get()[Note] -= 1
        interactableState.notes.set(interactableState.notes.get() + 1)
        return [ValueIncrease(gameState.score, 1)]

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
        helpers.draw_point(point.x, point.y, Inch(3), Palette.BLUE, render_scale)


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
    @staticmethod
    def initializeInteractableState() -> StateSpace:
        return StateSpace()

    @staticmethod
    def __put_note_into(
        interactableState: StateSpace, robotState: RobotState, gameState: StateSpace
    ) -> List[ValueChange]:
        robotState.gamepieces.get()[Note] -= 1
        interactableState.notes.set(interactableState.notes.get() + 1)
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
