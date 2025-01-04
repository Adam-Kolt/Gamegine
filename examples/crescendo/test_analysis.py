import logging
from typing import List
import pygame
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.render import helpers
from gamegine.render.style import Palette
from gamegine.representation.bounds import Circle, DiscreteBoundary, Point
from gamegine.representation.gamepiece import Gamepiece, GamepiecePhysicalProperties
from gamegine.representation.interactable import InteractionOption, RobotInteractable
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.simulation.state import StateSpace, ValueChange, ValueEntry, ValueIncrease

from gamegine.utils.NCIM.Dimensions.spatial import Inch, SpatialMeasurement
from gamegine.utils.NCIM.Dimensions.mass import Ounce, Pound

from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import (
    CircularPattern,
    Cylinder,
    Polygon,
    Rectangle,
    SymmetricalX,
    Transform3D,
)
from gamegine.representation.game import Game
from gamegine.representation.interactable import RobotInteractionConfig
from gamegine.representation.obstacle import Obstacle
from gamegine.representation.robot import (
    PhysicalParameters,
    SwerveRobot,
)
from gamegine.simulation.GameServer import GameServer
from gamegine.simulation.robot import RobotState
from gamegine.utils.NCIM.ncim import Inch, Pound, Ampere, Feet, Degree
from gamegine.utils.logging import GetLogger

Crescendo = Game("FRC Crescendo 2024")
# GetLogger().setLevel(logging.CRITICAL)

print("Name:", Crescendo.name)
Crescendo.set_field_size(Feet(54) + Inch(3.25), Feet(26) + Inch(11.25))
objs = SymmetricalX(
    [
        *CircularPattern(
            [
                Obstacle(
                    "Stage Leg",
                    Cylinder(
                        Inch(7),
                        Inch(74.5),
                        Transform3D((Inch(133), Inch(161.62), Inch(74.5) / 2)),
                        32,
                    ),
                )
            ],  # Circular("Stage Leg", Inch(133), Inch(161.62), Inch(7))
            (Inch(133) + Inch(59.771), Inch(161.62)),
            Degree(360),
            3,
            lambda i: str(i),
        ),
        Obstacle(
            "Subwoofer",
            Polygon(
                [
                    (Inch(0), Inch(64.081)),
                    (Inch(0), Inch(64.081) + Inch(82.645)),
                    (Inch(35.695), Inch(64.081) + Inch(82.645) - Inch(20.825)),
                    (Inch(35.695), Inch(64.081) + Inch(20.825)),
                ]
            ).get_3d(z_end=Feet(2)),
        ),
        Obstacle(
            "Source",
            Polygon(
                [
                    (Inch(0), Inch(281.5)),
                    (Inch(0), Crescendo.full_field_y()),
                    (Inch(72.111), Crescendo.full_field_y()),
                ]
            ).get_3d(z_end=Feet(4)),
        ),
        Obstacle(
            "Stage Base",
            Polygon(
                [
                    (Inch(133), Inch(161.62)),
                    (
                        Inch(133) + Inch(29.855) + Inch(59.77),
                        Inch(161.62) + Inch(51.76),
                    ),
                    (
                        Inch(133) + Inch(29.855) + Inch(59.77),
                        Inch(161.62) - Inch(51.76),
                    ),
                ]
            ).get_3d(Inch(27.83), Inch(74.5)),
        ),
    ],
    Crescendo.half_field_x(),
    "Red ",
    "Blue ",
)


Crescendo.add_obstacles(objs)
Crescendo.enable_field_border_obstacles()


class Note(Gamepiece):
    name = "Note"
    bounds = Circle(Inch(0), Inch(0), Inch(7)).get_3d(Inch(0), Inch(1))
    physical_properties = GamepiecePhysicalProperties(Ounce(8.3), 0.5)

    # AndyMark Note Dimensions
    INNER_RADIUS = Inch(5)
    OUTER_RADIUS = Inch(7)

    @classmethod
    def display(
        cls,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        render_scale: SpatialMeasurement,
    ):
        helpers.draw_point(x, y, cls.OUTER_RADIUS, Palette.ORANGE, render_scale)


class SpeakerState(StateSpace):
    def __init__(self) -> None:
        super().__init__()
        self.setValue("note_count", 0)

    @property
    def notes(self) -> ValueEntry[int]:
        return self.getValue("note_count")


def robotHasNote(
    interactableState: StateSpace, robotState: RobotState, gameState: StateSpace
):
    return robotState.gamepieces[Note] > 0


def is_auto_or_amplification(gameState: GameState) -> bool:
    return (
        gameState.current_time.get() <= gameState.auto_time.get()
        or gameState.getValue("amplification").get() > 0
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
        changes = []
        robotState.gamepieces.get()[Note] -= 1
        changes.append(ValueIncrease(interactableState.notes, 1))
        if is_auto_or_amplification(gameState):
            changes.append(ValueIncrease(gameState.score, 5))
        else:
            changes.append(ValueIncrease(gameState.score, 3))
        return changes

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
        if is_auto_or_amplification(gameState):
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


ROBOT_WIDTH = Inch(27)
ROBOT_LENGTH = Inch(27)
ROBOT_MASS = Pound(120)
ROBOT_MOI = Pound(120) * Inch(30) ** 2
ROBOT_SWERVE = SwerveConfig(
    SwerveModule(
        motors.MotorConfig(
            motors.KrakenX60,
            motors.PowerConfig(Ampere(60), Ampere(240), 1.0),
        ),
        gearing.MK4I.L3,
        motors.MotorConfig(
            motors.KrakenX60,
            motors.PowerConfig(Ampere(60), Ampere(240), 1.0),
        ),
        gearing.MK4I.L3,
    )
)

ROBOT_GEOMETRY = [
    Rectangle.from_center((Inch(0), Inch(0)), ROBOT_WIDTH, ROBOT_LENGTH).get_3d(
        Inch(0), Feet(3)
    )
]


SWERVE_ROBOT = SwerveRobot(
    "DaBot",
    ROBOT_SWERVE,
    ROBOT_GEOMETRY,
    PhysicalParameters(ROBOT_MASS, ROBOT_MOI),
)

SWERVE_ROBOT.add_interaction_config(
    RobotInteractionConfig(
        "Red Amplifier",
        "PutNoteInto",
        lambda interactableState, robotState, gameState: True,
        lambda interactableState, robotState, gameState: 1.0,
    )
)

SWERVE_ROBOT.add_interaction_config(
    RobotInteractionConfig(
        "Red Speaker",
        "ShootNoteInto",
        lambda interactableState, robotState, gameState: True,
        lambda interactableState, robotState, gameState: 5.0,
    )
)

Crescendo.add_interactable(
    Speaker(Inch(0), (Inch(64.081) * 2 + Inch(82.645)) / 2, Inch(0), "Red Speaker")
)

Crescendo.add_interactable(
    Amplifier(
        Inch(35.695) + Inch(20.825) / 2,
        Inch(64.081) + Inch(82.645) / 2,
        Inch(0),
        "Red Amplifier",
    )
)

Crescendo.add_interactable(
    Amplifier(
        Crescendo.full_field_x() - (Inch(35.695) + Inch(20.825) / 2),
        Inch(64.081) + Inch(82.645) / 2,
        Inch(0),
        "Blue Amplifier",
    )
)

note_selections = [
    (1, 2, 3),
    (2, 3, 1),
    (3, 1, 2),
]

start_locations = [
    (Inch(114) - Inch(70), Inch(161.62)),
    (Inch(114) - Inch(50), Inch(161.62) - Inch(57)),
    (Inch(114) - Inch(70), Inch(161.62) - Inch(57) * 2),
]

notes = [
    (Inch(114), Inch(161.62)),
    # (Inch(114), Inch(161.62) - Inch(57)),
    # (Inch(114), Inch(161.62) - Inch(57) * 2),
]


def is_auto_over(gameState: GameState) -> bool:
    return gameState.current_time.get() > gameState.auto_time.get()


test_results = {}
game_server = GameServer()
for i, start_location in enumerate(start_locations):

    game_server.add_robot(SWERVE_ROBOT)

    game_server.load_from_game(Crescendo)
    game_server.game_state.setValue("amplification", 0)

    game_server.init_robot(
        "DaBot",
        RobotState(
            *start_location,
            Degree(0),
            gamepieces={Note: 1},
        ),
    )

    pos = 0
    while (
        game_server.game_state.current_time.get()
        <= game_server.game_state.auto_time.get()
    ):
        game_server.drive_robot("DaBot", notes[pos][0], notes[pos][1], Degree(180))
        if is_auto_over(game_server.game_state):
            break
        game_server.pickup_gamepiece("DaBot", Note)
        game_server.process_action("Red Speaker", "ShootNoteInto", "DaBot")

        pos += 1
        if pos >= len(notes):
            break
    final_auto_score = game_server.game_state.score.get()

    if start_location not in test_results:
        test_results[start_location] = []

    test_results[start_location].append(
        (final_auto_score, game_server.game_state.current_time.get())
    )

traversal_space = game_server.get_traversal_space("DaBot")

print("Auto Test Results:")
print(test_results)


# Drawing end visualization
renderer = Renderer()
renderer.set_game(Crescendo)
renderer.init_display()
renderer.set_render_scale(Inch(0.5))
trajectories = game_server.get_trajectories("DaBot").values()
loop = True
while loop != False:
    loop = renderer.loop()
    ev = pygame.event.get()
    renderer.draw_elements(traversal_space.obstacles)

    renderer.draw_element(traversal_space.traversal_map)
    renderer.draw_elements(trajectories)

    renderer.draw_static_elements()
    game_server.game_state.get("robots").get("DaBot").draw_real(
        renderer.render_scale, SWERVE_ROBOT
    )
    renderer.render_frame()
