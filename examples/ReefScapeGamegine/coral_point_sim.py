import logging
from typing import List
import numpy as np
import pygame
from Reefscape import Names, Reefscape, StartingPositionsBlue
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule

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
from gamegine.utils.NCIM.ncim import (
    Inch,
    Pound,
    Ampere,
    Feet,
    Degree,
    Second,
    TemporalMeasurement,
)
from gamegine.utils.logging import GetLogger
from gamepieces import Coral
from scoring import ReefState

# ROBOT GEOMETRY SETUP

ROBOT_WIDTH = Inch(32)
ROBOT_LENGTH = Inch(32)
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
    "DunceBot",
    ROBOT_SWERVE,
    ROBOT_GEOMETRY,
    PhysicalParameters(ROBOT_MASS, ROBOT_MOI),
)

SWERVE_ROBOT.override_bounding_radius(Inch(16))

# Robot Interaction Ability Setup
CORAL_PICKUP_TIME = Second(1)
L1_SCORE_TIME = Second(0.5)
L2_SCORE_TIME = Second(1)
L3_SCORE_TIME = Second(1.5)
L4_SCORE_TIME = Second(2)
ALGAE_DISLODGE_TIME = Second(1)
START_LOCATION = StartingPositionsBlue.A
START_ROTATION = Degree(0)


def init_robot_interaction():
    SWERVE_ROBOT.add_interaction_config(
        RobotInteractionConfig(
            Names.TopCoralStation,
            "PickupCoral",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: CORAL_PICKUP_TIME,
        )
    )
    SWERVE_ROBOT.add_interaction_config(
        RobotInteractionConfig(
            Names.BottomCoralStation,
            "PickupCoral",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: CORAL_PICKUP_TIME,
        )
    )

    SWERVE_ROBOT.add_interaction_config(
        RobotInteractionConfig(
            Names.Reef,
            "l1",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: L1_SCORE_TIME,
        )
    )

    SWERVE_ROBOT.add_interaction_config(
        RobotInteractionConfig(
            Names.Reef,
            "l2",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: L2_SCORE_TIME,
        )
    )

    SWERVE_ROBOT.add_interaction_config(
        RobotInteractionConfig(
            Names.Reef,
            "l3",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: L3_SCORE_TIME,
        )
    )

    SWERVE_ROBOT.add_interaction_config(
        RobotInteractionConfig(
            Names.Reef,
            "l4",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: L4_SCORE_TIME,
        )
    )

    SWERVE_ROBOT.add_interaction_config(
        RobotInteractionConfig(
            Names.Reef,
            "l2_algae",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: ALGAE_DISLODGE_TIME,
        )
    )

    SWERVE_ROBOT.add_interaction_config(
        RobotInteractionConfig(
            Names.Reef,
            "l3_algae",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: ALGAE_DISLODGE_TIME,
        )
    )


init_robot_interaction()

game_server = GameServer()


def setup_server():
    game_server.load_from_game(Reefscape)
    game_server.add_robot(SWERVE_ROBOT)

    game_server.init_robot(
        "DunceBot",
        RobotState(
            *START_LOCATION,
            START_ROTATION,
            gamepieces={Coral: 1},
        ),
    )


def priority_cycle(game_server: GameServer, priority: List[str], bother_algae: bool):
    reef_state: ReefState = game_server.game_state.get("interactables").get(Names.Reef)
    robot_state = game_server.game_state.get("robots").get("DunceBot")

    if robot_state.gamepieces.get().get(Coral, 0) > 0:
        for level in priority:
            if reef_state.open_scoring_positions(level) > 0:
                game_server.drive_and_process_action(
                    Names.Reef,
                    level,
                    "DunceBot",
                    game_server.game_state.total_time,
                    no_safety_cooridor=False,
                )
                return
            else:
                if (
                    bother_algae
                    and (level == "l2" or level == "l3")
                    and reef_state.algae_count() > 0
                ):
                    game_server.drive_and_process_action(
                        Names.Reef,
                        level + "_algae",
                        "DunceBot",
                        game_server.game_state.total_time,
                    )
                    game_server.drive_and_process_action(
                        Names.Reef,
                        level,
                        "DunceBot",
                        game_server.game_state.total_time,
                    )
                    return
    else:
        game_server.drive_and_process_action(
            Names.TopCoralStation,
            "PickupCoral",
            "DunceBot",
            game_server.game_state.total_time,
        )


def apply_penalty_not_auto(regular: TemporalMeasurement, penalty: TemporalMeasurement):
    def penalty_not_auto(
        interactableState: StateSpace, robotState: RobotState, gameState: GameState
    ):
        if gameState.current_time.get() < gameState.auto_time.get():
            return regular
        else:
            return penalty + regular

    return penalty_not_auto


DEFENSE_PENALTY = Second(0)
DEFENSE_PENALTIES = []
Results = {}

config_key = {0: "Cracked Robot", 1: "Decent Robot", 2: "Slower Robot"}

robot_configs = [
    (0.5, 0.75, 0.75, 0.75),
    (2, 2, 2, 2),
    (1, 2, 4, 6),
]


for i, config in enumerate(robot_configs):
    L1_SCORE_TIME = Second(config[0])
    L2_SCORE_TIME = Second(config[1])
    L3_SCORE_TIME = Second(config[2])
    L4_SCORE_TIME = Second(config[3])

    DEFENSE_PENALTY = Second(0)
    DEFENSE_PENALTIES = []
    Results[config_key[i]] = []

    init_robot_interaction()
    while DEFENSE_PENALTY < Second(30):
        setup_server()
        # Test Strategy
        while (
            game_server.game_state.current_time.get()
            < game_server.game_state.auto_time.get()
            + game_server.game_state.teleop_time.get()
        ):
            priority_cycle(game_server, ["l4", "l3", "l2", "l1"], True)

        DEFENSE_PENALTIES.append(DEFENSE_PENALTY)
        Results[config_key[i]].append(game_server.game_state.score.get())

        DEFENSE_PENALTY += Second(0.1)
        SWERVE_ROBOT.add_interaction_config(
            RobotInteractionConfig(
                Names.TopCoralStation,
                "PickupCoral",
                lambda statespace, robotstate, gamestate: True,
                apply_penalty_not_auto(CORAL_PICKUP_TIME, DEFENSE_PENALTY),
            )
        )


# Plot out cores vs defense penalties
import matplotlib.pyplot as plt

# Plot each robot config on the same graph with different colors
for i, config in enumerate(robot_configs):
    plt.plot(DEFENSE_PENALTIES, Results[config_key[i]], label=config_key[i])


plt.xlabel("Defense Penalty (Seconds)")
plt.ylabel("Score (Points)")
plt.title("Defense Penalty vs Score")

# Give x axis more labels for easier reading
plt.xticks(np.arange(0, 30, 1))

# Plot horizontal lines showing how many points relative to other scoring options the defense is worth. To do this, use the first, zero defense penalty score as the base score

base_score = Results[config_key[0]][0]
L4_Score = 5
Deep_Climb = 12

plt.axhline(y=base_score - L4_Score, color="r", linestyle="--", label="L4 Worth")
plt.axhline(
    y=base_score - Deep_Climb,
    color="g",
    linestyle="--",
    label="Deep Climb Worth",
)


plt.legend(
    [
        "Cracked Robot",
        "Decent Robot",
        "Slower Robot",
    ]
)


plt.show()


# print results

print("Test Log:")
print(game_server.get_log())

print("Score: ", game_server.game_state.score.get())

# log gameserver log and score into text file. Make test log have each input on separate line
with open("test_log.txt", "w") as f:
    # Log starting conditions of the test and the strategy being used
    f.write("Starting Conditions: \n")
    f.write("Robot: DunceBot\n")
    f.write("Starting Position: " + str(START_LOCATION) + "\n")
    f.write("Starting Rotation: " + str(START_ROTATION) + "\n")
    f.write("Starting Gamepieces: " + str({Coral: 1}) + "\n")
    # Scoring times for actions
    f.write("Scoring Times Config: \n")
    f.write("Coral Pickup Time: " + str(CORAL_PICKUP_TIME) + "\n")
    f.write("L1 Score Time: " + str(L1_SCORE_TIME) + "\n")
    f.write("L2 Score Time: " + str(L2_SCORE_TIME) + "\n")
    f.write("L3 Score Time: " + str(L3_SCORE_TIME) + "\n")
    f.write("L4 Score Time: " + str(L4_SCORE_TIME) + "\n")
    f.write("Algae Dislodge Time: " + str(ALGAE_DISLODGE_TIME) + "\n")
    # Strategy

    f.write("Strategy: \n")
    f.write("Priority Cycle: ['l4', 'l3', 'l2', 'l1']\n")
    f.write("Bother Algae: True\n")
    f.write("\n")
    # Log game server log

    for line in game_server.get_log():
        f.write(str(line[0]) + " : " + str(line[1]) + " : " + str(line[2]) + "\n")
    f.write("Score: " + str(game_server.game_state.score.get()))
    f.write("\n")
    reef_state = game_server.game_state.get("interactables").get(Names.Reef)
    f.write(
        "Coral Scored in L4: "
        + str(reef_state.get("l4").getValue("row_score").get())
    )
    f.write("\n")
    f.write(
        "Coral Scored in L3: "
        + str(reef_state.get("l3").getValue("row_score").get())
    )
    f.write("\n")
    f.write(
        "Coral Scored in L2: "
        + str(reef_state.get("l2").getValue("row_score").get())
    )
    f.write("\n")
    f.write(
        "Coral Scored in L1: "
        + str(reef_state.get("l1").getValue("row_score").get())
    )
    f.write("\n")
    f.write(
        "Algae Remaining: "
        + str(reef_state.algae_count())
    )
    f.write("\n")


# Render Field

renderer = Renderer.create(game=Reefscape)
traversal_space = game_server.get_traversal_space("DunceBot")

# Add obstacles and trajectories to renderer
renderer.add_obstacles(traversal_space.obstacles)
renderer.add(traversal_space.traversal_map)

trajectories = game_server.get_trajectories("DunceBot").values()
for traj in trajectories:
    renderer.add(traj)

# Add field obstacles from the game
renderer.add_obstacles(Reefscape.get_obstacles())

# Run the arcade render loop
run()

