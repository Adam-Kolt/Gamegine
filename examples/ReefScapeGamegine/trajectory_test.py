import logging
import math
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

from gamegine.render.renderer import Renderer, run
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
    Radian,
    Second,
    TemporalMeasurement,
)
from gamegine.utils.logging import GetLogger
from gamepieces import Coral
from scoring import ReefState

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
PROCESSOR_SCORE_TIME = Second(0.5)
BARGE_CLIMB_TIME = Second(3)
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

    scoring_columns = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]
    Reef_Position = (Inch(176.746), Reefscape.half_field_y())
    away = Inch(32.5800055942) + ROBOT_LENGTH + Inch(3)
    angle_step = Degree(360) / 6
    curr_angle = Degree(180)
    for i, column in enumerate(scoring_columns):

        nav_point = (
            Reef_Position[0] + away * math.cos(curr_angle.to(Radian)),
            Reef_Position[1] + away * math.sin(curr_angle.to(Radian)),
            Degree(180) + curr_angle,
        )

        SWERVE_ROBOT.add_interaction_config(
            RobotInteractionConfig(
                Names.Reef,
                f"l1_{column}",
                lambda statespace, robotstate, gamestate: True,
                lambda statespace, robotstate, gamestate: L1_SCORE_TIME,
                navigation_point=nav_point,
            )
        )

        SWERVE_ROBOT.add_interaction_config(
            RobotInteractionConfig(
                Names.Reef,
                f"l2_{column}",
                lambda statespace, robotstate, gamestate: True,
                lambda statespace, robotstate, gamestate: L2_SCORE_TIME,
                navigation_point=nav_point,
            )
        )

        SWERVE_ROBOT.add_interaction_config(
            RobotInteractionConfig(
                Names.Reef,
                f"l3_{column}",
                lambda statespace, robotstate, gamestate: True,
                lambda statespace, robotstate, gamestate: L3_SCORE_TIME,
                navigation_point=nav_point,
            )
        )

        SWERVE_ROBOT.add_interaction_config(
            RobotInteractionConfig(
                Names.Reef,
                f"l4_{column}",
                lambda statespace, robotstate, gamestate: True,
                lambda statespace, robotstate, gamestate: L4_SCORE_TIME,
                navigation_point=nav_point,
            )
        )
        if (i + 1) % 2 == 0:
            curr_angle += angle_step

    algae_sides = [
        ["A", "B"],
        ["C", "D"],
        ["E", "F"],
        ["G", "H"],
        ["I", "J"],
        ["K", "L"],
    ]

    for side in algae_sides:
        SWERVE_ROBOT.add_interaction_config(
            RobotInteractionConfig(
                Names.Reef,
                f"dislodge_{side[0]}{side[1]}",
                lambda statespace, robotstate, gamestate: True,
                lambda statespace, robotstate, gamestate: ALGAE_DISLODGE_TIME,
            )
        )

        SWERVE_ROBOT.add_interaction_config(
            RobotInteractionConfig(
                Names.Reef,
                f"pickup_{side[0]}{side[1]}",
                lambda statespace, robotstate, gamestate: True,
                lambda statespace, robotstate, gamestate: ALGAE_DISLODGE_TIME,
            )
        )

    # SWERVE_ROBOT.add_interaction_config(
    #     RobotInteractionConfig(
    #         Names.Barge,
    #         "Deep2",
    #         lambda statespace, robotstate, gamestate: True,
    #         lambda statespace, robotstate, gamestate: BARGE_CLIMB_TIME,
    #         (Reefscape.half_field_x(), Inch(42.937 + 31.178), Degree(0)),
    #     )
    # )

    SWERVE_ROBOT.add_interaction_config(
        RobotInteractionConfig(
            Names.Processor,
            "processor",
            lambda statespace, robotstate, gamestate: True,
            lambda statespace, robotstate, gamestate: PROCESSOR_SCORE_TIME,
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


setup_server()

actions = game_server.get_actions_set("DunceBot")

for action in actions:
    result = game_server.drive_and_process_action(
        action[0], action[1], "DunceBot", game_server.game_state.total_time
    )


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

