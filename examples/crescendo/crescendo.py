import math
from typing import List, Tuple

import pint
import pygame
from examples.crescendo.gamepieces import Note
from examples.crescendo.scoring import Speaker
from gamegine.analysis.meshing import TriangulatedGraph, VisibilityGraph
from gamegine.analysis.trajectory.SafetyCorridorAssisted import SafetyCorridorAssisted
from gamegine.analysis.trajectory.generation import (
    SwerveTrajectory,
    Trajectory,
    TrajectoryKeypoint,
)
from gamegine.analysis.trajectory.lib.TrajGen import (
    SolverConfig,
    SwerveRobotConstraints,
    SwerveTrajectoryProblemBuilder,
    TrajectoryBuilderConfig,
    Waypoint,
)
from gamegine.analysis.trajectory.lib.constraints.avoidance import (
    SAFETY_CORRIDOR_DEBUG,
    SafetyCorridor,
)
from gamegine.analysis.trajectory.lib.constraints.constraints import (
    AngleEquals,
    VelocityEquals,
)
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.render.renderer import Renderer
from gamegine.representation.apriltag import AprilTag, AprilTagFamily
from gamegine.representation.bounds import (
    Circle,
    ExpandedObjectBounds,
    Point,
    Rectangle,
    SymmetricalX,
    CircularPattern,
)
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Circular, Polygonal, Rectangular
from gamegine.representation.robot import (
    PhysicalParameters,
    SwerveDrivetrainCharacteristics,
)
from gamegine.utils.NCIM.ComplexDimensions.MOI import PoundsInchesSquared
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.alpha import RadiansPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.electricpot import Volt
from gamegine.utils.NCIM.ComplexDimensions.omega import (
    RadiansPerSecond,
    RotationsPerSecond,
)
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Radian
from gamegine.utils.NCIM.Dimensions.current import Ampere
from gamegine.utils.NCIM.ncim import (
    Degree,
    Kilogram,
    KilogramMetersSquared,
    Meter,
    Centimeter,
    Feet,
    Inch,
    MetersPerSecond,
    Pound,
    SpatialMeasurement,
)
from gamegine.analysis import pathfinding
import time

from gamegine.utils.logging import Debug


Crescendo = Game("FRC Crescendo 2024")

print("Name:", Crescendo.name)
Crescendo.set_field_size(Feet(54) + Inch(3.25), Feet(26) + Inch(11.25))
objs = SymmetricalX(
    [
        *CircularPattern(
            [Circular("Stage Leg", Inch(133), Inch(161.62), Inch(7))],
            (Inch(133) + Inch(59.771), Inch(161.62)),
            Degree(360),
            3,
            lambda i: str(i),
        ),
        Polygonal(
            "Subwoofer",
            [
                (Inch(0), Inch(64.081)),
                (Inch(0), Inch(64.081) + Inch(82.645)),
                (Inch(35.695), Inch(64.081) + Inch(82.645) - Inch(20.825)),
                (Inch(35.695), Inch(64.081) + Inch(20.825)),
            ],
        ),
        Polygonal(
            "Source",
            [
                (Inch(0), Inch(281.5)),
                (Inch(0), Crescendo.full_field_y()),
                (Inch(72.111), Crescendo.full_field_y()),
            ],
        ),
        # Circular("Note 1", Inch(114.010), Inch(47.638), Inch(7)),
        # Circular("Note 2", Inch(114.010), Inch(47.638) + Inch(43.000), Inch(7)),
        # Circular("Note 3", Inch(114.010), Inch(47.638) + Inch(43.000) * 2, Inch(7)),
    ],
    Crescendo.half_field_x(),
    "Red ",
    "Blue ",
)


Crescendo.add_obstacles(objs)
Crescendo.enable_field_border_obstacles()

starting_points = []  # Points where the robot usually starts
points = [point.get_vertices()[0] for point in starting_points]


apriltag_list = [
    AprilTag(
        Inch(593.68),
        Inch(313.57),
        Inch(53.38),
        Degree(-120),
        1,
        AprilTagFamily.TAG_36h11,
    ),
    AprilTag(
        Inch(637.21),
        Inch(288.46),
        Inch(53.38),
        Degree(-120),
        2,
        AprilTagFamily.TAG_36h11,
    ),
    AprilTag(
        Inch(652.73),
        Inch(127.08),
        Inch(57.13),
        Degree(-180),
        3,
        AprilTagFamily.TAG_36h11,
    ),
    AprilTag(
        Inch(652.73),
        Inch(104.83),
        Inch(57.13),
        Degree(-180),
        4,
        AprilTagFamily.TAG_36h11,
    ),
    AprilTag(
        Inch(578.77), Inch(0.25), Inch(53.38), Degree(-270), 5, AprilTagFamily.TAG_36h11
    ),
    AprilTag(
        Inch(72.5), Inch(0.25), Inch(53.38), Degree(-270), 6, AprilTagFamily.TAG_36h11
    ),
    AprilTag(
        Inch(-1.5), Inch(104.83), Inch(57.13), Degree(0), 7, AprilTagFamily.TAG_36h11
    ),
    AprilTag(
        Inch(-1.5), Inch(127.08), Inch(57.13), Degree(0), 8, AprilTagFamily.TAG_36h11
    ),
    AprilTag(
        Inch(14.02), Inch(288.46), Inch(53.38), Degree(-60), 9, AprilTagFamily.TAG_36h11
    ),
    AprilTag(
        Inch(57.54),
        Inch(313.57),
        Inch(53.38),
        Degree(-60),
        10,
        AprilTagFamily.TAG_36h11,
    ),
    AprilTag(
        Inch(468.69),
        Inch(177.06),
        Inch(52.00),
        Degree(-300),
        11,
        AprilTagFamily.TAG_36h11,
    ),
    AprilTag(
        Inch(468.69),
        Inch(146.15),
        Inch(52.00),
        Degree(-60),
        12,
        AprilTagFamily.TAG_36h11,
    ),
    AprilTag(
        Inch(441.74),
        Inch(161.63),
        Inch(52.00),
        Degree(-180),
        13,
        AprilTagFamily.TAG_36h11,
    ),
    AprilTag(
        Inch(209.48), Inch(161.63), Inch(52.00), Degree(0), 14, AprilTagFamily.TAG_36h11
    ),
    AprilTag(
        Inch(182.73),
        Inch(146.15),
        Inch(52.00),
        Degree(-120),
        15,
        AprilTagFamily.TAG_36h11,
    ),
    AprilTag(
        Inch(182.73),
        Inch(177.06),
        Inch(52.00),
        Degree(-240),
        16,
        AprilTagFamily.TAG_36h11,
    ),
]

notes = [
    Note(Inch(114), Inch(161.62)),
    Note(Inch(114), Inch(161.62) - Inch(57)),
    Note(Inch(114), Inch(161.62) - Inch(57) * 2),
]

x_r, y_r = Crescendo.half_field_x(), Crescendo.half_field_y()
test_robot_bounds = [
    Rectangle(
        x_r - Inch(30 + 4) / 2, y_r - Inch(30 + 4) / 2, Inch(30 + 4), Inch(30 + 4)
    ).get_3d(Inch(1), Inch(5)),
    Rectangle(x_r - Inch(30) / 2, y_r - Inch(30) / 2, Inch(30), Inch(30)).get_3d(
        Inch(5), Feet(3)
    ),
]

test_speaker = Speaker(Inch(0), Inch(64.081) + Inch(82.645) / 2, Feet(3), "Red Speaker")
