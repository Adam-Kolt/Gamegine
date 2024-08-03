import math
from typing import List, Tuple

import pint
from gamegine.analysis.meshing import TriangulatedGraph, VisibilityGraph
from gamegine.analysis.trajectory.SafetyCorridorAssisted import SafetyCorridorAssisted
from gamegine.analysis.trajectory.generation import (
    SwerveTrajectory,
    Trajectory,
    TrajectoryKeypoint,
)
from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import (
    Circle,
    ExpandedObjectBounds,
    Point,
    SymmetricalX,
    CircularPattern,
)
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Circular, Polygonal, Rectangular
from gamegine.representation.robot import (
    PhysicalParameters,
    SwerveDrivetrainCharacteristics,
)
from gamegine.utils.unit import (
    Degree,
    Kilogram,
    KilogramMetersSquared,
    Meter,
    Centimeter,
    Feet,
    Inch,
    Pound,
    SpatialMeasurement,
)
from gamegine.analysis import pathfinding
import time


test_game = Game("FRC Crescendo 2024")

print("Name:", test_game.name)
test_game.set_field_size(Feet(54) + Inch(3.25), Feet(26) + Inch(11.25))
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
                (Inch(0), test_game.full_field_y()),
                (Inch(72.111), test_game.full_field_y()),
            ],
        ),
    ],
    test_game.half_field_x(),
    "Red ",
    "Blue ",
)


test_game.add_obstacles(objs)
test_game.enable_field_border_obstacles()

starting_points = []  # Points where the robot usually starts
points = [point.get_vertices()[0] for point in starting_points]

expanded_obstacles = ExpandedObjectBounds(
    test_game.get_obstacles(), robot_radius=Inch(24.075), discretization_quality=16
)
expanded_obstacles_v = ExpandedObjectBounds(
    test_game.get_obstacles(), robot_radius=Inch(25), discretization_quality=5
)
# map = VisibilityGraph(expanded_obstacles_v, points, test_game.field_size)
map = TriangulatedGraph(expanded_obstacles, Feet(4), test_game.get_field_size())


def CreatePath(
    start: Tuple[SpatialMeasurement, SpatialMeasurement],
    end: Tuple[SpatialMeasurement, SpatialMeasurement],
) -> pathfinding.Path:
    path = pathfinding.findPath(
        map,
        start,
        end,
        pathfinding.AStar,
        pathfinding.InitialConnectionPolicy.ConnectToClosest,
    )
    path.shortcut(expanded_obstacles)
    return path


corridors = []


def CreateTrajectory(
    start: Tuple[SpatialMeasurement, SpatialMeasurement],
    end: Tuple[SpatialMeasurement, SpatialMeasurement],
):
    path = CreatePath(start, end)

    trajectory_generator = SafetyCorridorAssisted(Centimeter(10))
    traj = trajectory_generator.calculate_trajectory(
        path,
        expanded_obstacles,
        TrajectoryKeypoint(),
        TrajectoryKeypoint(),
        PhysicalParameters(Pound(120), KilogramMetersSquared(5)),
        SwerveDrivetrainCharacteristics(),
    )
    corridors.extend(trajectory_generator.GetSafeCorridor())
    return traj


def Destinations(
    places: List[Tuple[SpatialMeasurement, SpatialMeasurement]]
) -> List[SwerveTrajectory]:
    out = []
    for i in range(len(places) - 1):
        start = places[i]
        end = places[i + 1]
        out.append(CreateTrajectory(start, end))
    return out


# safe_corridor = trajectory_generator.GetSafeCorridor()
trajectories = Destinations(
    [
        (Feet(6), Inch(64.081) + Inch(82.645) / 2),
        (Feet(36), Feet(3)),
    ]
)
renderer = Renderer()

renderer.set_game(test_game)
renderer.set_render_scale(Centimeter(1))
renderer.init_display()
print("Game set and display initialized")

while renderer.loop():

    renderer.draw_element(map)

    renderer.draw_elements(corridors)
    renderer.draw_elements(expanded_obstacles)
    renderer.draw_elements(trajectories)
    renderer.draw_static_elements()

    renderer.render_frame()
