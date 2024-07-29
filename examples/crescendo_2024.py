import math
from typing import Tuple

import pint
from gamegine.analysis.meshing import TriangulatedGraph, VisibilityGraph
from gamegine.analysis.trajectory.SafetyCorridorAssisted import SafetyCorridorAssisted
from gamegine.analysis.trajectory.generation import (
    SwerveDrivetrainParameters,
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
from gamegine.utils.unit import (
    Degree,
    Meter,
    Centimeter,
    Feet,
    Inch,
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
    test_game.get_obstacles(), robot_radius=Inch(24.075), discretization_quality=8
)
# map = VisibilityGraph(expanded_obstacles, points, test_game.field_size)
map = TriangulatedGraph(expanded_obstacles, Feet(2), test_game.get_field_size())


def CreatePath(
    start: Tuple[SpatialMeasurement, SpatialMeasurement],
    end: Tuple[SpatialMeasurement, SpatialMeasurement],
) -> pathfinding.Path:
    path = pathfinding.findPath(
        map,
        start,
        end,
        pathfinding.AStar,
        pathfinding.InitialConnectionPolicy.SnapToClosest,
    )
    path.shortcut(expanded_obstacles)
    return path


path_displays = [
    CreatePath((Feet(3), Feet(2)), (Feet(50), Feet(20))),
]

trajectory_generator = SafetyCorridorAssisted()
trajectory_generator.calculate_trajectory(
    path_displays[0],
    expanded_obstacles,
    TrajectoryKeypoint(),
    TrajectoryKeypoint(),
    SwerveDrivetrainParameters(),
)
safe_corridor = trajectory_generator.GetSafeCorridor()

renderer = Renderer()

renderer.set_game(test_game)
renderer.set_render_scale(Centimeter(1))
renderer.init_display()
print("Game set and display initialized")

while renderer.loop():
    renderer.draw_static_elements()
    renderer.draw_element(map)
    renderer.draw_elements(path_displays)
    renderer.draw_elements(safe_corridor)
    renderer.draw_elements(expanded_obstacles)

    time.sleep(0.1)
    renderer.render_frame()
