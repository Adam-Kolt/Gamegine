import math
from typing import List, Tuple

import pint
import pygame
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
        # Circular("Note 1", Inch(114.010), Inch(47.638), Inch(7)),
        # Circular("Note 2", Inch(114.010), Inch(47.638) + Inch(43.000), Inch(7)),
        # Circular("Note 3", Inch(114.010), Inch(47.638) + Inch(43.000) * 2, Inch(7)),
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
    test_game.get_obstacles(),
    robot_radius=Inch(20),
    discretization_quality=16,  # 24.075
)
expanded_obstacles_block = [
    obstacle.get_bounded_rectangle() for obstacle in expanded_obstacles
]
# map = VisibilityGraph(expanded_obstacles_v, points, test_game.field_size)
map = TriangulatedGraph(expanded_obstacles_block, Feet(2), test_game.get_field_size())


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
paths = []


def CreateTrajectory(
    start: Tuple[SpatialMeasurement, SpatialMeasurement],
    end: Tuple[SpatialMeasurement, SpatialMeasurement],
):
    path = CreatePath(start, end)
    paths.append(path)
    trajectory_generator = SafetyCorridorAssisted(Centimeter(10))
    traj = trajectory_generator.calculate_trajectory(
        path,
        expanded_obstacles,
        TrajectoryKeypoint(),
        TrajectoryKeypoint(),
        PhysicalParameters(Pound(120), KilogramMetersSquared(5)),
        SwerveDrivetrainCharacteristics(),
    )
    states = traj.get_points()
    path = pathfinding.Path([(state.x, state.y) for state in states])
    corridors.clear()
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
Current = (Feet(6), Inch(64.081) + Inch(82.645) / 2)
trajectories = []
renderer = Renderer()

renderer.set_game(test_game)
renderer.set_render_scale(Centimeter(1))
renderer.init_display()
print("Game set and display initialized")

loop = True
while loop != False:
    loop = renderer.loop()
    ev = pygame.event.get()

    for event in loop:
        if event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            x, y = pos[0] * Renderer.render_scale, pos[1] * Renderer.render_scale

            trajectories.append(CreateTrajectory(Current, (x, y)))
            Current = (x, y)
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_c:
                trajectories = []

    renderer.draw_element(map)

    renderer.draw_elements(corridors)
    renderer.draw_elements(expanded_obstacles)
    renderer.draw_elements(trajectories)
    renderer.draw_static_elements()

    renderer.render_frame()
