import time
from typing import Tuple

from gamegine.analysis import pathfinding
from gamegine.analysis.meshing import TriangulatedGraph
from gamegine.analysis.trajectory.SafetyCorridorAssisted import SafetyCorridorAssisted
from gamegine.analysis.trajectory.generation import TrajectoryKeypoint
from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import ExpandedObjectBounds
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Circular, Polygonal, Rectangular
from gamegine.representation.robot import (
    PhysicalParameters,
    SwerveDrivetrainCharacteristics,
)
from gamegine.utils.NCIM.ComplexDimensions.MOI import KilogramMetersSquared
from gamegine.utils.NCIM.Dimensions.mass import Pound
from gamegine.utils.NCIM.Dimensions.spatial import (
    Centimeter,
    Feet,
    Inch,
    SpatialMeasurement,
)

trajectory_test = Game("Trajectory Test")


ROBOT_RADIUS = Inch(20)

trajectory_test.set_field_size(Feet(50), Feet(50))

# Randomly Scattered Obstacles
obstacles = [
    Rectangular("Test Rectangle", Feet(1), Feet(1), Feet(2), Feet(2)),
    Circular("Test Circle", Feet(4), Feet(4), Feet(1)),
    Circular("Test Circle 2", Feet(10), Feet(10), Feet(1)),
    Circular("Test Circle 3", Feet(20), Feet(20), Feet(1)),
    Circular("Test Circle 4", Feet(30), Feet(30), Feet(1)),
    Circular("Test Circle 5", Feet(40), Feet(40), Feet(1)),
]


def CreatePath(
    start: Tuple[SpatialMeasurement, SpatialMeasurement],
    end: Tuple[SpatialMeasurement, SpatialMeasurement],
) -> pathfinding.Path:
    path = pathfinding.findPath(
        triangle_graph,
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
    return traj


trajectory_test.add_obstacles(obstacles)
trajectory_test.enable_field_border_obstacles()

expanded_obstacles = ExpandedObjectBounds(
    trajectory_test.get_obstacles(), ROBOT_RADIUS, 8
)

triangle_graph = TriangulatedGraph(
    expanded_obstacles, Feet(3), trajectory_test.get_field_size()
)

renderer = Renderer()

renderer.set_game(trajectory_test)
renderer.set_render_scale(Centimeter(1.5))
renderer.init_display()


trajectories = [
    CreateTrajectory((Feet(2), Feet(6)), (Feet(6), Feet(44))),
]

while renderer.loop() != False:
    renderer.draw_elements(expanded_obstacles)
    renderer.draw_static_elements()
    renderer.draw_element(triangle_graph)
    renderer.draw_elements(trajectories)

    renderer.render_frame()
    time.sleep(0.1)
