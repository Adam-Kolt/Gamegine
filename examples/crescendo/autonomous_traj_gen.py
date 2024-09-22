from typing import List, Tuple
import pygame
from examples.crescendo.crescendo import Crescendo
from gamegine.analysis import pathfinding
from gamegine.analysis.meshing import TriangulatedGraph
from gamegine.analysis.trajectory.generation import SwerveTrajectory
from gamegine.analysis.trajectory.lib.TrajGen import (
    MinimizationStrategy,
    SolverConfig,
    SwerveRobotConstraints,
    SwerveTrajectoryProblemBuilder,
    TrajectoryBuilderConfig,
    Waypoint,
)
from gamegine.analysis.trajectory.lib.constraints.avoidance import SafetyCorridor
from gamegine.analysis.trajectory.lib.constraints.constraints import (
    AngleEquals,
    VelocityEquals,
)
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import ExpandedObjectBounds
from gamegine.representation.robot import PhysicalParameters
from gamegine.utils.NCIM.ComplexDimensions.MOI import PoundsInchesSquared
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.alpha import (
    RadiansPerSecondSquared,
    RotationsPerSecondSquared,
)
from gamegine.utils.NCIM.ComplexDimensions.omega import (
    RadiansPerSecond,
    RotationsPerSecond,
)
from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Degree
from gamegine.utils.NCIM.Dimensions.current import Ampere
from gamegine.utils.NCIM.Dimensions.mass import Pound
from gamegine.utils.NCIM.Dimensions.spatial import (
    Centimeter,
    Feet,
    Inch,
    SpatialMeasurement,
)


expanded_obstacles = ExpandedObjectBounds(
    Crescendo.get_obstacles(),
    robot_radius=Inch(20),
    discretization_quality=16,  # 24.075
)
slightly_more_expanded_obstacles = ExpandedObjectBounds(
    Crescendo.get_obstacles(),
    robot_radius=Inch(20) + Inch(2),
    discretization_quality=16,  # 24.075
)
expanded_obstacles_block = [
    obstacle.get_bounded_rectangle() for obstacle in expanded_obstacles
]
# map = VisibilityGraph(expanded_obstacles_v, points, Crescendo.field_size)
map = TriangulatedGraph(
    slightly_more_expanded_obstacles, Feet(2), Crescendo.get_field_size()
)


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

start = (Inch(35.695) + Inch(25), Inch(64.081) + Inch(20.825))
note_locations = [
    (Feet(50), Feet(3)),
    (Crescendo.half_field_x(), Crescendo.half_field_y()),
]

shot_location = (Inch(35.695) + Inch(30), Inch(64.081) + Inch(82.645) / 2)

# Auto Trajectory:


builder = SwerveTrajectoryProblemBuilder()

builder.waypoint(
    Waypoint(start[0], start[1]).given(
        VelocityEquals(MetersPerSecond(0), MetersPerSecond(0)),
        AngleEquals(Degree(180)),
    )
)
curr = start

pathes = []


for i, note in enumerate(note_locations):
    pathes.append(CreatePath(curr, note))

    builder.waypoint(
        Waypoint(note[0], note[1]).given(
            VelocityEquals(MetersPerSecond(0), MetersPerSecond(0)),
            AngleEquals(Degree(0)),
        )
    )

    pathes.append(CreatePath(note, shot_location))

    builder.waypoint(
        Waypoint(shot_location[0], shot_location[1]).given(
            VelocityEquals(MetersPerSecond(0), MetersPerSecond(0)),
            AngleEquals(Degree(0)),
        )
    )
    curr = shot_location


builder.guide_pathes(pathes)
builder.points_constraint(SafetyCorridor(expanded_obstacles))

trajectory = builder.generate(
    TrajectoryBuilderConfig(
        trajectory_resolution=Centimeter(15),
        stretch_factor=1.5,
        min_spacing=Centimeter(5),
        minimization_strategy=MinimizationStrategy.TIME,
    )
).solve(
    SwerveRobotConstraints(
        MeterPerSecondSquared(10),
        MetersPerSecond(10),
        RotationsPerSecond(3),
        RotationsPerSecondSquared(3),
        SwerveConfig(
            module=SwerveModule(
                motors.MotorConfig(
                    motors.KrakenX60,
                    motors.PowerConfig(Ampere(60), Ampere(360), 1.0),
                ),
                gearing.MK4I.L3,
                motors.MotorConfig(
                    motors.KrakenX60,
                    motors.PowerConfig(Ampere(60), Ampere(360), 1.0),
                ),
                gearing.MK4I.L3,
            )
        ),
        physical_parameters=PhysicalParameters(
            mass=Pound(110),
            moi=PoundsInchesSquared(21327.14),
        ),
    ),
    SolverConfig(timeout=5, max_iterations=1000, solution_tolerance=1e-4),
)


# safe_corridor = trajectory_generator.GetSafeCorridor()
Current = (Feet(6), Inch(64.081) + Inch(82.645) / 2)
trajectories = []
renderer = Renderer()

renderer.set_game(Crescendo)
renderer.set_render_scale(Centimeter(1))
renderer.init_display()
print("Game set and display initialized")

loop = True
while loop != False:
    loop = renderer.loop()

    renderer.draw_element(map)

    renderer.draw_elements(expanded_obstacles)
    renderer.draw_element(trajectory)
    renderer.draw_elements(paths)
    renderer.draw_static_elements()

    renderer.render_frame()
