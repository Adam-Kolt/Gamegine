import time
from typing import Tuple

from gamegine.analysis import pathfinding
from gamegine.analysis.meshing import TriangulatedGraph
from gamegine.analysis.trajectory.SafetyCorridorAssisted import SafetyCorridorAssisted
from gamegine.analysis.trajectory.generation import TrajectoryKeypoint
from gamegine.analysis.trajectory.lib.TrajGen import (
    SolverConfig,
    SwerveRobotConstraints,
    SwerveTrajectoryProblemBuilder,
    TrajectoryBuilderConfig,
    TrajectoryProblemBuilder,
    TrajectoryRobotConstraints,
    Waypoint,
)
from gamegine.analysis.trajectory.lib.constraints.constraints import (
    AngleEquals,
    VelocityEquals,
    VelocityMagnitudeEquals,
)
from gamegine.reference import gearing, motors
from gamegine.reference.motors import MotorConfig
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.render.renderer import Renderer
from gamegine.representation.bounds import ExpandedObjectBounds
from gamegine.representation.game import Game
from gamegine.representation.obstacle import Circular, Polygonal, Rectangular
from gamegine.representation.robot import (
    PhysicalParameters,
    SwerveDrivetrainCharacteristics,
)
from gamegine.utils.NCIM.ComplexDimensions.MOI import KilogramMetersSquared
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.alpha import RadiansPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond
from gamegine.utils.NCIM.Dimensions.angular import Degree
from gamegine.utils.NCIM.Dimensions.current import Ampere
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

    builder = SwerveTrajectoryProblemBuilder()
    builder.waypoint(
        Waypoint(start[0], start[1]).given(
            VelocityEquals(MetersPerSecond(-2), MeterPerSecondSquared(3)),
            AngleEquals(Degree(0)),
        )
    )
    builder.waypoint(
        Waypoint(end[0], end[1]).given(
            VelocityEquals(MetersPerSecond(2), MeterPerSecondSquared(0)),
            AngleEquals(Degree(180)),
        )
    )

    trajectory = builder.generate(
        TrajectoryBuilderConfig(
            trajectory_resolution=Centimeter(15), stretch_factor=1.5
        )
    ).solve(
        SwerveRobotConstraints(
            MeterPerSecondSquared(5),
            MetersPerSecond(10),
            RadiansPerSecondSquared(3.14),
            RadiansPerSecond(3.14),
            SwerveConfig(
                module=SwerveModule(
                    MotorConfig(
                        motors.KrakenX60,
                        motors.PowerConfig(Ampere(40), Ampere(360), 1.0),
                    ),
                    gearing.MK4I.L1,
                    MotorConfig(
                        motors.KrakenX60,
                        motors.PowerConfig(Ampere(40), Ampere(360), 1.0),
                    ),
                    gearing.MK4I.L1,
                )
            ),
            physical_parameters=PhysicalParameters(
                mass=Pound(120),
                moi=KilogramMetersSquared(10),
            ),
        ),
        SolverConfig(),
    )

    return trajectory


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
    CreateTrajectory((Feet(6), Feet(20)), (Feet(20), Feet(6))),
    CreateTrajectory((Feet(20), Feet(6)), (Feet(20), Feet(20))),
]

while renderer.loop() != False:
    renderer.draw_elements(expanded_obstacles)
    renderer.draw_static_elements()
    renderer.draw_element(triangle_graph)
    renderer.draw_elements(trajectories)

    renderer.render_frame()
    time.sleep(0.1)
