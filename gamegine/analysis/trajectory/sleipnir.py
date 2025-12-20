from typing import Any, Tuple

from gamegine.analysis import pathfinding
from gamegine.analysis.trajectory.generator import OfflineTrajectoryGenerator
from gamegine.analysis.trajectory.lib.TrajGen import (
    SolverConfig,
    SwerveRobotConstraints,
    SwerveTrajectory,
    SwerveTrajectoryProblemBuilder,
    TrajectoryBuilderConfig,
    Waypoint,
)
from gamegine.analysis.trajectory.lib.constraints.avoidance import SafetyCorridor
from gamegine.analysis.trajectory.lib.constraints.constraints import (
    AngleEquals,
    VelocityEquals,
)
from gamegine.representation.robot import SwerveRobot
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement
from gamegine.utils.NCIM.Dimensions.spatial import SpatialMeasurement, Centimeter
from gamegine.utils.NCIM.ncim import (
    MeterPerSecondSquared,
    MetersPerSecond,
    RadiansPerSecond,
    RadiansPerSecondSquared,
)

class SleipnirOfflineGenerator(OfflineTrajectoryGenerator):
    """Implementation of OfflineTrajectoryGenerator using Sleipnir optimization library."""

    def generate(
        self,
        robot_name: str,
        robot: SwerveRobot,
        start_state: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        target_state: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        path: pathfinding.Path,
        traversal_space: Any,
        constraints: Any = None,
        no_safety_corridor: bool = False
    ) -> SwerveTrajectory:
        start_x, start_y, start_heading = start_state
        x, y, heading = target_state

        builder = SwerveTrajectoryProblemBuilder()
        builder.waypoint(
            Waypoint(start_x, start_y).given(
                VelocityEquals(MetersPerSecond(0), MetersPerSecond(0)),
                AngleEquals(start_heading),
            )
        )
        builder.waypoint(
            Waypoint(x, y).given(
                VelocityEquals(MetersPerSecond(0), MetersPerSecond(0)),
                AngleEquals(heading),
            )
        )
        builder.guide_pathes([path])
        
        if not no_safety_corridor:
            builder.points_constraint(SafetyCorridor(traversal_space.obstacles))

        # Use passed constraints object for config if available, else default
        # Note: This hardcodes some default config values similar to original code
        # A more robust solution would pass a config object into the generator init or generate method
        trajectory_resolution = Centimeter(10)
        stretch_factor = 1.5
        min_spacing = Centimeter(8)
        
        if constraints and hasattr(constraints, 'trajectory_resolution'):
             trajectory_resolution = constraints.trajectory_resolution
        if constraints and hasattr(constraints, 'stretch_factor'):
             stretch_factor = constraints.stretch_factor
        if constraints and hasattr(constraints, 'min_spacing'):
             min_spacing = constraints.min_spacing

        return builder.generate(
            TrajectoryBuilderConfig(
                trajectory_resolution=trajectory_resolution,
                stretch_factor=stretch_factor,
                min_spacing=min_spacing,
            )
        ).solve(
            SwerveRobotConstraints(
                MeterPerSecondSquared(6.5),
                MetersPerSecond(6.1),
                RadiansPerSecondSquared(7.5),
                RadiansPerSecond(18),
                robot.get_drivetrain(),
                physical_parameters=robot.get_physics(),
            ),
            SolverConfig(timeout=10, max_iterations=10000, solution_tolerance=1e-9),
        )
