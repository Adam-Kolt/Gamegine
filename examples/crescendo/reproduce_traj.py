from gamegine.analysis.trajectory.lib.TrajGen import (
    SolverConfig,
    MinimizationStrategy,
    SwerveRobotConstraints,
    SwerveTrajectoryProblemBuilder,
    TrajectoryBuilderConfig,
    Waypoint,
)
from gamegine.analysis.trajectory.lib.constraints.avoidance import SafetyCorridor
from gamegine.analysis.trajectory.lib.constraints.constraints import VelocityEquals
from gamegine.reference import gearing, motors
from gamegine.reference.swerve import SwerveConfig, SwerveModule
from gamegine.representation.bounds import ExpandedObjectBounds
from gamegine.representation.robot import PhysicalParameters
from gamegine.utils.NCIM.ComplexDimensions.MOI import PoundsInchesSquared
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.alpha import RadiansPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.omega import RadiansPerSecond
from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond
from gamegine.utils.NCIM.Dimensions.angular import Degree
from gamegine.utils.NCIM.Dimensions.current import Ampere
from gamegine.utils.NCIM.Dimensions.mass import Pound
from gamegine.utils.NCIM.Dimensions.spatial import Centimeter, Feet, Inch, Meter
# Mocking Crescendo and other dependencies if simpler, but importing should work
from examples.crescendo.crescendo import Crescendo
from gamegine.analysis import pathfinding
from gamegine.analysis.meshing import TriangulatedGraph

def run_test():
    expanded_obstacles = ExpandedObjectBounds(
        Crescendo.get_obstacles(),
        robot_radius=Inch(20),
        discretization_quality=16,
    )
    
    slightly_more_expanded_obstacles = ExpandedObjectBounds(
        Crescendo.get_obstacles(),
        robot_radius=Inch(20) + Inch(2),
        discretization_quality=16,
    )

    t_map = TriangulatedGraph(
        slightly_more_expanded_obstacles, Feet(2), Crescendo.get_field_size()
    )

    start = (Meter(1.46), Meter(1.25))
    # A destination that requires moving around obstacles
    end = (Meter(13.11), Meter(5.27)) 
    
    # Pathfinding
    path = pathfinding.findPath(
        t_map,
        start,
        end,
        pathfinding.AStar,
        pathfinding.InitialConnectionPolicy.ConnectToClosest,
    )
    path.shortcut(expanded_obstacles)
    
    print(f"Path found with {len(path.get_points())} points")

    # Trajectory Generation
    builder = SwerveTrajectoryProblemBuilder()
    builder.waypoint(
        Waypoint(start[0], start[1]).given(
            VelocityEquals(MetersPerSecond(0), MetersPerSecond(0)),
        )
    )
    builder.waypoint(
        Waypoint(end[0], end[1]).given(
            VelocityEquals(MetersPerSecond(0), MetersPerSecond(0)),
        )
    )
    builder.guide_pathes([path])
    
    # Pass the A* path to SafetyCorridor for proper corridor generation
    astar_path_points = path.get_points()
    builder.points_constraint(SafetyCorridor(expanded_obstacles, guide_path=astar_path_points))

    print("Generating trajectory...")
    import logging
    logging.basicConfig(level=logging.DEBUG, force=True)
    try:
        trajectory = builder.generate(
            TrajectoryBuilderConfig(
            trajectory_resolution=Centimeter(25),  # Increased from 15cm for more solver flexibility
                stretch_factor=1.5,
                min_spacing=Centimeter(5),
            )
        ).solve(
            SwerveRobotConstraints(
                MeterPerSecondSquared(8),
                MetersPerSecond(6),
                RadiansPerSecondSquared(3.14),
                RadiansPerSecond(3.14),
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
            SolverConfig(timeout=10, max_iterations=10000, solution_tolerance=1e-9),
        )
        print("Trajectory generated successfully.")
        print(f"Time: {trajectory.get_travel_time()}")
    except Exception as e:
        print(f"Trajectory generation failed: {e}")
        # If it returns a trajectory but with errors printed by the C++ backend (Sleipnir), 
        # python exception might not be raised unless we check status.
        # But let's see what happens.

if __name__ == "__main__":
    run_test()
