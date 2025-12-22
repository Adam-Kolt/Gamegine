from enum import Enum

import math
from typing import Callable, List
from sleipnir.optimization import Problem as OptimizationProblem
from gamegine.analysis.trajectory.lib.constraints import base
from gamegine.analysis.trajectory.lib.constraints.spacing import (
    PositionMaxSpacingConstraint,
    PositionMinSpacingConstraint,
)
from gamegine.analysis.trajectory.lib.constraints.swerve import (
    SwerveKinematicConstraints,
    SwerveModuleConstraints,
)
from gamegine.render import helpers
from gamegine.render.style import Palette
import pygame
from dataclasses import dataclass, field

from gamegine.analysis.pathfinding import Path
from gamegine.analysis.trajectory.lib import (
    CALCULATION_UNIT_ANGULAR,
    CALCULATION_UNIT_SPATIAL,
    CALCULATION_UNIT_TEMPORAL,
)
from gamegine.analysis.trajectory.lib.constraints.constraints import (
    AccelerationLessThan,
    AngularAccelerationLessThan,
    AngularVelocityLessThan,
    PositionEquals,
    VelocityLessThan,
)
from gamegine.analysis.trajectory.lib.constraints.kinematics import (
    OmegaKinematicsConstraint,
    PositionKinematicsConstraint,
    ThetaKinematicsConstraint,
    VelocityKinematicsConstraint,
)
from gamegine.analysis.trajectory.lib.problemVariables import (
    PointVariables,
    SwervePointVariables,
)
from gamegine.analysis.trajectory.lib.trajectoryStates import TrajectoryState
from gamegine.reference.swerve import SwerveConfig
from gamegine.render.drawable import Drawable
from gamegine.representation.robot import PhysicalParameters
from gamegine.utils import logging
from gamegine.utils.NCIM.ComplexDimensions.acceleration import (
    Acceleration,
    AccelerationUnit,
    MeterPerSecondSquared,
)
from gamegine.utils.NCIM.ComplexDimensions.alpha import (
    Alpha,
    AlphaUnit,
    RadiansPerSecondSquared,
)
from gamegine.utils.NCIM.ComplexDimensions.omega import (
    Omega,
    OmegaUnit,
    RadiansPerSecond,
)
from gamegine.utils.NCIM.ComplexDimensions.velocity import (
    MetersPerSecond,
    Velocity,
    VelocityUnit,
)
from gamegine.utils.NCIM.Dimensions.angular import Radian
from gamegine.utils.NCIM.Dimensions.spatial import (
    Centimeter,
    Inch,
    Meter,
    SpatialMeasurement,
)
from sortedcontainers import SortedDict

from gamegine.utils.NCIM.Dimensions.temporal import Second, TemporalMeasurement
from gamegine.utils.NCIM.ncim import RatioOf


@dataclass
class TrajectoryRobotConstraints:
    """Dataclass used to store robot constraints for a trajectory optimization problem.

    :param max_acceleration: The maximum acceleration of the robot.
    :type max_acceleration: :class:`Acceleration`
    :param max_velocity: The maximum velocity of the robot.
    :type max_velocity: :class:`Velocity`
    :param max_angular_acceleration: The maximum angular acceleration of the robot.
    :type max_angular_acceleration: :class:`Alpha`
    :param max_angular_velocity: The maximum angular velocity of the robot.
    :type max_angular_velocity: :class:`Omega`"""

    max_acceleration: Acceleration = MeterPerSecondSquared(0)
    max_velocity: Velocity = MetersPerSecond(0)
    max_angular_acceleration: Alpha = RadiansPerSecondSquared(0)
    max_angular_velocity: Omega = RadiansPerSecond(0)


@dataclass
class SwerveRobotConstraints(TrajectoryRobotConstraints):
    """Dataclass used to store swerve drive robot constraints for a trajectory optimization problem.

    :param swerve_config: The configuration of the swerve drive.
    :type swerve_config: :class:`SwerveConfig`
    :param physical_parameters: The physical parameters of the robot.
    :type physical_parameters: :class:`PhysicalParameters`"""

    swerve_config: SwerveConfig = None
    physical_parameters: PhysicalParameters = None


@dataclass
class SolverConfig:
    """Dataclass used to store configuration information for a trajectory optimization solver. This decides how accurately the solver attempts to solve the problem, in addition to allowing for a time limit on the solver.

    :param solution_tolerance: The tolerance of the solution.
    :type solution_tolerance: float
    :param max_iterations: The maximum number of iterations the solver will run.
    :type max_iterations: int
    :param timeout: The maximum time the solver will run before stopping.
    :type timeout: float"""

    solution_tolerance: float = 1e-2
    max_iterations: int = 1000
    timeout: float = 100.0


class Trajectory(Drawable):
    """Class used to store a trajectory generated from an optimization problem. Contains information about the trajectory, including length, time, and robot constraints.

    :param points: The points along the trajectory.
    :type points: List[:class:`TrajectoryState`]
    :param robot_constraints: The constraints of the robot.
    :type robot_constraints: :class:`TrajectoryRobotConstraints`"""

    def __compute_trajectory_parameters(self):
        self.travel_time = sum(
            [point.dt for point in self.points[:-1]], start=Second(0)
        )
        self.path_length = Meter(0)
        max_velocity = max(
            [point.get_velocity_magnitude() for point in self.points[:-1]]
        )
        max_acceleration = max(
            [point.get_acceleration_magnitude() for point in self.points[:-1]]
        )
        for i in range(len(self.points) - 1):
            point = self.points[i]
            point2 = self.points[i + 1]
            self.path_length += (
                (point2.x - point.x) ** 2 + (point2.y - point.y) ** 2
            ) ** 0.5

        logging.Info(
            f"Computed trajectory parameters: {self.get_length()} length, {self.get_travel_time()} time, {max_velocity} max velocity, {max_acceleration} max acceleration."
        )

    def __init__(
        self,
        points: List[TrajectoryState],
        robot_constraints: TrajectoryRobotConstraints,
    ):
        self.robot_constraints = robot_constraints
        self.points = points
        self.travel_time = 0
        self.path_length = 0
        self.__compute_trajectory_parameters()
        pass

    def get_travel_time(self) -> TemporalMeasurement:
        """Returns the total travel time of the trajectory.

        :return: The total travel time of the trajectory.
        :rtype: :class:`TemporalMeasurement`"""
        return self.travel_time

    def get_length(self) -> SpatialMeasurement:
        """Returns the total length of the trajectory.

        :return: The total length of the trajectory.
        :rtype: :class:`SpatialMeasurement`"""
        return self.path_length

    def get_at_time(self, time: TemporalMeasurement) -> TrajectoryState:
        """Returns the state of the robot at a given time.

        :param time: The time at which to get the state of the robot.
        :type time: :class:`TemporalMeasurement`
        :return: The state of the robot at the given time.
        :rtype: :class:`TrajectoryState`"""
        pass

    def get_robot_constraints(self) -> TrajectoryRobotConstraints:
        """Returns the robot constraints of the trajectory.

        :return: The robot constraints of the trajectory.
        :rtype: :class:`TrajectoryRobotConstraints`"""
        return self.robot_constraints

    def export_to_file(self, file_path: str):
        # TODO: Change to new Choreo Format
        """Exports the trajectory to a JSON file, formatted in the Choreo structure.

        :param file_path: The path to the file to export the trajectory to.
        :type file_path: str"""

        with open(file_path, "w") as file:
            file.write(
                """
                       {\n
                       "samples": ["""
            )
            curr_time = Second(0)
            for i in range(len(self.points)):
                point = self.points[i]

                file.write(
                    f"""
                    {{
                        "x": {point.x.to(Meter)},
                        "y": {point.y.to(Meter)},
                        "heading": {point.theta.to(Radian)},
                        "angularVelocity": {point.omega.to(RadiansPerSecond)},
                        "velocityX": {point.vel_x.to(MetersPerSecond)},
                        "velocityY": {point.vel_y.to(MetersPerSecond)},
                        "timestamp": {curr_time.to(Second)}
                    }}"""
                )
                curr_time += point.dt
                if i < len(self.points) - 1:
                    file.write(",")

            file.write(
                """
                ]
            }"""
            )

    def __str__(self) -> str:
        return f"Trajectory: {len(self.points)} points, {self.get_length()} length, {self.get_travel_time()} time. Optimized for {self.robot_constraints}."

    def draw(self, render_scale: SpatialMeasurement):
        """Draws the trajectory on the screen.

        :param render_scale: The scale at which to render the trajectory.
        :type render_scale: :class:`SpatialMeasurement`"""

        for i in range(len(self.points) - 1):
            point = self.points[i]
            point2 = self.points[i + 1]
            pygame.draw.line(
                pygame.display.get_surface(),
                (255, 100, 0),
                (RatioOf(point.x, render_scale), RatioOf(point.y, render_scale)),
                (RatioOf(point2.x, render_scale), RatioOf(point2.y, render_scale)),
                width=int(RatioOf(Inch(2), render_scale)),
            )

        helpers.draw_point(point2.x, point2.y, Inch(2), Palette.PINK, render_scale)
        helpers.draw_point(
            self.points[0].x,
            self.points[0].y,
            Inch(2),
            Palette.PINK,
            render_scale,
        )


class SwerveTrajectory(Trajectory):
    """Class used to store a swerve drive trajectory generated from an optimization problem. Contains information about the trajectory, including length, time, and robot constraints.

    :param points: The points along the trajectory.
    :type points: List[:class:`TrajectoryState`]
    :param robot_constraints: The constraints of the robot.
    :type robot_constraints: :class:`SwerveRobotConstraints`"""

    def __init__(
        self,
        points: List[TrajectoryState],
        robot_constraints: SwerveRobotConstraints,
    ):
        super().__init__(points, robot_constraints)

        pass

    def get_robot_constraints(self) -> SwerveRobotConstraints:
        return self.robot_constraints

    def __str__(self) -> str:
        return f"Swerve Trajectory: {len(self.points)} points, {self.get_length()} length, {self.get_travel_time()} time. Optimized for {self.robot_constraints}."

    def interpolate_trajectory_states(
        self, state1: TrajectoryState, state2: TrajectoryState, t: TemporalMeasurement
    ) -> TrajectoryState:
        """Interpolates between two trajectory states based on a given time.

        :param state1: The first trajectory state to interpolate between.
        :type state1: :class:`Trajectory
        :param state2: The second trajectory state to interpolate between.
        :type state2: :class:`Trajectory
        :param t: The time at which to interpolate between the two states.
        :type t: :class:`TemporalMeasurement`
        :return: The interpolated trajectory state.
        :rtype: :class:`TrajectoryState`"""

        # Use Kinematic Interpolation (s = ut + 0.5at^2)
        # We assume constant acceleration over the interval (as modeled by the solver)
        dt_val = t
        
        # Acceleration to use for this interval
        # Ideally state1.acc should be used.
        curr_acc_x = state1.acc_x
        curr_acc_y = state1.acc_y
        curr_alpha = state1.alpha
        
        # Fallback if acceleration is missing (e.g. last point)
        if curr_acc_x is None:
             curr_acc_x = Acceleration(0, AccelerationUnit(CALCULATION_UNIT_SPATIAL, CALCULATION_UNIT_TEMPORAL))
             curr_acc_y = Acceleration(0, AccelerationUnit(CALCULATION_UNIT_SPATIAL, CALCULATION_UNIT_TEMPORAL))
             curr_alpha = Alpha(0, AlphaUnit(CALCULATION_UNIT_ANGULAR, CALCULATION_UNIT_TEMPORAL))

        # Position
        x = state1.x + state1.vel_x * dt_val + 0.5 * curr_acc_x * dt_val**2
        y = state1.y + state1.vel_y * dt_val + 0.5 * curr_acc_y * dt_val**2
        theta = state1.theta + state1.omega * dt_val + 0.5 * curr_alpha * dt_val**2
        
        # Velocity
        vel_x = state1.vel_x + curr_acc_x * dt_val
        vel_y = state1.vel_y + curr_acc_y * dt_val
        omega = state1.omega + curr_alpha * dt_val
        
        # Acceleration (Constant over interval)
        acc_x = curr_acc_x
        acc_y = curr_acc_y
        alpha = curr_alpha
        
        dt = t

        return TrajectoryState(
            x, y, theta, vel_x, vel_y, acc_x, acc_y, omega, alpha, dt
        )

    def get_at_time(self, time):
        cur = Second(0)
        for i in range(len(self.points) - 1):
            point = self.points[i]
            point2 = self.points[i + 1]
            if point.dt + cur > time:
                return self.interpolate_trajectory_states(point, point2, time - cur)
            cur += point.dt
        return self.points[-1]

    def draw(self, render_scale: SpatialMeasurement):
        module_points = [
            self.robot_constraints.swerve_config.top_left_offset,
            self.robot_constraints.swerve_config.top_right_offset,
            self.robot_constraints.swerve_config.bottom_right_offset,
            self.robot_constraints.swerve_config.bottom_left_offset,
        ]

        for i in range(len(self.points) - 1):
            point = self.points[i]
            point2 = self.points[i + 1]
            pygame.draw.line(
                pygame.display.get_surface(),
                (255, 100, 0),
                (RatioOf(point.x, render_scale), RatioOf(point.y, render_scale)),
                (RatioOf(point2.x, render_scale), RatioOf(point2.y, render_scale)),
                width=int(RatioOf(Inch(2), render_scale)),
            )
            angle = point.theta

            colors = [
                Palette.RED,
                Palette.ORANGE,
                Palette.BLUE,
                Palette.YELLOW,
            ]
            field_relative_module_points = []
            for i, offset in enumerate(module_points):
                cos = math.cos(angle.to(Radian))
                sin = math.sin(angle.to(Radian))
                x = point.x + offset[0] * cos - offset[1] * sin
                y = point.y + offset[0] * sin + offset[1] * cos
                field_relative_module_points.append((x, y))
                helpers.draw_point(x, y, Inch(1), colors[i], render_scale)

            # for i, module_point in enumerate(field_relative_module_points[:-1]):
            #     pygame.draw.line(
            #         pygame.display.get_surface(),
            #         colors[i].get_color_array(),
            #         (
            #             RatioOf(field_relative_module_points[i][0], render_scale),
            #             RatioOf(field_relative_module_points[i][1], render_scale),
            #         ),
            #         (
            #             RatioOf(field_relative_module_points[i + 1][0], render_scale),
            #             RatioOf(field_relative_module_points[i + 1][1], render_scale),
            #         ),
            #         width=int(RatioOf(Inch(2), render_scale)),
            #     )

            # pygame.draw.line(
            #     pygame.display.get_surface(),
            #     colors[3].get_color_array(),
            #     (
            #         RatioOf(field_relative_module_points[3][0], render_scale),
            #         RatioOf(field_relative_module_points[3][1], render_scale),
            #     ),
            #     (
            #         RatioOf(field_relative_module_points[0][0], render_scale),
            #         RatioOf(field_relative_module_points[0][1], render_scale),
            #     ),
            #     width=int(RatioOf(Inch(2), render_scale)),
            # )

            # Draw the robot center
            helpers.draw_point(point.x, point.y, Inch(2), Palette.PINK, render_scale)

            # Draw the robot heading
            pygame.draw.line(
                pygame.display.get_surface(),
                Palette.PINK.get_color_array(),
                (
                    RatioOf(point.x, render_scale),
                    RatioOf(point.y, render_scale),
                ),
                (
                    RatioOf(
                        point.x + Inch(6) * math.cos(point.theta.to(Radian)),
                        render_scale,
                    ),
                    RatioOf(
                        point.y + Inch(6) * math.sin(point.theta.to(Radian)),
                        render_scale,
                    ),
                ),
                width=int(RatioOf(Inch(1), render_scale)),
            )


class TrajectoryProblem:
    """Class for defining the optimization problem for a trajectory. Contains the problem and the point variables used in the problem.

    :param intialized_problem: The optimization problem to solve.
    :type intialized_problem: :class:`OptimizationProblem`
    :param point_vars: The state variables at all discrete points in the trajectory.
    :type point_vars: :class:`PointVariables`"""

    def __init__(
        self, intialized_problem: OptimizationProblem, point_vars: PointVariables
    ):
        self.point_vars = point_vars
        self.problem = intialized_problem

        pass

    def set_solver_callback(self, callback: Callable):
        """Sets a callback function to be called for each solver iteration.
        
        The callback receives an IterationInfo object and should return True to stop early,
        False to continue.

        :param callback: The callback function (Callable[[IterationInfo], bool]).
        :type callback: Callable"""
        self.problem.add_callback(callback)

    def apply_constraints(self, robot_constraints: TrajectoryRobotConstraints):
        """Applies constraints to the optimization problem based on the robot constraints.

        :param robot_constraints: The constraints of the robot.
        :type robot_constraints: :class:`TrajectoryRobotConstraints`"""

        AccelerationLessThan(robot_constraints.max_acceleration)(
            self.problem, self.point_vars
        )

        VelocityLessThan(robot_constraints.max_velocity)(self.problem, self.point_vars)

        AngularVelocityLessThan(robot_constraints.max_angular_velocity)(
            self.problem, self.point_vars
        )

        AngularAccelerationLessThan(robot_constraints.max_angular_acceleration)(
            self.problem, self.point_vars
        )

    @staticmethod
    def get_trajectory_states_from_vars(vars) -> List[TrajectoryState]:
        """Returns a list of trajectory states from the point variables.

        :param vars: The point variables to get the trajectory states from.
        :type vars: :class:`PointVariables`
        :return: A list of trajectory states from the point variables.
        :rtype: List[:class:`TrajectoryState`]"""

        VEL_UNIT = VelocityUnit(CALCULATION_UNIT_SPATIAL, CALCULATION_UNIT_TEMPORAL)
        ACCEL_UNIT = AccelerationUnit(
            CALCULATION_UNIT_SPATIAL, CALCULATION_UNIT_TEMPORAL
        )
        ALPHA_UNIT = AlphaUnit(CALCULATION_UNIT_ANGULAR, CALCULATION_UNIT_TEMPORAL)
        OMEGA_UNIT = OmegaUnit(CALCULATION_UNIT_ANGULAR, CALCULATION_UNIT_TEMPORAL)

        states = []

        for i in range(len(vars.ACCEL_X)):

            states.append(
                TrajectoryState(
                    x=CALCULATION_UNIT_SPATIAL(vars.POS_X.value(i)),
                    y=CALCULATION_UNIT_SPATIAL(vars.POS_Y.value(i)),
                    theta=CALCULATION_UNIT_ANGULAR(vars.THETA.value(i)),
                    vel_x=VEL_UNIT(vars.VEL_X.value(i)),
                    vel_y=VEL_UNIT(vars.VEL_Y.value(i)),
                    acc_x=ACCEL_UNIT(vars.ACCEL_X.value(i)),
                    acc_y=ACCEL_UNIT(vars.ACCEL_Y.value(i)),
                    omega=OMEGA_UNIT(vars.OMEGA.value(i)),
                    alpha=ALPHA_UNIT(vars.ALPHA.value(i)),
                    dt=CALCULATION_UNIT_TEMPORAL(vars.DT.value(i)),
                )
            )

        final_pos = len(vars.POS_X) - 1
        # Add final state
        states.append(
            TrajectoryState(
                x=CALCULATION_UNIT_SPATIAL(vars.POS_X.value(final_pos)),
                y=CALCULATION_UNIT_SPATIAL(vars.POS_Y.value(final_pos)),
                theta=CALCULATION_UNIT_ANGULAR(vars.THETA.value(final_pos)),
                vel_x=VEL_UNIT(vars.VEL_X.value(final_pos)),
                vel_y=VEL_UNIT(vars.VEL_Y.value(final_pos)),
                acc_x=None,
                acc_y=None,
                omega=OMEGA_UNIT(vars.OMEGA.value(final_pos)),
                alpha=None,
                dt=None,
            )
        )

        return states

    def get_trajectory_states(self) -> List[TrajectoryState]:
        """Returns a list of trajectory states from the point variables.

        :return: A list of trajectory states from the point variables.
        :rtype: List[:class:`TrajectoryState`]"""
        return self.get_trajectory_states_from_vars(self.point_vars)

    def solve(
        self, robot_constraints: TrajectoryRobotConstraints, config: SolverConfig
    ) -> Trajectory:
        """Solves the optimization problem and returns the solution.

        :param robot_constraints: The constraints of the robot.
        :type robot_constraints: :class:`TrajectoryRobotConstraints`
        :param config: The configuration of the solver.
        :type config: :class:`SolverConfig`
        :return: The solution to the optimization problem.
        :rtype: :class:`Trajectory`"""
        self.apply_constraints(robot_constraints)

        self.problem.solve(
            tolerance=config.solution_tolerance,
            max_iterations=config.max_iterations,
            timeout=config.timeout,
        )

        return Trajectory(self.get_trajectory_states(), robot_constraints)


class SwerveTrajectoryProblem(TrajectoryProblem):
    """Class for defining the optimization problem for a swerve drive trajectory. Contains the problem and the point variables used in the problem.

    :param problem: The optimization problem to solve.
    :type problem: :class:`OptimizationProblem`
    :param point_vars: The state variables at all discrete points in the trajectory.
    :type point_vars: :class:`SwervePointVariables`"""

    def __init__(self, problem: OptimizationProblem, point_vars: SwervePointVariables):
        super().__init__(problem, point_vars)
        pass

    def apply_swerve_constraints(self, robot_constraints: SwerveRobotConstraints):
        """Applies constraints to the optimization problem based on the swerve drive robot constraints.

        :param robot_constraints: The constraints of the robot.
        :type robot_constraints: :class:`SwerveRobotConstraints`"""

        # FIXME: This screws up the solvers feasability
        SwerveModuleConstraints(
            robot_constraints.swerve_config, robot_constraints.physical_parameters
        )(self.problem, self.point_vars)

        SwerveKinematicConstraints(
            robot_constraints.swerve_config, robot_constraints.physical_parameters
        )(self.problem, self.point_vars)

    def solve(self, robot_constraints: SwerveRobotConstraints, config: SolverConfig, 
              iteration_callback: Callable[[List], None] = None):
        """Solves the optimization problem and returns the solution.
        
        :param iteration_callback: Optional callback called each iteration with current positions.
                                   Signature: callback(positions: List[Tuple[float, float]]) -> None
        """

        self.apply_constraints(robot_constraints)
        # TODO: FIX SWERVE CONSTRAINTS
        self.apply_swerve_constraints(robot_constraints)
        
        # Register iteration callback if provided
        if iteration_callback is not None:
            point_vars = self.point_vars
            iteration_count = [0]  # Mutable counter for throttling
            
            def sleipnir_callback(info) -> bool:
                iteration_count[0] += 1
                # Throttle: only call user callback every 10 iterations
                if iteration_count[0] % 1 != 0:
                    return False
                
                # Extract current positions from point variables
                positions = []
                for i in range(len(point_vars.POS_X)):
                    x = point_vars.POS_X[i].value()
                    y = point_vars.POS_Y[i].value()
                    positions.append((x, y))
                
                try:
                    iteration_callback(positions)
                except Exception as e:
                    logging.Warn(f"Iteration callback error: {e}")
                    
                return False  # Continue solving
            
            self.problem.add_callback(sleipnir_callback)

        status = self.problem.solve(
            tolerance=config.solution_tolerance,
            max_iterations=config.max_iterations,
            timeout=config.timeout,
            diagnostics=True,  # DIAGNOSTICS
        )

        return SwerveTrajectory(self.get_trajectory_states(), robot_constraints)


@dataclass
class Waypoint:
    """Dataclass used to store a waypoint that the trajectory must pass through.

    :param x: The x position of the waypoint.
    :type x: :class:`SpatialMeasurement`
    :param y: The y position of the waypoint.
    :type y: :class:`SpatialMeasurement`"""

    x: SpatialMeasurement
    y: SpatialMeasurement
    constraints: List[Callable[[OptimizationProblem, PointVariables, int], None]] = (
        field(default_factory=list)
    )
    control_point_index: int = -1

    def given(
        self,
        *constraints: List[Callable[[OptimizationProblem, PointVariables, int], None]],
    ) -> "Waypoint":
        """Adds constraints to the waypoint.

        :param constraints: The constraints to add to the waypoint.
        :type constraints: List[Callable[[OptimizationProblem, PointVariables, int], None]]
        :return: The waypoint with the added constraints.
        :rtype: :class:`Waypoint`"""
        self.constraints.extend(constraints)
        return self

    def __post_init__(self):
        if self.x <= 0:
            raise ValueError("Waypoint x must be positive.")
        if self.y <= 0:
            raise ValueError("Waypoint y must be positive.")

        self.constraints.append(PositionEquals(self.x, self.y))


class MinimizationStrategy(Enum):
    """Enum used to define the minimization strategy for a trajectory optimization problem.

    :param TIME: Minimize the time taken to complete the trajectory.
    :param DISTANCE: Minimize the distance traveled in the trajectory."""

    TIME = 0
    DISTANCE = 1
    SMOOTHNESS = 2


@dataclass
class TrajectoryBuilderConfig:
    """Dataclass used to store configuration information for a trajectory optimization problem.

    :param trajectory_resolution: The resolution of the trajectory.
    :type trajectory_resolution: :class:`SpatialMeasurement`
    :param stretch_factor: Factor which controls how much the trajectory can "stretch" from its initial path.
    :type stretch_factor: float
    :param min_spacing: The minimum spacing between points in the trajectory, can help space out points and keep them from clumping up.
    :type min_spacing: :class:`SpatialMeasurement`
    :param apply_kinematic_constraints: Whether to apply kinematic constraints to the trajectory.
    :type apply_kinematic_constraints: bool
    :param minimization_strategy: The minimization strategy for the trajectory.
    :type minimization_strategy: :class:`MinimizationStrategy`"""

    trajectory_resolution: SpatialMeasurement = Centimeter(10)
    stretch_factor: float = 1.1
    min_spacing: SpatialMeasurement = Centimeter(1)
    apply_kinematic_constraints: bool = True
    minimization_strategy: MinimizationStrategy = MinimizationStrategy.TIME


class TrajectoryProblemBuilder:
    """Class used to build a trajectory optimization problem and generate a solution. Allows for constraints and objectives to be added and setup.

    :param initial_pathes: The initial pathes to use for the trajectory.
    :type initial_pathes: List[:class:`Path`]
    :param point_constraints: The constraints that apply to all points in the trajectory.
    :type point_constraints: List[Callable[[OptimizationProblem, PointVariables], None]
    """

    def __init__(self) -> None:
        self.problem = None
        self.point_vars: PointVariables = None
        self.waypoints: SortedDict[int, Waypoint] = SortedDict()
        self.initial_pathes = []
        self.point_constraints = []

        pass

    def generate_point_vars(self, num_points: int):
        """Initializes the state variables for all points in the trajectory.

        :param num_points: The number of points in the trajectory.
        :type num_points: int
        :return: The state variables at all discrete points in the trajectory.
        :rtype: :class:`PointVariables`"""
        return PointVariables.with_initial_variables(self.problem, num_points)

    def initialize_state_variables_with_initial_pathes(
        self, initial_pathes: List[Path], resolution: SpatialMeasurement
    ) -> int:
        """Initializes the state variables for all points in the trajectory based on the initial pathes.

        :param initial_pathes: The initial pathes to use for the trajectory.
        :type initial_pathes: List[:class:`Path`]
        :param resolution: The resolution of the trajectory.
        :type resolution: :class:`SpatialMeasurement`
        :return: The number of points in the trajectory.
        :rtype: int"""

        X_nodes = []
        Y_nodes = []

        waypoints = self.waypoints.items()

        # Set the control node for first waypoint
        waypoints[0][1].control_point_index = 0

        for i, (order, waypoint) in enumerate(waypoints[:-1]):
            if i < len(self.initial_pathes):
                path = self.initial_pathes[i]
            else:
                path = Path(
                    [
                        (waypoint.x, waypoint.y),
                        (waypoints[i + 1][1].x, waypoints[i + 1][1].y),
                    ]
                )
            if i == 0:
                dissected_path = Path(
                    path.dissected(units_per_node=resolution).get_points()
                )
            else:
                dissected_path = Path(
                    path.dissected(units_per_node=resolution).get_points()[1:]
                )  # TODO: Do this better. This is done to ensure first point is not in same position as last point of previous path, which cooks the solver

            for node in dissected_path.get_points():
                X_nodes.append(node[0].to(CALCULATION_UNIT_SPATIAL))
                Y_nodes.append(node[1].to(CALCULATION_UNIT_SPATIAL))

            waypoints[i + 1][1].control_point_index = len(X_nodes) - 1

        self.point_vars = self.generate_point_vars(len(X_nodes))

        # Initialize time steps
        init_dt = (resolution / MetersPerSecond(5)).to(CALCULATION_UNIT_TEMPORAL)
        for i in range(len(X_nodes) - 1):
            self.point_vars.DT[i].set_value(init_dt)

        # Initialize state variables to initial paths
        for i in range(len(X_nodes)):
            self.point_vars.POS_X[i].set_value(X_nodes[i])
            self.point_vars.POS_Y[i].set_value(Y_nodes[i])

        # === KINEMATICALLY-CONSISTENT INITIAL GUESS ===
        # Uses cubic spline fitting + trapezoidal velocity profile
        n = len(X_nodes)
        
        # Debug: Print first and last few positions to verify ordering
        logging.Debug(f"Initial guess: {n} points")
        if n > 0:
            logging.Debug(f"  First 3: {[(X_nodes[i], Y_nodes[i]) for i in range(min(3, n))]}")
            logging.Debug(f"  Last 3: {[(X_nodes[i], Y_nodes[i]) for i in range(max(0, n-3), n)]}")
        
        # Kinematic limits
        max_vel = 3.0  # m/s
        max_accel = 2.5  # m/s^2
        min_curvature_radius = 0.3  # meters - minimum turning radius
        
        try:
            from scipy.interpolate import CubicSpline
            import numpy as np
            
            # Step 1: Fit cubic spline through path points
            # Create parameter t based on cumulative arc length
            x_arr = np.array(X_nodes)
            y_arr = np.array(Y_nodes)
            
            # Compute cumulative arc length for parameterization
            diffs = np.sqrt(np.diff(x_arr)**2 + np.diff(y_arr)**2)
            arc_lengths = np.concatenate([[0], np.cumsum(diffs)])
            total_length = arc_lengths[-1]
            
            if total_length < 0.01:
                # Path too short, use zero velocity fallback
                raise ValueError("Path too short for spline fitting")
            
            # Normalize to [0, 1] for spline parameter
            t_param = arc_lengths / total_length
            
            # Fit cubic splines for x and y
            cs_x = CubicSpline(t_param, x_arr, bc_type='natural')
            cs_y = CubicSpline(t_param, y_arr, bc_type='natural')
            
            # Step 2: Compute derivatives at each point
            # First derivative (tangent/velocity direction)
            dx_dt = cs_x(t_param, 1)  # First derivative
            dy_dt = cs_y(t_param, 1)
            
            # Second derivative (for curvature)
            d2x_dt2 = cs_x(t_param, 2)
            d2y_dt2 = cs_y(t_param, 2)
            
            # Step 3: Compute curvature at each point
            # κ = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
            speed_squared = dx_dt**2 + dy_dt**2
            speed = np.sqrt(speed_squared)
            speed_cubed = speed_squared * speed
            
            # Avoid division by zero
            speed_cubed = np.maximum(speed_cubed, 1e-6)
            curvature = np.abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / speed_cubed
            
            # Step 4: Compute curvature-limited max velocity at each point
            # v_max_curvature = sqrt(a_centripetal_max / curvature)
            # Using a_centripetal = v^2 / R = v^2 * curvature
            centripetal_limit = max_accel * 0.5  # Reserve some accel budget for tangential
            curvature_safe = np.maximum(curvature, 1.0 / min_curvature_radius)
            v_curvature_limit = np.sqrt(centripetal_limit / curvature_safe)
            v_curvature_limit = np.minimum(v_curvature_limit, max_vel)
            
            # Endpoints must have zero velocity
            v_curvature_limit[0] = 0
            v_curvature_limit[-1] = 0
            
            # Step 5: Forward pass - respect acceleration limits
            segment_lengths = diffs
            forward_vel = np.zeros(n)
            for i in range(1, n):
                v_prev = forward_vel[i-1]
                ds = segment_lengths[i-1]
                # v^2 = v_prev^2 + 2*a*ds
                v_max_from_accel = np.sqrt(v_prev**2 + 2 * max_accel * ds)
                forward_vel[i] = min(v_curvature_limit[i], v_max_from_accel)
            
            # Step 6: Backward pass - respect deceleration limits
            backward_vel = np.zeros(n)
            backward_vel[-1] = 0
            for i in range(n-2, -1, -1):
                v_next = backward_vel[i+1]
                ds = segment_lengths[i]
                v_max_from_decel = np.sqrt(v_next**2 + 2 * max_accel * ds)
                backward_vel[i] = min(forward_vel[i], v_max_from_decel)
            
            velocities = backward_vel
            
            # Step 7: Compute velocity vectors (tangent direction * speed)
            # Normalize tangent vectors
            tangent_x = dx_dt / np.maximum(speed, 1e-6)
            tangent_y = dy_dt / np.maximum(speed, 1e-6)
            
            vel_x = velocities * tangent_x
            vel_y = velocities * tangent_y
            
            # Step 8: Compute time steps from velocity
            dt_values = np.zeros(n-1)
            for i in range(n-1):
                ds = segment_lengths[i]
                avg_v = (velocities[i] + velocities[i+1]) / 2
                if avg_v > 0.01:
                    dt_values[i] = ds / avg_v
                else:
                    dt_values[i] = ds / 0.5  # Fallback for near-zero velocity
                dt_values[i] = np.clip(dt_values[i], 0.01, 2.0)
            
            # Step 9: Compute accelerations from velocity changes
            accel_x = np.zeros(n-1)
            accel_y = np.zeros(n-1)
            for i in range(n-1):
                dt = dt_values[i]
                if dt > 0.001:
                    accel_x[i] = (vel_x[i+1] - vel_x[i]) / dt
                    accel_y[i] = (vel_y[i+1] - vel_y[i]) / dt
            
            # Apply to point variables
            for i in range(n):
                self.point_vars.VEL_X[i].set_value(float(vel_x[i]))
                self.point_vars.VEL_Y[i].set_value(float(vel_y[i]))
            
            for i in range(n-1):
                self.point_vars.DT[i].set_value(float(dt_values[i]))
                self.point_vars.ACCEL_X[i].set_value(float(accel_x[i]))
                self.point_vars.ACCEL_Y[i].set_value(float(accel_y[i]))
            
            logging.Info(f"Initialized trajectory with cubic spline + velocity profile (total length: {total_length:.2f}m)")
            
        except Exception as e:
            # Fallback: zero velocity initialization
            logging.Warn(f"Cubic spline initialization failed ({e}), using zero velocity fallback")
            for i in range(n):
                self.point_vars.VEL_X[i].set_value(0)
                self.point_vars.VEL_Y[i].set_value(0)
            for i in range(n-1):
                self.point_vars.ACCEL_X[i].set_value(0)
                self.point_vars.ACCEL_Y[i].set_value(0)
        
        # Initialize Swerve Variables if applicable
        if isinstance(self.point_vars, SwervePointVariables):
            logging.Info("Initializing Swerve Point Variables...")
            for i in range(len(X_nodes)):
                vx = self.point_vars.VEL_X[i].value()
                vy = self.point_vars.VEL_Y[i].value()
                
                # Assume 0 omega initially, so module velocity = robot velocity
                for module in [self.point_vars.TL, self.point_vars.TR, self.point_vars.BL, self.point_vars.BR]:
                     module.VX[i].set_value(vx)
                     module.VY[i].set_value(vy)
                     # Forces default to 0
                     if i < len(X_nodes) - 1:
                         module.FX[i].set_value(0) 
                         module.FY[i].set_value(0)

        self.__apply_waypoint_constraints()

        return len(X_nodes)

    def __apply_waypoint_constraints(self):
        for order, waypoint in self.waypoints.items():

            for constraint in waypoint.constraints:
                constraint(self.problem, self.point_vars, waypoint.control_point_index)
                logging.Info(f"Applied waypoint constraint ({constraint}).")

    def __apply_kinematic_constraints(self):
        PositionKinematicsConstraint(self.problem, self.point_vars)
        VelocityKinematicsConstraint(self.problem, self.point_vars)
        ThetaKinematicsConstraint(self.problem, self.point_vars)
        OmegaKinematicsConstraint(self.problem, self.point_vars)

    def apply_basic_constraints(self):
        """Applies basic constraints to the optimization problem."""
        # Enforce strict positive dt to prevent division by zero / infinite velocity
        base.MagnitudeGreaterThanConstraint(self.problem, self.point_vars.DT, 0.01)
        base.MagnitudeLessThanConstraint(self.problem, self.point_vars.DT, 1)

    def __apply_spacing_constraints(self, config: TrajectoryBuilderConfig):
        PositionMinSpacingConstraint(config.min_spacing)(self.problem, self.point_vars)
        PositionMaxSpacingConstraint(
            config.trajectory_resolution * config.stretch_factor
        )(self.problem, self.point_vars)

    def apply_config_constraints(self, config: TrajectoryBuilderConfig):
        """Applies constraints to the optimization problem based on the configuration.

        :param config: The configuration of the trajectory optimization problem.
        :type config: :class:`TrajectoryBuilderConfig`"""

        if config.apply_kinematic_constraints:
            self.__apply_kinematic_constraints()

        if config.min_spacing > Inch(0) or config.stretch_factor > 1:
            self.__apply_spacing_constraints(config)

    def apply_minimization_objective(self, config: TrajectoryBuilderConfig):
        """Applies the minimization objective to the optimization problem based on the configuration.

        :param config: The configuration of the trajectory optimization problem.
        :type config: :class:`TrajectoryBuilderConfig`"""

        total = 0
        match config.minimization_strategy:
            case MinimizationStrategy.TIME:
                for i in range(len(self.point_vars.DT)):
                    total += self.point_vars.DT[i]
            case MinimizationStrategy.DISTANCE:
                for i in range(len(self.point_vars.POS_X) - 1):
                    total += (
                        self.point_vars.POS_X[i + 1] - self.point_vars.POS_X[i]
                    ) ** 2 + (
                        self.point_vars.POS_Y[i + 1] - self.point_vars.POS_Y[i]
                    ) ** 2
            case MinimizationStrategy.SMOOTHNESS:
                # Minimize Time (REMOVED to prioritize smoothness/feasibility)
                # time_weight = 1.0
                # for i in range(len(self.point_vars.DT)):
                #     total += self.point_vars.DT[i] * time_weight
                
                # Minimize Jerk (Change in Acceleration)
                jerk_weight = 1.0
                for i in range(len(self.point_vars.ACCEL_X) - 1):
                    # (a_i+1 - a_i)^2
                    jerk_x = (self.point_vars.ACCEL_X[i+1] - self.point_vars.ACCEL_X[i])**2
                    jerk_y = (self.point_vars.ACCEL_Y[i+1] - self.point_vars.ACCEL_Y[i])**2
                    total += (jerk_x + jerk_y) * jerk_weight

                # Minimize Angular Jerk
                angular_jerk_weight = 1.0
                for i in range(len(self.point_vars.ALPHA) - 1):
                     angular_jerk = (self.point_vars.ALPHA[i+1] - self.point_vars.ALPHA[i])**2
                     total += angular_jerk * angular_jerk_weight

        # Minimize Safety Corridor Slacks (Soft Constraints)
        # Moderate penalty to encourage staying within corridors while maintaining numerical stability
        # With MAX_SLACK=1m per direction, max penalty per point is 4 * 1e4 = 40k
        slack_weight = 1e4
        for i in range(len(self.point_vars.POS_X)):
            slack_sq = (
                self.point_vars.SLACK_X_POS[i]**2 +
                self.point_vars.SLACK_X_NEG[i]**2 +
                self.point_vars.SLACK_Y_POS[i]**2 +
                self.point_vars.SLACK_Y_NEG[i]**2
            )
            total += slack_sq * slack_weight
        
        self.problem.minimize(total)

    def generate(
        self, config: TrajectoryBuilderConfig = TrajectoryBuilderConfig()
    ) -> TrajectoryProblem:
        """Sets up and generates the optimization problem, based on the constraints and objectives currently added to the problem.

        :param config: The configuration of the trajectory optimization problem.
        :type config: :class:`Trajectory
        :return: The optimization problem for the trajectory.
        :rtype: :class:`TrajectoryProblem`"""
        self.problem = OptimizationProblem()
        if len(self.waypoints) < 2:
            raise ValueError("Trajectory must have at least two waypoints.")

        steps = self.initialize_state_variables_with_initial_pathes(
            self.initial_pathes, config.trajectory_resolution
        )

        self.apply_config_constraints(config)
        self.apply_basic_constraints()

        # Apply constraints for all points
        for constraint in self.point_constraints:
            constraint(self.problem, self.point_vars)

        # Minimization objective
        self.apply_minimization_objective(config)

        return TrajectoryProblem(self.problem, self.point_vars)

    def solve(self):
        """Solves the optimization problem and returns the solution."""
        if self.problem is None:
            self.generate()
        pass

    def points_constraint(
        self, constraint: Callable[[OptimizationProblem, PointVariables], None]
    ):
        """Adds a constraint that applies to all points in the trajectory.

        :param constraint: The constraint to add.
        :type constraint: Callable[[OptimizationProblem, PointVariables], None]"""

        self.point_constraints.append(constraint)
        pass

    def waypoint(self, waypoint: Waypoint, order: int = -1):
        """Adds a waypoint that the trajectory must pass through.

        :param waypoint: The waypoint to add.
        :type waypoint: :class:`Waypoint`
        :param order: The order of the waypoint in the trajectory.
        :type order: int"""

        if order == -1:
            order = len(self.waypoints)
        self.waypoints[order] = waypoint
        pass

    def guide_pathes(self, guide_pathes: List[Path]):
        """Adds guide pathes that aid the trajectory in reaching its waypoints.

        :param guide_pathes: The guide pathes to add.
        :type guide_pathes: List[:class:`Path`]"""

        if len(guide_pathes) != len(self.waypoints) - 1:
            logging.Warn("Number of guide pathes does not match number of waypoints.")
        self.initial_pathes = guide_pathes


class SwerveTrajectoryProblemBuilder(TrajectoryProblemBuilder):
    """Class used to build a swerve drive trajectory optimization problem and generate a solution. Allows for constraints and objectives to be added and setup."""

    def __init__(self) -> None:
        super().__init__()
        self.point_vars: SwervePointVariables = None
        pass

    def generate_point_vars(self, num_points: int):
        """Initializes the state variables for all points in the trajectory.

        :param num_points: The number of points in the trajectory.
        :type num_points: int
        :return: The state variables at all discrete points in the trajectory.
        :rtype: :class:`SwervePointVariables`"""

        return SwervePointVariables.with_initial_variables(self.problem, num_points)

    def generate(
        self, config: TrajectoryBuilderConfig = TrajectoryBuilderConfig()
    ) -> SwerveTrajectoryProblem:
        """Sets up and generates the optimization problem, based on the constraints and objectives currently added to the problem.

        :param config: The configuration of the trajectory optimization problem.
        :type config: :class:`TrajectoryBuilderConfig`
        :return: The optimization problem for the trajectory.
        :rtype: :class:`SwerveTrajectoryProblem`"""
        self.problem = OptimizationProblem()

        if len(self.waypoints) < 2:
            raise ValueError("Trajectory must have at least two waypoints.")

        steps = self.initialize_state_variables_with_initial_pathes(
            self.initial_pathes, config.trajectory_resolution
        )

        self.apply_config_constraints(config)
        self.apply_basic_constraints()

        # Apply constraints for all points
        for constraint in self.point_constraints:
            constraint(self.problem, self.point_vars)

        # Minimization objective
        self.apply_minimization_objective(config)

        return SwerveTrajectoryProblem(self.problem, self.point_vars)
