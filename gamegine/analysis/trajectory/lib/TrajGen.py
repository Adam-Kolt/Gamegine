from enum import Enum

import math
from typing import Callable, List
from jormungandr.optimization import OptimizationProblem
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

    solution_tolerance: float = 1e-6
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
        self.robot_constraints = robot_constraints
        self.points = points
        self.travel_time = 0
        self.path_length = 0
        pass

    def get_robot_constraints(self) -> SwerveRobotConstraints:
        return self.robot_constraints

    def __str__(self) -> str:
        return f"Swerve Trajectory: {len(self.points)} points, {self.get_length()} length, {self.get_travel_time()} time. Optimized for {self.robot_constraints}."

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

        :param callback: The callback function to call.
        :type callback: Callable"""
        self.problem.callback(callback)

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

    def solve(self, robot_constraints: SwerveRobotConstraints, config: SolverConfig):
        """Solves the optimization problem and returns the solution."""

        self.apply_constraints(robot_constraints)
        self.apply_swerve_constraints(robot_constraints)

        status = self.problem.solve(
            tolerance=config.solution_tolerance,
            max_iterations=config.max_iterations,
            timeout=config.timeout,
            diagnostics=True,
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
            dissected_path = Path(
                path.dissected(units_per_node=resolution).get_points()[1:]
            )  # TODO: Do this better. This is done to ensure first point is not in same position as last point of previous path, which cooks the solver

            for node in dissected_path.get_points():
                X_nodes.append(node[0].to(CALCULATION_UNIT_SPATIAL))
                Y_nodes.append(node[1].to(CALCULATION_UNIT_SPATIAL))

            waypoints[i + 1][1].control_point_index = len(X_nodes) - 1

        self.point_vars = self.generate_point_vars(len(X_nodes))

        # Initialize state variables to inital pathes
        for i in range(len(X_nodes)):

            self.point_vars.POS_X[i].set_value(X_nodes[i])
            self.point_vars.POS_Y[i].set_value(Y_nodes[i])

        # Initialize time steps
        init_dt = (resolution / MetersPerSecond(4)).to(CALCULATION_UNIT_TEMPORAL)
        for i in range(len(X_nodes) - 1):
            self.point_vars.DT[i].set_value(init_dt)

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
        OmegaKinematicsConstraint(self.problem, self.point_vars)
        ThetaKinematicsConstraint(self.problem, self.point_vars)

    def apply_basic_constraints(self):
        """Applies basic constraints to the optimization problem."""
        base.MagnitudeGreaterThanConstraint(self.problem, self.point_vars.DT, 0)

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

        if config.min_spacing > 0 or config.stretch_factor > 1:
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
