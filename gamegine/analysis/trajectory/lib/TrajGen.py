from enum import Enum
import logging
from typing import Callable, List
from jormungandr.optimization import OptimizationProblem
from dataclasses import dataclass, field

from gamegine.analysis.pathfinding import Path
from gamegine.analysis.trajectory.lib import CALCULATION_UNIT_SPATIAL
from gamegine.analysis.trajectory.lib.constraints.constraints import (
    AccelerationLessThan,
    AngularAccelerationLessThan,
    AngularVelocityLessThan,
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
from gamegine.utils.NCIM.ComplexDimensions.acceleration import (
    Acceleration,
    MeterPerSecondSquared,
)
from gamegine.utils.NCIM.ComplexDimensions.alpha import Alpha, RadiansPerSecondSquared
from gamegine.utils.NCIM.ComplexDimensions.omega import Omega, RadiansPerSecond
from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond, Velocity
from gamegine.utils.NCIM.Dimensions.spatial import Centimeter, Meter, SpatialMeasurement
from sortedcontainers import SortedDict


@dataclass
class TrajectoryRobotConstraints:
    """Dataclass used to store robot constraints for a trajectory optimization problem."""

    max_acceleration: Acceleration = MeterPerSecondSquared(0)
    max_velocity: Velocity = MetersPerSecond(0)
    max_angular_acceleration: Alpha = RadiansPerSecondSquared(0)
    max_angular_velocity: Omega = RadiansPerSecond(0)


@dataclass
class SwerveRobotConstraints(TrajectoryRobotConstraints):
    """Dataclass used to store swerve drive robot constraints for a trajectory optimization problem."""

    swerve_config: SwerveConfig


@dataclass
class SolverConfig:
    solution_tolerance: float = 1e-6
    max_iterations: int = 1000
    timeout: float = 100.0


class Trajectory(Drawable):
    def __init__(self, points: List[TrajectoryState]) -> None:
        self.points = points
        pass


class TrajectoryProblem:
    def __init__(
        self, intialized_problem: OptimizationProblem, point_vars: PointVariables
    ):
        self.point_vars = point_vars
        self.problem = intialized_problem

        pass

    def __apply_constraints(self, robot_constraints: TrajectoryRobotConstraints):
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

    def get_trajectory_states(self) -> List[TrajectoryState]:
        vars = self.point_vars

        states = []

        for i in range(len(vars.ACCEL_X)):
            states.append(
                TrajectoryState(
                    x=vars.POS_X[i],
                    y=vars.POS_Y[i],
                    theta=vars.THETA[i],
                    vel_x=vars.VEL_X[i],
                    vel_y=vars.VEL_Y[i],
                    acc_x=vars.ACCEL_X[i],
                    acc_y=vars.ACCEL_Y[i],
                    omega=vars.OMEGA[i],
                    alpha=vars.ALPHA[i],
                )
            )

        # Add final state
        states.append(
            TrajectoryState(
                x=CALCULATION_UNIT_SPATIAL(vars.POS_X[-1]),
                y=CALCULATION_UNIT_SPATIAL(vars.POS_Y[-1]),
                theta=vars.THETA[-1],
                vel_x=vars.VEL_X[-1],
                vel_y=vars.VEL_Y[-1],
                acc_x=vars.ACCEL_X[-1],
                acc_y=vars.ACCEL_Y[-1],
                omega=vars.OMEGA[-1],
                alpha=None,
            )
        )

    def solve(
        self, robot_constraints: TrajectoryRobotConstraints, config: SolverConfig
    ) -> Trajectory:
        """Solves the optimization problem and returns the solution."""
        self.__apply_constraints(robot_constraints)

        self.problem.solve(
            tolerance=config.solution_tolerance,
            max_iterations=config.max_iterations,
            timeout=config.timeout,
        )


class SwerveTrajectoryProblem(TrajectoryProblem):
    def __init__(self, problem: OptimizationProblem) -> None:
        super().__init__(problem)
        pass

    def solve(self, robot_constraints: SwerveRobotConstraints):
        """Solves the optimization problem and returns the solution."""
        pass


@dataclass
class Waypoint:
    """Dataclass used to store a waypoint that the trajectory must pass through."""

    x: SpatialMeasurement
    y: SpatialMeasurement
    constraints: List[Callable[[OptimizationProblem, PointVariables, int]]] = field(
        default_factory=list
    )
    control_point_index: int = -1

    def given(
        self, *constraints: List[Callable[[OptimizationProblem, PointVariables, int]]]
    ) -> "Waypoint":
        """Adds constraints to the waypoint."""
        self.constraints.extend(constraints)
        return self

    def __post_init__(self):
        if self.x <= 0:
            raise ValueError("Waypoint x must be positive.")
        if self.y <= 0:
            raise ValueError("Waypoint y must be positive.")


class MinimizationStrategy(Enum):
    TIME = 0
    DISTANCE = 1


@dataclass
class TrajectoryBuilderConfig:
    """Dataclass used to store configuration information for a trajectory optimization problem."""

    trajectory_resolution: SpatialMeasurement = Centimeter(10)
    stretch_factor: float = 1.1
    min_spacing: SpatialMeasurement = Centimeter(1)
    apply_kinematic_constraints: bool = True
    minimization_strategy: MinimizationStrategy = MinimizationStrategy.TIME


class TrajectoryProblemBuilder:
    """Class used to build a trajectory optimization problem and generate a solution. Allows for constraints and objectives to be added and setup."""

    def __init__(self) -> None:
        self.problem = None
        self.point_vars: PointVariables = None
        self.waypoints: SortedDict[int, Waypoint] = SortedDict()
        self.initial_pathes = []
        self.point_constraints = []

        pass

    def __generate_point_vars(self, num_points: int):
        return PointVariables.with_initial_variables(self.problem, len(num_points))

    def __initialize_state_variables_with_initial_pathes(
        self, initial_pathes: List[Path], resolution: SpatialMeasurement
    ) -> int:
        X_nodes = []
        Y_nodes = []

        waypoints = self.waypoints.items()

        # Set the control node for first waypoint
        waypoints[0][1].control_point_index = 0

        for i, (order, waypoint) in enumerate(waypoints)[:-1]:
            if i < len(self.initial_pathes):
                path = self.initial_pathes[i]
            else:
                path = Path(
                    [
                        (waypoint.x, waypoint.y),
                        (waypoints[i + 1][1].x, waypoints[i + 1][1].y),
                    ]
                )
            dissected_path = path.dissected(units_per_node=resolution)

            for node in dissected_path.get_points():
                X_nodes.append(node[0].to(CALCULATION_UNIT_SPATIAL))
                Y_nodes.append(node[1].to(CALCULATION_UNIT_SPATIAL))

            waypoints[i + 1][1].control_point_index = len(X_nodes)

        self.point_vars = self.__generate_point_vars(len(X_nodes))

        # Initialize state variables to inital pathes
        for i in range(len(X_nodes)):
            self.point_vars.POS_X[i].set_value(X_nodes[i])
            self.point_vars.POS_Y[i].set_value(Y_nodes[i])

        self.__apply_waypoint_constraints()

        return len(X_nodes)

    def __apply_waypoint_constraints(self):
        for order, waypoint in self.waypoints.items():
            for constraint in waypoint.constraints:
                constraint(self.problem, self.point_vars, waypoint.control_point_index)

    def __apply_kinematic_constraints(self):
        PositionKinematicsConstraint(self.problem, self.point_vars)
        VelocityKinematicsConstraint(self.problem, self.point_vars)
        OmegaKinematicsConstraint(self.problem, self.point_vars)
        ThetaKinematicsConstraint(self.problem, self.point_vars)

    def __apply_spacing_constraints(self, config: TrajectoryBuilderConfig):
        pass

    def __apply_config_constraints(self, config: TrajectoryBuilderConfig):
        if config.apply_kinematic_constraints:
            self.__apply_kinematic_constraints

        if config.min_spacing > 0 or config.stretch_factor > 1:
            self.__apply_spacing_constraints(
                config.min_spacing, config.stretch_factor, config
            )

    def __apply_minimization_objective(self, config: TrajectoryBuilderConfig):
        total = 0
        match config.minimization_strategy:
            case MinimizationStrategy.TIME:
                for i in range(len(self.point_vars.DT)):
                    total += self.point_vars.DT[i]
            case MinimizationStrategy.DISTANCE:
                for i in range(len(self.point_vars.POS_X) - 1):
                    total += (
                        (self.point_vars.POS_X[i + 1] - self.point_vars.POS_X[i]) ** 2
                        + (self.point_vars.POS_Y[i + 1] - self.point_vars.POS_Y[i]) ** 2
                    ) ** 0.5

        self.problem.minimize(total)

    def generate(
        self, config: TrajectoryBuilderConfig = TrajectoryBuilderConfig()
    ) -> TrajectoryProblem:
        """Sets up and generates the optimization problem, based on the constraints and objectives currently added to the problem."""
        self.problem = OptimizationProblem()
        if len(self.waypoints) < 2:
            raise ValueError("Trajectory must have at least two waypoints.")

        steps = self.__initialize_state_variables_with_initial_pathes(
            self.initial_pathes, config.trajectory_resolution
        )

        self.__apply_config_constraints(config)

        # Apply constraints for all points
        for constraint in self.point_constraints:
            constraint(self.point_vars)

        # Minimization objective
        self.__apply_minimization_objective(config)

        return TrajectoryProblem(self.problem)

    def solve(self):
        """Solves the optimization problem and returns the solution."""
        if self.problem is None:
            self.generate()
        pass

    def points_constraint(
        self, constraint: Callable[[OptimizationProblem, PointVariables]]
    ):
        """Adds a constraint that applies to all points in the trajectory."""
        self.point_constraints.append(constraint)
        pass

    def waypoint(self, waypoint: Waypoint, order: int = -1):
        """Adds a waypoint that the trajectory must pass through."""
        if order == -1:
            order = len(self.waypoints)
        self.waypoints[order] = waypoint
        pass

    def guide_pathes(self, guide_pathes: List[Path]):
        """Adds guide pathes that aid the trajectory in reaching its waypoints."""
        if len(guide_pathes) != len(self.waypoints):
            logging.warning(
                "Number of guide pathes does not match number of waypoints."
            )
        self.intial_pathes = guide_pathes


class SwerveTrajectoryProblemBuilder(TrajectoryProblemBuilder):
    """Class used to build a swerve drive trajectory optimization problem and generate a solution. Allows for constraints and objectives to be added and setup."""

    def __init__(self) -> None:
        super().__init__()
        self.point_vars: SwervePointVariables = None
        pass

    def __generate_point_vars(self, num_points: int):
        return SwervePointVariables.with_initial_variables(
            self.problem, len(num_points)
        )

    def generate(self) -> SwerveTrajectoryProblem:
        """Sets up and generates the optimization problem, based on the constraints and objectives currently added to the problem."""
        self.problem = OptimizationProblem()
        pass
