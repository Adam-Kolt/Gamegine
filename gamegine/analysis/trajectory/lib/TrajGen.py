import logging
from typing import Callable, List
from jormungandr.optimization import OptimizationProblem
from dataclasses import dataclass, field

from gamegine.analysis.pathfinding import Path
from gamegine.analysis.trajectory.lib import CALCULATION_UNIT_SPATIAL
from gamegine.reference.swerve import SwerveConfig
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
class PointVariables:
    """Dataclass used to store the state variables at all discrete points in a trajectory."""

    POS_X: list
    POS_Y: list
    VEL_X: list
    VEL_Y: list
    ACCEL_X: list
    ACCEL_Y: list
    THETA: list
    OMEGA: list
    ALPHA: list
    DT: list

    @classmethod
    def with_initial_variables(cls, problem: OptimizationProblem, num_points: int):
        """Initializes the state variables for all points in the trajectory."""

        return cls(
            POS_X=problem.decision_variable(num_points),
            POS_Y=problem.decision_variable(num_points),
            VEL_X=problem.decision_variable(num_points),
            VEL_Y=problem.decision_variable(num_points),
            ACCEL_X=problem.decision_variable(num_points),
            ACCEL_Y=problem.decision_variable(num_points),
            THETA=problem.decision_variable(num_points),
            OMEGA=problem.decision_variable(num_points - 1),
            ALPHA=problem.decision_variable(num_points - 1),
            DT=problem.decision_variable(num_points - 1),
        )


@dataclass
class ModuleVariables:
    FX: list
    FY: list
    VX: list
    VY: list


class SwervePointVariables(PointVariables):
    """Dataclass used to store the state variables at all discrete points in a swerve drive trajectory."""

    TL: ModuleVariables
    TR: ModuleVariables
    BL: ModuleVariables
    BR: ModuleVariables

    @classmethod
    def with_initial_variables(cls, problem: OptimizationProblem, num_points: int):
        """Initializes the state variables for all points in the trajectory."""

        return cls(
            POS_X=problem.decision_variable(num_points),
            POS_Y=problem.decision_variable(num_points),
            VEL_X=problem.decision_variable(num_points),
            VEL_Y=problem.decision_variable(num_points),
            ACCEL_X=problem.decision_variable(num_points),
            ACCEL_Y=problem.decision_variable(num_points),
            THETA=problem.decision_variable(num_points),
            OMEGA=problem.decision_variable(num_points - 1),
            ALPHA=problem.decision_variable(num_points - 1),
            DT=problem.decision_variable(num_points - 1),
            TL=ModuleVariables(
                FX=problem.decision_variable(num_points - 1),
                FY=problem.decision_variable(num_points - 1),
                VX=problem.decision_variable(num_points),
                VY=problem.decision_variable(num_points),
            ),
            TR=ModuleVariables(
                FX=problem.decision_variable(num_points - 1),
                FY=problem.decision_variable(num_points - 1),
                VX=problem.decision_variable(num_points),
                VY=problem.decision_variable(num_points),
            ),
            BL=ModuleVariables(
                FX=problem.decision_variable(num_points - 1),
                FY=problem.decision_variable(num_points - 1),
                VX=problem.decision_variable(num_points),
                VY=problem.decision_variable(num_points),
            ),
            BR=ModuleVariables(
                FX=problem.decision_variable(num_points - 1),
                FY=problem.decision_variable(num_points - 1),
                VX=problem.decision_variable(num_points),
                VY=problem.decision_variable(num_points),
            ),
        )


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


class TrajectoryProblem:
    def __init__(self, problem: OptimizationProblem) -> None:
        self.problem = problem

        pass

    def solve(self, robot_constraints: TrajectoryRobotConstraints):
        """Solves the optimization problem and returns the solution."""
        pass


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

    def generate(
        self, resolution: SpatialMeasurement = Centimeter(10)
    ) -> TrajectoryProblem:
        """Sets up and generates the optimization problem, based on the constraints and objectives currently added to the problem."""
        self.problem = OptimizationProblem()
        if len(self.waypoints) < 2:
            raise ValueError("Trajectory must have at least two waypoints.")

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

        # Apply constraints for each waypoint
        for i, (order, waypoint) in enumerate(waypoints):
            for constraint in waypoint.constraints:
                constraint(self.point_vars, waypoint.control_point_index)

        # Apply constraints for all points
        for constraint in self.point_constraints:
            constraint(self.point_vars)

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
