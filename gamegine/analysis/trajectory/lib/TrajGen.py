from typing import Callable, List
from jormungandr.optimization import OptimizationProblem
from dataclasses import dataclass

from gamegine.analysis.pathfinding import Path


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


class TrajectoryProblem:

    def __init__(self, problem: OptimizationProblem) -> None:
        self.problem = problem
        pass

    def solve(self):
        """Solves the optimization problem and returns the solution."""
        pass


class TrajectoryProblemBuilder:
    """Class used to build a trajectory optimization problem and generate a solution. Allows for constraints and objectives to be added and setup."""

    def __init__(self) -> None:
        self.problem = None
        self.point_vars: PointVariables = None
        pass

    def generate(self) -> TrajectoryProblem:
        """Sets up and generates the optimization problem, based on the constraints and objectives currently added to the problem."""
        self.problem = OptimizationProblem()
        pass

    def solve(self):
        """Solves the optimization problem and returns the solution."""
        if self.problem is None:
            self.generate()
        pass

    def point_constraint(self, constraint: Callable[[PointVariables]]):
        """Adds a constraint that applies to a point or points in the trajectory."""
        pass

    def waypoint(self, order: int, x: float, y: float):
        """Adds a waypoint that the trajectory must pass through."""
        pass

    def guide_pathes(self, guide_pathes: List[Path]):
        """Adds guide pathes that aid the trajectory in reaching its waypoints."""
        pass


class SwerveTrajectoryProblemBuilder(TrajectoryProblemBuilder):
    """Class used to build a swerve drive trajectory optimization problem and generate a solution. Allows for constraints and objectives to be added and setup."""

    def __init__(self) -> None:
        super().__init__()
        self.point_vars: SwervePointVariables = None
        pass

    def generate(self) -> TrajectoryProblem:
        """Sets up and generates the optimization problem, based on the constraints and objectives currently added to the problem."""
        self.problem = OptimizationProblem()
        pass
