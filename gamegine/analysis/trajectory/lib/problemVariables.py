from dataclasses import dataclass
from jormungandr.optimization import OptimizationProblem


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
