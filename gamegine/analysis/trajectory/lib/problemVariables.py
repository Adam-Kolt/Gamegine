from dataclasses import dataclass
from jormungandr.optimization import OptimizationProblem


@dataclass
class PointVariables:
    """Dataclass used to store the state variables at all discrete points in a trajectory.

    :param POS_X: The x position of the robot.
    :type POS_X: list containing Sleipnir decision variables
    :param POS_Y: The y position of the robot.
    :type POS_Y: list containing Sleipnir decision variables
    :param VEL_X: The x velocity of the robot.
    :type VEL_X: list containing Sleipnir decision variables
    :param VEL_Y: The y velocity of the robot.
    :type VEL_Y: list containing Sleipnir decision variables
    :param ACCEL_X: The x acceleration of the robot.
    :type ACCEL_X: list containing Sleipnir decision variables
    :param ACCEL_Y: The y acceleration of the robot.
    :type ACCEL_Y: list containing Sleipnir decision variables
    :param THETA: The angle of the robot.
    :type THETA: list containing Sleipnir decision variables
    :param OMEGA: The angular velocity of the robot.
    :type OMEGA: list containing Sleipnir decision variables
    :param ALPHA: The angular acceleration of the robot.
    :type ALPHA: list containing Sleipnir decision variables
    :param DT: The time step between each point.
    :type DT: list containing Sleipnir decision variables
    """

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
        """Initializes the state variables for all points in the trajectory.

        :param problem: The optimization problem to use to create the decision variables.
        :type problem: :class:`OptimizationProblem`
        :param num_points: The number of points in the trajectory.
        :type num_points: int
        :return: The state variables for all points in the trajectory.
        :rtype: :class:`PointVariables`
        """

        return cls(
            POS_X=problem.decision_variable(num_points),
            POS_Y=problem.decision_variable(num_points),
            VEL_X=problem.decision_variable(num_points),
            VEL_Y=problem.decision_variable(num_points),
            ACCEL_X=problem.decision_variable(num_points - 1),
            ACCEL_Y=problem.decision_variable(num_points - 1),
            THETA=problem.decision_variable(num_points),
            OMEGA=problem.decision_variable(num_points),
            ALPHA=problem.decision_variable(num_points - 1),
            DT=problem.decision_variable(num_points - 1),
        )


@dataclass
class ModuleVariables:
    FX: list
    FY: list
    VX: list
    VY: list


@dataclass
class SwervePointVariables(PointVariables):
    """Dataclass used to store the state variables at all discrete points in a swerve drive trajectory.

    :param POS_X: The x position of the robot.
    :type POS_X: list containing Sleipnir decision variables
    :param POS_Y: The y position of the robot.
    :type POS_Y: list containing Sleipnir decision variables
    :param VEL_X: The x velocity of the robot.
    :type VEL_X: list containing Sleipnir decision variables
    :param VEL_Y: The y velocity of the robot.
    :type VEL_Y: list containing Sleipnir decision variables
    :param ACCEL_X: The x acceleration of the robot.
    :type ACCEL_X: list containing Sleipnir decision variables
    :param ACCEL_Y: The y acceleration of the robot.
    :type ACCEL_Y: list containing Sleipnir decision variables
    :param THETA: The angle of the robot.
    :type THETA: list containing Sleipnir decision variables
    :param OMEGA: The angular velocity of the robot.
    :type OMEGA: list containing Sleipnir decision variables
    :param ALPHA: The angular acceleration of the robot.
    :type ALPHA: list containing Sleipnir decision variables
    :param DT: The time step between each point.
    :type DT: list containing Sleipnir decision variables
    :param TL: The state variables for the top left swerve module.
    :type TL: :class:`ModuleVariables`
    :param TR: The state variables for the top right swerve module.
    :type TR: :class:`ModuleVariables`
    :param BL: The state variables for the bottom left swerve module.
    :type BL: :class:`ModuleVariables`
    :param BR: The state variables for the bottom right swerve module.
    :type BR: :class:`ModuleVariables
    """

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
            ACCEL_X=problem.decision_variable(num_points - 1),
            ACCEL_Y=problem.decision_variable(num_points - 1),
            THETA=problem.decision_variable(num_points),
            OMEGA=problem.decision_variable(num_points),
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
