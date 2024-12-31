from gamegine.analysis.trajectory.lib.constraints.base import (
    __DerivativeAgreementConstraint,
)


def VelocityKinematicsConstraint(problem, point_variables):
    """
    A constraint that enforces the kinematic equation for velocity, v = u + at.

    :param problem: The optimization problem.
    :type problem: Problem
    :param point_variables: The variables for the points in the trajectory.
    :type point_variables: PointVariables
    """
    __DerivativeAgreementConstraint(
        problem, point_variables.ACCEL_X, point_variables.VEL_X, point_variables.DT
    )

    __DerivativeAgreementConstraint(
        problem, point_variables.ACCEL_Y, point_variables.VEL_Y, point_variables.DT
    )


def PositionKinematicsConstraint(problem, point_variables):
    """
    A constraint that enforces the kinematic equation for position, s = ut + 0.5at^2.

    :param problem: The optimization problem.
    :type problem: Problem
    :param point_variables: The variables for the points in the trajectory.
    :type point_variables: PointVariables
    """

    __DerivativeAgreementConstraint(
        problem, point_variables.VEL_X, point_variables.POS_X, point_variables.DT
    )

    __DerivativeAgreementConstraint(
        problem, point_variables.VEL_Y, point_variables.POS_Y, point_variables.DT
    )


def OmegaKinematicsConstraint(problem, point_variables):
    """
    A constraint that enforces the kinematic equation for angular velocity, w = u + at.

    :param problem: The optimization problem.
    :type problem: Problem
    :param point_variables: The variables for the points in the trajectory.
    :type point_variables: PointVariables
    """
    __DerivativeAgreementConstraint(
        problem, point_variables.ALPHA, point_variables.OMEGA, point_variables.DT
    )


def ThetaKinematicsConstraint(problem, point_variables):
    """
    A constraint that enforces the kinematic equation for angular position, theta = omega*t + 0.5*alpha*t^2.

    :param problem: The optimization problem.
    :type problem: Problem
    :param point_variables: The variables for the points in the trajectory.
    :type point_variables: PointVariables
    :return: The constraint function.
    :rtype: Callable[[Problem, PointVariables], None]
    """
    __DerivativeAgreementConstraint(
        problem, point_variables.OMEGA, point_variables.THETA, point_variables.DT
    )
