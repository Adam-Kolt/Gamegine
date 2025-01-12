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

    for i in range(1, len(point_variables.POS_X)):
        dt = point_variables.DT[i - 1]
        curr_x = point_variables.POS_X[i]
        prev_x = point_variables.POS_X[i - 1]
        prev_vel_x = point_variables.VEL_X[i - 1]
        prev_accel_x = point_variables.ACCEL_X[i - 1]

        problem.subject_to(
            curr_x == prev_x + prev_vel_x * dt + 0.5 * prev_accel_x * dt**2
        )

        curr_y = point_variables.POS_Y[i]
        prev_y = point_variables.POS_Y[i - 1]
        prev_vel_y = point_variables.VEL_Y[i - 1]
        prev_accel_y = point_variables.ACCEL_Y[i - 1]

        problem.subject_to(
            curr_y == prev_y + prev_vel_y * dt + 0.5 * prev_accel_y * dt**2
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

    for i in range(1, len(point_variables.THETA)):
        dt = point_variables.DT[i - 1]
        curr_theta = point_variables.THETA[i]
        prev_theta = point_variables.THETA[i - 1]
        prev_omega = point_variables.OMEGA[i - 1]
        prev_alpha = point_variables.ALPHA[i - 1]

        problem.subject_to(
            curr_theta == prev_theta + prev_omega * dt + 0.5 * prev_alpha * dt**2
        )
