from gamegine.utils import logging


def get_magnitude_squared(var1, var2):
    """Returns the square of the magnitude of the vector defined by the two variables.

    :param var1: The first variable.
    :type var1: float
    :param var2: The second variable.
    :type var2: float
    :return: The square of the magnitude of the vector defined by the two variables.
    :rtype: float
    """
    return var1**2 + var2**2


def __get_indices(index: int, length: int):
    return range(length) if index == -1 else [index]


def __VectorMagnitudeEqualityConstraint(
    problem, xVars, yVars, magnitude: float, index: int = -1
):
    """Constraint for setting the magnitude of a vector to a specific value.

    :param problem: The optimization problem.
    :type problem: Problem
    :param xVars: The x-components of the vectors.
    :type xVars: list
    :param yVars: The y-components of the vectors.
    :type yVars: list
    :param magnitude: The desired magnitude of the vectors.
    :type magnitude: float
    :param index: The index of the vector to constrain. If -1, all vectors are constrained.
    :type index: int
    """
    for i in __get_indices(index, len(xVars)):
        problem.subject_to(get_magnitude_squared(xVars[i], yVars[i]) == magnitude**2)


def __VectorMagnitudeLessThanConstraint(
    problem, xVars, yVars, magnitude: float, index: int = -1
):
    """Constraint for setting the magnitude of a vector to be less than a specific value.

    :param problem: The optimization problem.
    :type problem: Problem
    :param xVars: The x-components of the vectors.
    :type xVars: list
    :param yVars: The y-components of the vectors.
    :type yVars: list
    :param magnitude: The desired maximum magnitude of the vectors.
    :type magnitude: float
    :param index: The index of the vector to constrain. If -1, all vectors are constrained.
    :type index: int"""
    for i in __get_indices(index, len(xVars)):
        problem.subject_to(get_magnitude_squared(xVars[i], yVars[i]) < magnitude**2)


def __VectorMagnitudeGreaterThanConstraint(
    problem, xVars, yVars, magnitude: float, index: int = -1
):
    """Constraint for setting the magnitude of a vector to be greater than a specific value.

    :param problem: The optimization problem.
    :type problem: Problem
    :param xVars: The x-components of the vectors.
    :type xVars: list
    :param yVars: The y-components of the vectors.
    :type yVars: list
    :param magnitude: The desired minimum magnitude of the vectors.
    :type magnitude: float
    :param index: The index of the vector to constrain. If -1, all vectors are constrained.
    :type index: int"""
    for i in __get_indices(index, len(xVars)):
        problem.subject_to(get_magnitude_squared(xVars[i], yVars[i]) > magnitude**2)


def __MagnitudeEqualityConstraint(problem, vars, magnitude: float, index: int = -1):
    """Constraint for setting the magnitude of a scalar to a specific value.

    :param problem: The optimization problem.
    :type problem: Problem
    :param vars: The scalar variables.
    :type vars: list
    :param magnitude: The desired magnitude of the scalar.
    :type magnitude: float
    :param index: The index of the scalar to constrain. If -1, all scalars are constrained.
    :type index: int"""
    for i in __get_indices(index, len(vars)):
        logging.Debug(
            f"Adding magnitude equality constraint of {magnitude} to control point {i}"
        )
        problem.subject_to(vars[i] == magnitude)


def MagnitudeLessThanConstraint(problem, vars, magnitude: float, index: int = -1):
    """Constraint for setting the magnitude of a scalar to be less than a specific value.

    :param problem: The optimization problem.
    :type problem: Problem
    :param vars: The scalar variables.
    :type vars: list
    :param magnitude: The desired maximum magnitude of the scalar.
    :type magnitude: float
    :param index: The index of the scalar to constrain. If -1, all scalars are constrained.
    :type index: int"""

    for i in __get_indices(index, len(vars)):
        problem.subject_to(vars[i] < magnitude)


def MagnitudeGreaterThanConstraint(problem, vars, magnitude: float, index: int = -1):
    """Constraint for setting the magnitude of a scalar to be greater than a specific value.

    :param problem: The optimization problem.
    :type problem: Problem
    :param vars: The scalar variables.
    :type vars: list
    :param magnitude: The desired minimum magnitude of the scalar.
    :type magnitude: float
    :param index: The index of the scalar to constrain. If -1, all scalars are constrained.
    :type index: int"""
    for i in __get_indices(index, len(vars)):
        problem.subject_to(vars[i] > magnitude)


def __DerivativeAgreementConstraint(problem, derivativeVars, vars, timeSteps):
    """Constraint for setting the derivative of a variable to be equal to the previous value plus the product of the time step and the previous derivative.

    :param problem: The optimization problem.
    :type problem: Problem
    :param derivativeVars: The derivative variables.
    :type derivativeVars: list
    :param vars: The variables.
    :type vars: list
    :param timeSteps: The time steps between each point.
    :type timeSteps: list"""

    for i in range(1, len(vars)):
        problem.subject_to(
            vars[i] == vars[i - 1] + timeSteps[i - 1] * derivativeVars[i - 1]
        )


def __MinVectorSpacingConstraint(problem, xVars, yVars, minSpacing: float):
    """Constraint for setting the minimum spacing between vectors.

    :param problem: The optimization problem.
    :type problem: Problem
    :param xVars: The x-components of the vectors.
    :type xVars: list
    :param yVars: The y-components of the vectors.
    :type yVars: list
    :param minSpacing: The desired minimum spacing between vectors.
    :type minSpacing: float"""

    for i in range(len(xVars) - 1):
        problem.subject_to(
            get_magnitude_squared(xVars[i] - xVars[i + 1], yVars[i] - yVars[i + 1])
            >= minSpacing**2
        )


def __MaxVectorSpacingConstraint(problem, xVars, yVars, maxSpacing: float):
    """Constraint for setting the maximum spacing between vectors.

    :param problem: The optimization problem.
    :type problem: Problem
    :param xVars: The x-components of the vectors.
    :type xVars: list
    :param yVars: The y-components of the vectors.
    :type yVars: list
    :param maxSpacing: The desired maximum spacing between vectors.
    :type maxSpacing: float"""

    for i in range(len(xVars) - 1):
        problem.subject_to(
            get_magnitude_squared(xVars[i] - xVars[i + 1], yVars[i] - yVars[i + 1])
            <= maxSpacing**2
        )
