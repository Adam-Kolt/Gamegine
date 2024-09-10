def get_magnitude_squared(var1, var2):
    return var1**2 + var2**2


def __get_indices(index: int, length: int):
    return range(length) if index == -1 else [index]


def __VectorMagnitudeEqualityConstraint(
    problem, xVars, yVars, magnitude: float, index: int = -1
):
    for i in __get_indices(index, len(xVars)):
        problem.subject_to(get_magnitude_squared(xVars[i], yVars[i]) == magnitude**2)


def __VectorMagnitudeLessThanConstraint(
    problem, xVars, yVars, magnitude: float, index: int = -1
):
    for i in __get_indices(index, len(xVars)):
        problem.subject_to(get_magnitude_squared(xVars[i], yVars[i]) < magnitude**2)


def __VectorMagnitudeGreaterThanConstraint(
    problem, xVars, yVars, magnitude: float, index: int = -1
):
    for i in __get_indices(index, len(xVars)):
        problem.subject_to(get_magnitude_squared(xVars[i], yVars[i]) > magnitude**2)


def __MagnitudeEqualityConstraint(problem, vars, magnitude: float, index: int = -1):
    for i in __get_indices(index, len(vars)):
        problem.subject_to(vars[i] == magnitude)


def __MagnitudeLessThanConstraint(problem, vars, magnitude: float, index: int = -1):
    for i in __get_indices(index, len(vars)):
        problem.subject_to(vars[i] < magnitude)


def __MagnitudeGreaterThanConstraint(problem, vars, magnitude: float, index: int = -1):
    for i in __get_indices(index, len(vars)):
        problem.subject_to(vars[i] > magnitude)


def __DerivativeAgreementConstraint(problem, derivativeVars, vars, timeSteps):
    for i in range(1, len(vars)):
        problem.subject_to(
            vars[i] == vars[i - 1] + timeSteps[i - 1] * derivativeVars[i - 1]
        )


def __MinVectorSpacingConstraint(problem, xVars, yVars, minSpacing: float):
    for i in range(len(xVars) - 1):
        problem.subject_to(
            get_magnitude_squared(xVars[i] - xVars[i + 1], yVars[i] - yVars[i + 1])
            >= minSpacing**2
        )


def __MaxVectorSpacingConstraint(problem, xVars, yVars, maxSpacing: float):
    for i in range(len(xVars) - 1):
        problem.subject_to(
            get_magnitude_squared(xVars[i] - xVars[i + 1], yVars[i] - yVars[i + 1])
            <= maxSpacing**2
        )
