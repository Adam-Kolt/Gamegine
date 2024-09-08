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
