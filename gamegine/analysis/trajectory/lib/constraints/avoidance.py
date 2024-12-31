from typing import Dict, List
from gamegine.analysis.trajectory.lib import CALCULATION_UNIT_SPATIAL
from gamegine.analysis.trajectory.lib.problemVariables import PointVariables
from gamegine.render import helpers
from gamegine.render.style import Palette
from gamegine.representation.bounds import DiscreteBoundary, Rectangle
from gamegine.utils.NCIM.Dimensions.spatial import Centimeter, Meter, SpatialMeasurement
from gamegine.utils.logging import Debug, Warn


class SafeCorridor(Rectangle):
    """Class for representing a safe corridor for the robot to travel through. Wraps around a rectangle to provide a safe corridor for the robot to travel through."""

    def draw(self, render_scale: SpatialMeasurement):
        helpers.draw_fancy_rectangle(self, Palette.BLUE, render_scale)


def __GetExpandedRectangle(
    x: SpatialMeasurement,
    y: SpatialMeasurement,
    obstacles: List[DiscreteBoundary],
    alternate_obstacles: List[DiscreteBoundary] = None,
) -> SafeCorridor:
    INITIAL_STEP_SIZE = Meter(1)

    # Adding small amount to prevent it being a line is kinda scuff, but it works for now
    rectangle = [
        x,
        y,
        x + Centimeter(0.001),
        y + Centimeter(0.001),
    ]  # [min_x, min_y, max_x, max_y]
    expansion_order = [
        3,
        2,
        1,
        0,
    ]  # Order of expansion: 0 = min_x, 1 = min_y, 2 = max_x, 3 = max_y

    for i in expansion_order:
        step_size = INITIAL_STEP_SIZE

        # Gets direction of expansion...possibly a bit scuff
        if i < 2:
            step_size = -step_size

        # If already intersects with an obstacle, debug
        if any(
            obstacle.intersects_rectangle(
                rectangle[0], rectangle[1], rectangle[2], rectangle[3]
            )
            for obstacle in obstacles
        ):
            if not alternate_obstacles is None:
                Warn(
                    f"Initial rectangle intersects with obstacle: {rectangle}. Attempting to use alternate obstacles"
                )
                obstacles = alternate_obstacles
                if any(
                    obstacle.intersects_rectangle(
                        rectangle[0], rectangle[1], rectangle[2], rectangle[3]
                    )
                    for obstacle in obstacles
                ):
                    Warn(
                        f"Something really went wrong, initial rectangle still interesects with obstacles: {rectangle}. Skipping"
                    )
                    continue
            else:
                Debug(
                    f"Initial rectangle intersects with obstacle: {rectangle}. skipping"
                )
                continue

        # Expand until the rectangle intersects with an obstacle
        while not any(
            obstacle.intersects_rectangle(
                rectangle[0], rectangle[1], rectangle[2], rectangle[3]
            )
            for obstacle in obstacles
        ):
            Debug(f"Expanding rectangle: {rectangle} by {step_size} in direction {i}")
            rectangle[i] += step_size

        # Expand and backoff until rectangle perfectly fits within obstacles
        for x in range(4):
            Debug(f"Stage {x}: {rectangle}")
            while not any(
                obstacle.intersects_rectangle(
                    rectangle[0], rectangle[1], rectangle[2], rectangle[3]
                )
                for obstacle in obstacles
            ):

                rectangle[i] += step_size

            # Shrink back in decremental steps until the rectangle no longer intersects with an obstacle
            while any(
                obstacle.intersects_rectangle(
                    rectangle[0], rectangle[1], rectangle[2], rectangle[3]
                )
                for obstacle in obstacles
            ):

                step_size /= 2
                rectangle[i] -= step_size

    return SafeCorridor(
        rectangle[0],
        rectangle[1],
        rectangle[2] - rectangle[0],
        rectangle[3] - rectangle[1],
    )


def __EncloseVectorInRectangle(problem, x, y, rectangle: Rectangle):
    problem.subject_to(x >= rectangle.get_min_x().to(CALCULATION_UNIT_SPATIAL))
    problem.subject_to(x <= rectangle.get_max_x().to(CALCULATION_UNIT_SPATIAL))
    problem.subject_to(y >= rectangle.get_min_y().to(CALCULATION_UNIT_SPATIAL))
    problem.subject_to(y <= rectangle.get_max_y().to(CALCULATION_UNIT_SPATIAL))


SAFETY_CORRIDOR_DEBUG = []


def SafetyCorridor(obstacles: List[DiscreteBoundary]):
    """Returns a contraint function that ensures that the robot's path is within a safe corridor.

    :param obstacles: The obstacles that the robot must avoid.
    :type obstacles: List[:class:`DiscreteBoundary`]
    :return: The constraint function that ensures the robot's path is within a safe corridor.
    :rtype: Callable[[Problem, PointVariables], None]"""

    def __safety_corridor(problem, point_variables: PointVariables):
        safe_corridor: List[Rectangle] = []
        rect_obstacles = [obstacle.get_bounded_rectangle() for obstacle in obstacles]

        for i in range(len(point_variables.POS_X)):
            x = CALCULATION_UNIT_SPATIAL(point_variables.POS_X[i].value())
            y = CALCULATION_UNIT_SPATIAL(point_variables.POS_Y[i].value())
            found = False
            for rectangle in safe_corridor:
                if rectangle.contains_point(x, y):
                    __EncloseVectorInRectangle(
                        problem,
                        point_variables.POS_X[i],
                        point_variables.POS_Y[i],
                        rectangle,
                    )
                    found = True
                    break
            if not found:
                safe_corridor.append(
                    __GetExpandedRectangle(x, y, rect_obstacles, obstacles)
                )

                __EncloseVectorInRectangle(
                    problem,
                    point_variables.POS_X[i],
                    point_variables.POS_Y[i],
                    safe_corridor[-1],
                )
        global SAFETY_CORRIDOR_DEBUG
        SAFETY_CORRIDOR_DEBUG.extend(safe_corridor)

    return __safety_corridor
