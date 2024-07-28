import math
from typing import Dict, List, Tuple
import pint

from gamegine.analysis.pathfinding import Path
from gamegine.analysis.trajectory.generation import (
    HolonomicTrajectoryParameters,
    TrajectoryGenerator,
    Trajectory,
)
from gamegine.render import helpers
from gamegine.render.style import Palette
from gamegine.representation.bounds import DiscreteBoundary, Rectangle
from gamegine.utils.logging import Debug, Warn
from gamegine.utils.unit import Centimeter, Meter, SpatialMeasurement


class SafeCorridor(Rectangle):
    def draw(self, render_scale: SpatialMeasurement):
        helpers.draw_fancy_rectangle(self, Palette.BLUE, render_scale)


class SafetyCorridorAssisted(TrajectoryGenerator):
    def __init__(self, units_per_node: SpatialMeasurement = Centimeter(10)):
        self.safe_corridor: List[SafeCorridor] = []
        self.units_per_node = units_per_node

    def calculate_trajectory(
        self,
        path: Path,
        parameters: HolonomicTrajectoryParameters,
        obstacles: List[DiscreteBoundary],
    ) -> Trajectory:
        self.dissected_path = path.dissected(units_per_node=self.units_per_node)
        corridor = self.__GenerateSafeCorridor(
            self.dissected_path.get_points(), obstacles
        )
        self.safe_corridor = corridor[0]
        Debug(f"Safe corridor: {self.safe_corridor}")
        Debug(f"Path map: {corridor[1]}")
        pass

    def __DissectPath(
        self, path: Path
    ) -> List[Tuple[SpatialMeasurement, SpatialMeasurement]]:
        path = path.get_points()
        disected_path = []
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            distance = math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
            steps = int(distance / self.units_per_node)
            for j in range(steps):
                disected_path.append(
                    (
                        a[0] + (b[0] - a[0]) * j / steps,
                        a[1] + (b[1] - a[1]) * j / steps,
                    )
                )
        disected_path.append(path[-1])
        return disected_path

    def __GetExpandedRectangle(
        self,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        obstacles: List[DiscreteBoundary],
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
                Warn(
                    f"Initial rectangle intersects with obstacle: {rectangle}. Skipping."
                )
                continue

            # Expand until the rectangle intersects with an obstacle
            while not any(
                obstacle.intersects_rectangle(
                    rectangle[0], rectangle[1], rectangle[2], rectangle[3]
                )
                for obstacle in obstacles
            ):
                Debug(
                    f"Expanding rectangle: {rectangle} by {step_size} in direction {i}"
                )
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

    def __GenerateSafeCorridor(
        self,
        path: List[Tuple[SpatialMeasurement, SpatialMeasurement]],
        obstacles: List[DiscreteBoundary],
    ) -> Tuple[List[SafeCorridor], Dict[int, int]]:
        safe_corridor: List[Rectangle] = []
        path_map: Dict[int, int] = {}
        rect_obstacles = [
            obstacle for obstacle in obstacles
        ]  # A Remnant of the past...for now

        safe_corridor.append(
            self.__GetExpandedRectangle(path[0][0], path[0][1], rect_obstacles)
        )
        path_map[0] = 0

        for i in range(1, len(path)):
            last_rectangle = safe_corridor[-1]
            if not last_rectangle.contains_point(path[i][0], path[i][1]):
                safe_corridor.append(
                    self.__GetExpandedRectangle(path[i][0], path[i][1], rect_obstacles)
                )
            path_map[i] = len(safe_corridor) - 1

        Debug(
            f"Generated Safe Corridor with {len(safe_corridor)} rectangles for path containing {len(path)} points"
        )
        return safe_corridor, path_map

    def GetSafeCorridor(self) -> List[SafeCorridor]:
        return self.safe_corridor
