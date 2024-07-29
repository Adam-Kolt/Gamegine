import math
from typing import Dict, List, Tuple
import pint

from gamegine.analysis.pathfinding import Path
from gamegine.analysis.trajectory.generation import (
    GuidedSwerveTrajectoryGenerator,
    SwerveDrivetrainParameters,
    SwerveTrajectory,
    TrajectoryKeypoint,
)
from gamegine.render import helpers
from gamegine.render.style import Palette
from gamegine.representation.bounds import DiscreteBoundary, Rectangle
from gamegine.utils.logging import Debug, Warn
from gamegine.utils.unit import Centimeter, Meter, SpatialMeasurement


class SafeCorridor(Rectangle):
    def draw(self, render_scale: SpatialMeasurement):
        helpers.draw_fancy_rectangle(self, Palette.BLUE, render_scale)


class SafetyCorridorAssisted(GuidedSwerveTrajectoryGenerator):
    def __init__(self, units_per_node: SpatialMeasurement = Centimeter(10)):
        self.safe_corridor: List[SafeCorridor] = []
        self.units_per_node = units_per_node

    def calculate_trajectory(
        self,
        guide_path: Path,
        obstacles: List[DiscreteBoundary],
        start_parameters: TrajectoryKeypoint,
        end_parameters: TrajectoryKeypoint,
        drivetrain_parameters: SwerveDrivetrainParameters,
    ) -> SwerveTrajectory:
        self.dissected_path = guide_path.dissected(units_per_node=self.units_per_node)
        corridor = self.__GenerateSafeCorridor(
            self.dissected_path.get_points(), obstacles
        )
        self.safe_corridor = corridor[0]
        Debug(f"Safe corridor: {self.safe_corridor}")
        Debug(f"Path map: {corridor[1]}")
        pass

    def __GetExpandedRectangle(
        self,
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
        rect_obstacles = [obstacle.get_bounded_rectangle() for obstacle in obstacles]

        safe_corridor.append(
            self.__GetExpandedRectangle(
                path[0][0], path[0][1], rect_obstacles, obstacles
            )
        )
        path_map[0] = 0

        for i in range(1, len(path)):
            last_rectangle = safe_corridor[-1]
            if not last_rectangle.contains_point(path[i][0], path[i][1]):
                safe_corridor.append(
                    self.__GetExpandedRectangle(
                        path[i][0],
                        path[i][1],
                        rect_obstacles,
                        alternate_obstacles=obstacles,
                    )
                )
            path_map[i] = len(safe_corridor) - 1

        Debug(
            f"Generated Safe Corridor with {len(safe_corridor)} rectangles for path containing {len(path)} points"
        )
        return safe_corridor, path_map

    def GetSafeCorridor(self) -> List[SafeCorridor]:
        return self.safe_corridor
