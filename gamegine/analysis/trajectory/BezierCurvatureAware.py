# Approach based on paper: "Autonomous Trajectory Planning Based on Bézier Curve with Curvature Constraints and Piecewise-Jerk Speed-Time Optimization" by You Wang, et al.



from typing import List, Tuple
import pint

from gamegine.analysis.trajectory.generation import TrajectoryParameters, TrajectoryGenerator, Trajectory
from gamegine.render import helpers
from gamegine.render.style import Palette
from gamegine.representation.bounds import DiscreteBoundary, Rectangle
from gamegine.utils.logging import Debug
from gamegine.utils.unit import Centimeter, Meter, SpatialMeasurement


class SafeCorridor(Rectangle):
    def draw(self, render_scale: SpatialMeasurement):
        helpers.draw_fancy_rectangle(self, Palette.BLUE, render_scale)


class BezierCurvatureAware(TrajectoryGenerator):
    def calculate_trajectory(self, path: List[Tuple[pint.Quantity, pint.Quantity]], parameters: TrajectoryParameters, obstacles: List[DiscreteBoundary]) -> Trajectory:
        self.safe_corridor = self.__GenerateSafeCorridor(path, obstacles)
        Debug(f"Safe corridor: {self.safe_corridor}")
        pass


    def __GetExpandedRectangle(self, x: SpatialMeasurement, y: SpatialMeasurement, obstacles: List[DiscreteBoundary]) -> SafeCorridor:
        INITIAL_STEP_SIZE = Meter(1)

        # TODO: Adding small amount to prevent it being a line is kinda scuff, but it works for now
        rectangle = [x, y, x+Centimeter(1), y+Centimeter(1)] # [min_x, min_y, max_x, max_y] 
        expansion_order = [3, 2, 1, 0] # Order of expansion: 0 = min_x, 1 = min_y, 2 = max_x, 3 = max_y

        for i in expansion_order:
            step_size = INITIAL_STEP_SIZE

            # Gets direction of expansion...possibly a bit scuff
            if i < 2:
                step_size = -step_size

            # Expand until the rectangle intersects with an obstacle
            while not any(obstacle.intersects_rectangle(rectangle[0], rectangle[1], rectangle[2], rectangle[3]) for obstacle in obstacles):
                Debug(f"Expanding rectangle: {rectangle} by {step_size} in direction {i}")
                rectangle[i] += step_size

            # Expand and backoff until rectangle perfectly fits within obstacles
            for x in range(4):
                while not any(obstacle.intersects_rectangle(rectangle[0], rectangle[1], rectangle[2], rectangle[3]) for obstacle in obstacles):
                    rectangle[i] += step_size
            
                # Shrink back in decremental steps until the rectangle no longer intersects with an obstacle
                while any(obstacle.intersects_rectangle(rectangle[0], rectangle[1], rectangle[2], rectangle[3]) for obstacle in obstacles):
                    step_size /= 2
                    rectangle[i] -= step_size

                

        return SafeCorridor(rectangle[0], rectangle[1], rectangle[2]-rectangle[0], rectangle[3]-rectangle[1])

    def __GenerateSafeCorridor(self, path: List[Tuple[SpatialMeasurement, SpatialMeasurement]], obstacles: List[DiscreteBoundary]) -> List[SafeCorridor]:
        safe_corridor: List[Rectangle] = []
        rect_obstacles = [obstacle.get_bounded_rectangle() for obstacle in obstacles] # Convert obstacles to rectangles for better corridor expansion

        safe_corridor.append(self.__GetExpandedRectangle(path[0][0], path[0][1], rect_obstacles))
        for i in range(1, len(path)):
            last_rectangle = safe_corridor[-1]
            if last_rectangle.contains_point(path[i][0], path[i][1]):
                continue
            safe_corridor.append(self.__GetExpandedRectangle(path[i][0], path[i][1], rect_obstacles))

        return safe_corridor
    
    def GetSafeCorridor(self) -> List[SafeCorridor]:
        return self.safe_corridor