# Approach based on paper: "Autonomous Trajectory Planning Based on Bézier Curve with Curvature Constraints and Piecewise-Jerk Speed-Time Optimization" by You Wang, et al.



from typing import List, Tuple
import pint

from gamegine.analysis.trajectory.generation import TrajectoryParameters, TrajectoryGenerator, Trajectory
from gamegine.representation.bounds import DiscreteBoundary, Rectangle




class BezierCurvatureAware(TrajectoryGenerator):
    def calculate_trajectory(self, path: List[Tuple[pint.Quantity, pint.Quantity]], parameters: TrajectoryParameters, obstacles: List[DiscreteBoundary]) -> Trajectory:
        pass


    def __GetExpandedRectangle(self, x: pint.Quantity, y: pint.Quantity, obstacles: List[DiscreteBoundary]) -> Rectangle:
        pass

    def __GenerateSafeCorridor(self, path: List[Tuple[pint.Quantity, pint.Quantity]], obstacles: List[DiscreteBoundary]) -> List[Rectangle]:
        safe_corridor = []

        for i in range(1, len(path)):
            pass