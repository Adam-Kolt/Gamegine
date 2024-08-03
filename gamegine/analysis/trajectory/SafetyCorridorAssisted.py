import math
from typing import Dict, List, Tuple
import pint

from gamegine.analysis.pathfinding import Path
from gamegine.analysis.trajectory.generation import (
    GuidedSwerveTrajectoryGenerator,
    InterpolatedSwerveTrajectory,
    SwerveTrajectory,
    TrajectoryKeypoint,
    TrajectoryState,
)
from gamegine.render import helpers
from gamegine.render.style import Palette
from gamegine.representation.bounds import DiscreteBoundary, Rectangle
from gamegine.representation.robot import (
    PhysicalParameters,
    SwerveDrivetrainCharacteristics,
)
from gamegine.utils.logging import Debug, Warn
from gamegine.utils.unit import (
    Centimeter,
    ComplexUnits,
    MassMeasurement,
    Meter,
    Radian,
    SpatialMeasurement,
    Units,
)
from jormungandr.optimization import OptimizationProblem
import numpy as np


class SafeCorridor(Rectangle):
    def draw(self, render_scale: SpatialMeasurement):
        helpers.draw_fancy_rectangle(self, Palette.BLUE, render_scale)


class SafetyCorridorAssisted(GuidedSwerveTrajectoryGenerator):
    def __init__(self, units_per_node: SpatialMeasurement = Centimeter(20)):
        self.safe_corridor: List[SafeCorridor] = []
        self.units_per_node = units_per_node

    def __get_max_acc_ms(
        self, mass: MassMeasurement, drivetrain: SwerveDrivetrainCharacteristics
    ):
        return drivetrain.module.max_module_force.to(Units.Force.Newton) / mass.to(
            Units.Mass.Kilogram
        )

    def __bound_in_rect(self, rect: Rectangle, problem, x, y):
        problem.subject_to(x > rect.get_min_x().to(Units.Spatial.Meter))
        problem.subject_to(x < rect.get_max_x().to(Units.Spatial.Meter))
        problem.subject_to(y > rect.get_min_y().to(Units.Spatial.Meter))
        problem.subject_to(y < rect.get_max_y().to(Units.Spatial.Meter))

    def __apply_boundary_constraints(
        self,
        problem,
        corridor: List[SafeCorridor],
        corridor_path_map: Dict[int, int],
        x_s: List,
        y_s: List,
    ):
        for i in range(len(x_s)):
            c_index = corridor_path_map[i]
            c = corridor[c_index]
            c.get_vertices()
            self.__bound_in_rect(c, problem, x_s[i], y_s[i])

    def __sleipnir_optimize(
        self,
        path: Path,
        corridor: List[SafeCorridor],
        corridor_path_map: Dict[int, int],
        start: TrajectoryKeypoint,
        end: TrajectoryKeypoint,
        physics: PhysicalParameters,
        drivetrain: SwerveDrivetrainCharacteristics,
    ):
        # Basic Constraints
        MAX_SPEED = drivetrain.module.max_module_speed.to(
            ComplexUnits.Velocity.MeterPerSecond
        )
        MAX_ACC = self.__get_max_acc_ms(physics.mass, drivetrain)

        Debug("Creating Optimization Problem")
        trajectory_problem = OptimizationProblem()

        initial_points = [
            (x.to(Units.Spatial.Meter), y.to(Units.Spatial.Meter))
            for x, y in path.get_points()
        ]
        steps = len(initial_points)

        X = trajectory_problem.decision_variable(steps)
        Y = trajectory_problem.decision_variable(steps)
        VX = trajectory_problem.decision_variable(steps)
        VY = trajectory_problem.decision_variable(steps)
        AX = trajectory_problem.decision_variable(steps - 1)
        AY = trajectory_problem.decision_variable(steps - 1)
        DT = trajectory_problem.decision_variable(steps - 1)

        # Apply "Endpoint" Constraints
        # Start
        trajectory_problem.subject_to(X[0] == initial_points[0][0])
        trajectory_problem.subject_to(Y[0] == initial_points[0][1])
        trajectory_problem.subject_to(
            VX[0] == start.velocity_x.to(ComplexUnits.Velocity.MeterPerSecond)
        )
        trajectory_problem.subject_to(
            VY[0] == start.velocity_y.to(ComplexUnits.Velocity.MeterPerSecond)
        )

        # End
        trajectory_problem.subject_to(X[-1] == initial_points[-1][0])
        trajectory_problem.subject_to(Y[-1] == initial_points[-1][1])
        trajectory_problem.subject_to(
            VX[-1] == start.velocity_x.to(ComplexUnits.Velocity.MeterPerSecond)
        )
        trajectory_problem.subject_to(
            VY[-1] == start.velocity_y.to(ComplexUnits.Velocity.MeterPerSecond)
        )

        self.__apply_boundary_constraints(
            trajectory_problem, corridor, corridor_path_map, X, Y
        )

        # Kinematics
        for i in range(steps - 1):
            dt = DT[i]
            trajectory_problem.subject_to(dt > 0.0)
            dt.set_value(0.2)

            curr_x = X[i]
            curr_y = Y[i]

            curr_vx = VX[i]
            curr_vy = VY[i]
            trajectory_problem.subject_to(curr_vx**2 + curr_vy**2 < MAX_SPEED**2)

            curr_ax = AX[i]
            curr_ay = AY[i]
            trajectory_problem.subject_to(curr_ax**2 + curr_ay**2 < MAX_ACC**2)

            next_x = X[i + 1]
            next_y = Y[i + 1]
            trajectory_problem.subject_to(next_x == curr_x + curr_vx * dt)
            trajectory_problem.subject_to(next_y == curr_y + curr_vy * dt)

            next_vx = VX[i + 1]
            next_vy = VY[i + 1]
            trajectory_problem.subject_to(next_vx == curr_vx + curr_ax * dt)
            trajectory_problem.subject_to(next_vy == curr_vy + curr_ay * dt)

            # Space Out
            trajectory_problem.subject_to(
                (next_x - curr_x) ** 2 + (next_y - curr_y) ** 2
                > (self.units_per_node.to(Units.Spatial.Meter) / 3) ** 2
            )

            # Not too much though
            trajectory_problem.subject_to(
                (next_x - curr_x) ** 2 + (next_y - curr_y) ** 2
                < (self.units_per_node.to(Units.Spatial.Meter) * 1.5) ** 2
            )

        # Initial Guesses
        for i in range(steps):
            X[i].set_value(initial_points[i][0])
            Y[i].set_value(initial_points[i][1])

        # Minimize time
        total = 0
        for i in range(steps - 1):
            dt = DT[i]
            total += dt

        trajectory_problem.minimize(total)

        trajectory_problem.solve(tolerance=1e-11)

        points = []
        for i in range(steps):
            points.append((Meter(X.value(i)), Meter(Y.value(i))))

        return points

    def calculate_trajectory(
        self,
        guide_path: Path,
        obstacles: List[DiscreteBoundary],
        start_parameters: TrajectoryKeypoint,
        end_parameters: TrajectoryKeypoint,
        physical_parameters: PhysicalParameters,
        drivetrain_parameters: SwerveDrivetrainCharacteristics,
    ) -> SwerveTrajectory:
        self.dissected_path = guide_path.dissected(units_per_node=self.units_per_node)
        self.drivetrain = drivetrain_parameters
        self.start = start_parameters
        self.end = end_parameters

        corridor = self.__GenerateSafeCorridor(
            self.dissected_path.get_points(), obstacles
        )
        self.safe_corridor = corridor[0]
        Debug(f"Safe corridor: {self.safe_corridor}")

        trajectory_path = self.__sleipnir_optimize(
            self.dissected_path,
            self.safe_corridor,
            corridor[1],
            start_parameters,
            end_parameters,
            physical_parameters,
            drivetrain_parameters,
        )

        return InterpolatedSwerveTrajectory(
            [
                TrajectoryState(point[0], point[1], Radian(0))
                for point in trajectory_path
            ]
        )

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
