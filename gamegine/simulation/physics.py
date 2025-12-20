from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from gamegine.analysis import pathfinding
from gamegine.analysis.meshing import Map, TriangulatedGraph
from gamegine.analysis.trajectory.lib.TrajGen import SwerveTrajectory
from gamegine.analysis.trajectory.sleipnir import SleipnirOfflineGenerator
from gamegine.representation import obstacle
from gamegine.representation.bounds import ExpandedObjectBounds
from gamegine.representation.robot import SwerveRobot
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement
from gamegine.utils.NCIM.Dimensions.spatial import (
    Centimeter,
    Feet,
    Inch,
    SpatialMeasurement,
)


@dataclass
class PhysicsConfig:
    mesh_resolution: SpatialMeasurement = Feet(2)
    discretization_quality: int = 4
    trajectory_resolution: SpatialMeasurement = Centimeter(10)
    stretch_factor: float = 1.5
    min_spacing: SpatialMeasurement = Centimeter(8)


@dataclass
class TraversalSpace:
    traversal_map: Map
    obstacles: List[obstacle.Obstacle] = field(default_factory=list)


class PhysicsEngine:
    """Handles all physics-related calculations including pathfinding and trajectory generation."""

    def __init__(self, config: PhysicsConfig = PhysicsConfig()):
        self.config = config
        self.robot_traversal_space: Dict[str, TraversalSpace] = {}
        self.trajectories: Dict[str, Dict] = {}
        self.latest_trajectory: SwerveTrajectory = None
        self.trajectory_generator = SleipnirOfflineGenerator()

    def prepare_traversal_space(
        self,
        robot_name: str,
        robot: SwerveRobot,
        game_obstacles: List[obstacle.Obstacle],
        field_size: Tuple[SpatialMeasurement, SpatialMeasurement],
    ) -> TraversalSpace:
        """Generates the expanded obstacles for the game."""
        if robot_name in self.robot_traversal_space:
            return self.robot_traversal_space[robot_name]

        obstacles = ExpandedObjectBounds(
            game_obstacles,
            robot.get_bounding_radius(),
            self.config.discretization_quality,
        )

        map_obstacles = ExpandedObjectBounds(
            game_obstacles,
            Inch(2) + robot.get_bounding_radius(),
            self.config.discretization_quality,
        )

        triangle_map = TriangulatedGraph(
            map_obstacles,
            self.config.mesh_resolution,
            field_size,
            self.config.discretization_quality,
        )

        traversal_space = TraversalSpace(
            traversal_map=triangle_map, obstacles=obstacles
        )

        self.robot_traversal_space[robot_name] = traversal_space

        return traversal_space

    def pathfind(
        self,
        robot_name: str,
        start_x: SpatialMeasurement,
        start_y: SpatialMeasurement,
        target_x: SpatialMeasurement,
        target_y: SpatialMeasurement,
        traversal_space: TraversalSpace,
    ) -> pathfinding.Path:
        """Pathfinds a robot to a new position."""
        path = pathfinding.findPath(
            traversal_space.traversal_map,
            (start_x, start_y),
            (target_x, target_y),
            pathfinding.AStar,
            pathfinding.InitialConnectionPolicy.ConnectToClosest,
        )
        path.shortcut(traversal_space.obstacles)
        return path

    def generate_trajectory(
        self,
        robot_name: str,
        robot: SwerveRobot,
        start_state: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        target_state: Tuple[SpatialMeasurement, SpatialMeasurement, AngularMeasurement],
        path: pathfinding.Path,
        traversal_space: TraversalSpace,
        no_safety_corridor=False,
    ) -> SwerveTrajectory:
        start_x, start_y, start_heading = start_state
        x, y, heading = target_state

        trajectory_key = (start_x, start_y, start_heading, x, y, heading)
        if trajectory_key in self.trajectories.get(robot_name, {}):
            return self.trajectories[robot_name][trajectory_key]

        trajectory = self.trajectory_generator.generate(
            robot_name,
            robot,
            start_state,
            target_state,
            path,
            traversal_space,
            constraints=self.config,
            no_safety_corridor=no_safety_corridor
        )

        if robot_name not in self.trajectories:
            self.trajectories[robot_name] = {}

        self.trajectories[robot_name][trajectory_key] = trajectory
        self.latest_trajectory = trajectory
        return trajectory

    def get_latest_trajectory(self) -> SwerveTrajectory:
        return self.latest_trajectory
