from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

from gamegine.analysis import pathfinding
from gamegine.analysis.meshing import Map, TriangulatedGraph
from gamegine.analysis.trajectory.lib.TrajGen import SwerveTrajectory
from gamegine.analysis.trajectory.generator import TrajectoryGenerator, SplineTrajectoryGenerator
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
from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond
from gamegine.utils.NCIM.ComplexDimensions.acceleration import MeterPerSecondSquared
from gamegine.utils.NCIM.Dimensions.spatial import Meter
from gamegine.analysis.trajectory.dynamic_obstacles import TrajectoryObstacle, DynamicObstacle


def _default_trajectory_generator():
    """Factory for default SplineTrajectoryGenerator."""
    return SplineTrajectoryGenerator(
        MetersPerSecond(5.0),
        MeterPerSecondSquared(9.8),
        Meter(0.1),
        Meter(0.15),
    )


@dataclass
class PhysicsConfig:
    """Configuration for the PhysicsEngine.
    
    :param mesh_resolution: Resolution for the navigation mesh.
    :param discretization_quality: Quality of obstacle discretization.
    :param trajectory_resolution: Resolution for trajectory generation.
    :param stretch_factor: Stretch factor for meshing.
    :param min_spacing: Minimum spacing between nodes.
    :param trajectory_generator: The trajectory generator to use (defaults to SplineTrajectoryGenerator).
    """
    mesh_resolution: SpatialMeasurement = Feet(2)
    discretization_quality: int = 4
    trajectory_resolution: SpatialMeasurement = Centimeter(10)
    stretch_factor: float = 1.5
    min_spacing: SpatialMeasurement = Centimeter(8)
    trajectory_generator: Optional[TrajectoryGenerator] = field(default_factory=_default_trajectory_generator)


@dataclass
class TraversalSpace:
    """Represents the navigable space for a robot, including map and obstacles."""
    traversal_map: Map
    obstacles: List[obstacle.Obstacle] = field(default_factory=list)


class PhysicsEngine:
    """Handles all physics-related calculations including pathfinding and trajectory generation."""

    def __init__(self, config: PhysicsConfig = PhysicsConfig()):
        """Initializes the PhysicsEngine.

        :param config: The physics configuration to use.
        :type config: PhysicsConfig
        """
        self.config = config
        self.robot_traversal_space: Dict[str, TraversalSpace] = {}
        self.trajectories: Dict[str, Dict] = {}
        self.latest_trajectory: SwerveTrajectory = None
        # Use configured trajectory generator (defaults to SplineTrajectoryGenerator)
        self.trajectory_generator = config.trajectory_generator
        # Active trajectories for dynamic obstacle avoidance
        # Maps robot_name -> (trajectory, start_time, robot_radius)
        self.active_trajectories: Dict[str, Tuple[SwerveTrajectory, float, float]] = {}

    def prepare_traversal_space(
        self,
        robot_name: str,
        robot: SwerveRobot,
        game_obstacles: List[obstacle.Obstacle],
        field_size: Tuple[SpatialMeasurement, SpatialMeasurement],
    ) -> TraversalSpace:
        """Generates and caches the traversal space for a specific robot.

        :param robot_name: The name of the robot.
        :param robot: The robot instance (used for bounding radius).
        :param game_obstacles: List of game obstacles.
        :param field_size: Dimensions of the field (width, height).
        :return: The generated TraversalSpace.
        :rtype: TraversalSpace
        """
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
        """Finds a path for the robot from start to target.

        :param robot_name: The name of the robot.
        :param start_x: Starting X coordinate.
        :param start_y: Starting Y coordinate.
        :param target_x: Target X coordinate.
        :param target_y: Target Y coordinate.
        :param traversal_space: The traversal space to use for pathfinding.
        :return: A Path object representing the route.
        :rtype: pathfinding.Path
        """
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
        start_time: float = 0.0,
        avoid_other_robots: bool = True,
        battery_model = None,
    ) -> SwerveTrajectory:
        """Generates a swerve trajectory for the robot.

        :param robot_name: The name of the robot.
        :param robot: The robot instance.
        :param start_state: Tuple of (x, y, heading) for start.
        :param target_state: Tuple of (x, y, heading) for target.
        :param path: The path to follow.
        :param traversal_space: The traversal space definition.
        :param no_safety_corridor: If True, skips safety corridor generation.
        :param start_time: Absolute start time for this trajectory (for dynamic obstacle avoidance).
        :param avoid_other_robots: If True, avoid other robots' active trajectories.
        :param battery_model: Optional BatteryModel for voltage-aware acceleration limiting.
        :return: The generated SwerveTrajectory.
        :rtype: SwerveTrajectory
        """
        start_x, start_y, start_heading = start_state
        x, y, heading = target_state

        trajectory_key = (start_x, start_y, start_heading, x, y, heading)
        # Don't use cache when avoiding other robots (dynamic obstacles change each time)
        if not avoid_other_robots and trajectory_key in self.trajectories.get(robot_name, {}):
            return self.trajectories[robot_name][trajectory_key]
        
        # Get dynamic obstacles from other robots
        dynamic_obstacles = None
        if avoid_other_robots:
            dynamic_obstacles = self.get_dynamic_obstacles_for(
                robot_name, 
                start_time, 
                robot.get_bounding_radius().to(Meter) if hasattr(robot.get_bounding_radius(), 'to') else float(robot.get_bounding_radius())
            )

        trajectory = self.trajectory_generator.generate(
            robot_name,
            robot,
            start_state,
            target_state,
            path,
            traversal_space,
            constraints=self.config,
            no_safety_corridor=no_safety_corridor,
            dynamic_obstacles=dynamic_obstacles,
            trajectory_start_time=start_time,
            battery_model=battery_model,
        )

        if robot_name not in self.trajectories:
            self.trajectories[robot_name] = {}

        self.trajectories[robot_name][trajectory_key] = trajectory
        self.latest_trajectory = trajectory
        return trajectory

    def get_latest_trajectory(self) -> SwerveTrajectory:
        """Returns the most recently generated trajectory.

        :return: The latest SwerveTrajectory.
        :rtype: SwerveTrajectory
        """
        return self.latest_trajectory

    def register_active_trajectory(
        self,
        robot_name: str,
        trajectory: SwerveTrajectory,
        start_time: float,
        robot_radius: float,
    ) -> None:
        """Register a trajectory as active for dynamic obstacle avoidance.
        
        Other robots generating trajectories will avoid this robot.
        
        :param robot_name: Name of the robot following this trajectory.
        :param trajectory: The SwerveTrajectory being followed.
        :param start_time: Absolute start time of the trajectory (seconds).
        :param robot_radius: Bounding radius of the robot (meters).
        """
        self.active_trajectories[robot_name] = (trajectory, start_time, robot_radius)
    
    def clear_active_trajectory(self, robot_name: str) -> None:
        """Clear an active trajectory when the robot completes its action.
        
        :param robot_name: Name of the robot whose trajectory to clear.
        """
        self.active_trajectories.pop(robot_name, None)
    
    def clear_all_active_trajectories(self) -> None:
        """Clear all active trajectories (e.g., on environment reset)."""
        self.active_trajectories.clear()
    
    def get_dynamic_obstacles_for(
        self,
        robot_name: str,
        query_start_time: float,
        query_robot_radius: float,
    ) -> List[DynamicObstacle]:
        """Get dynamic obstacles representing other robots' active trajectories.
        
        :param robot_name: Name of the robot doing the query (excluded from obstacles).
        :param query_start_time: When the querying robot's trajectory will start.
        :param query_robot_radius: Bounding radius of the querying robot.
        :return: List of TrajectoryObstacle instances for all other robots.
        """
        obstacles = []
        for other_name, (traj, start_time, radius) in self.active_trajectories.items():
            if other_name == robot_name:
                continue  # Don't avoid yourself
            obstacles.append(TrajectoryObstacle(
                trajectory=traj,
                start_time=start_time,
                robot_radius=radius,
                query_robot_radius=query_robot_radius,
            ))
        return obstacles
