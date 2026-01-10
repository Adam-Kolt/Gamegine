from dataclasses import dataclass, field
import math
from typing import Dict, List, Tuple
from abc import ABC, abstractmethod

from gamegine.analysis import pathfinding
from gamegine.analysis.meshing import Map
from gamegine.analysis.trajectory.lib.TrajGen import SwerveTrajectory
from gamegine.simulation.physics import PhysicsEngine, PhysicsConfig
from gamegine.representation import obstacle
from gamegine.representation.bounds import ExpandedObjectBounds
from gamegine.representation.game import Game
from gamegine.representation.interactable import (
    RobotInteractable,
    RobotInteractionConfig,
)
from gamegine.representation.robot import SwerveRobot
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.simulation.state import ValueChange, ValueIncrease
from gamegine.simulation.match import MatchController
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement, Radian
from gamegine.utils.NCIM.Dimensions.spatial import (
    SpatialMeasurement,
    Inch,
    Centimeter,
    Feet,
)
from gamegine.utils.NCIM.ncim import (
    MetersPerSecond,
    MeterPerSecondSquared,
    RadiansPerSecond,
    RadiansPerSecondSquared,
    Second,
    Meter,
)
from gamegine.simulation.rules import RuleEngine
from gamegine.utils.logging import Info, Warn, Debug


@dataclass
class ServerConfig:
    mesh_resolution: SpatialMeasurement = Feet(2)
    discretization_quality: int = 4
    physics_config: PhysicsConfig = field(default_factory=PhysicsConfig)
    fast_mode: bool = False  # Skip trajectory generation, use teleport with time estimate
    default_max_velocity: float = 5.0  # m/s for fast mode time estimation


@dataclass
class TraversalSpace:
    traversal_map: Map
    obstacles: List[obstacle.Obstacle] = field(default_factory=list)


class AbstractGameServer(ABC):
    """
    Base class for game servers (Discrete/Continuous).
    Manages the MatchController and delegates movement to subclasses.
    """
    
    ROBOT_PREFIX = "Robot "

    def __init__(self, config: ServerConfig = ServerConfig()) -> None:
        self.config = config
        self.match = MatchController()
        # Proxies for compatibility or ease of access
        self.robots = self.match.robots 
        
    def load_from_game(self, game: Game) -> None:
        self.match.load_game(game)

    def update(self, dt: float) -> bool:
        """Updates the game state (Time). Subclasses may add physics stepping."""
        return self.match.update_time(dt)

    def add_interactable(self, interactable: RobotInteractable) -> None:
        self.match.add_interactable(interactable)

    def get_obstacles(self) -> List[obstacle.Obstacle]:
        return self.match.get_obstacles()
        
    def set_obstacles(self, obstacles: List[obstacle.Obstacle]) -> None:
        self.match.set_obstacles(obstacles)

    def get_actions_set(self, robot_name: str) -> List[Tuple[str, str]]:
        return self.match.get_actions_set(robot_name)
    
    def process_action(self, interactable_name, interaction_name, robot_name, time_cutoff=None) -> bool:
        return self.match.process_action(interactable_name, interaction_name, robot_name, time_cutoff)
    
    def pickup_gamepiece(self, robot_name: str, gamepiece) -> None:
        self.match.pickup_gamepiece(robot_name, gamepiece)
        
    def get_log(self):
        return self.match.get_log()
        
    @abstractmethod
    def drive_robot(self, robot_name: str, x: SpatialMeasurement, y: SpatialMeasurement, theta: AngularMeasurement, no_safety_corridor=False) -> None:
        pass
    
    # Expose game_state as property for backward compatibility
    @property
    def game_state(self):
        return self.match.game_state
    
    @game_state.setter
    def game_state(self, value):
        self.match.game_state = value

    # Deprecated/Forwarded methods
    def init_game_state(self, state):
        self.match.game_state = state # Assuming we can overwrite. 
        # MatchController usually inits its own state in load_game.
        # But for tests or manual setup:
        if "interactables" not in state.spaces:
             state.createSpace("interactables")
        if "robots" not in state.spaces:
             state.createSpace("robots")

    def add_robot(self, robot: SwerveRobot) -> None:
        """Adds a robot to the game server.

        :param robot: The robot to add.
        :type robot: :class:`SwerveRobot`
        """
        name = robot.name.replace(self.ROBOT_PREFIX, "")
        self.match.add_robot(name, robot)

    def init_robot(self, robot_name: str, robot_state: RobotState) -> None:
        """Initializes a robot with the given state.

        :param robot_name: The name of the robot.
        :type robot_name: str
        :param robot_state: The state of the robot.
        :type robot_state: :class:`RobotState`
        """
        self.match.init_robot_state(robot_name, robot_state)


class DiscreteGameServer(AbstractGameServer):
    """
    Discrete Environment Server.
    Moves robots instanstly by calculating trajectory cost.
    Equivalent to original GameServer.
    """
    
    def __init__(self, config: ServerConfig = ServerConfig()):
        super().__init__(config)
        self.physics_engine = PhysicsEngine(config.physics_config)
    
    def drive_robot(
        self,
        robot_name: str,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        theta: AngularMeasurement,
        no_safety_corridor=False,
    ) -> None:
        if robot_name not in self.robots:
            raise ValueError("Robot not added")

        robot_state: RobotState = self.match.game_state.get("robots").get(robot_name)

        if (x, y) == (robot_state.x.get(), robot_state.y.get()):
            Debug(f"Robot {robot_name} is already at ({x}, {y}), no need to move")
            return

        # Get current time before trajectory generation (for dynamic obstacle avoidance)
        current_time = self.match.game_state.current_time.get()
        
        path = self.__pathfind(robot_name, x, y)
        trajectory = self.__trajectory(
            robot_name, x, y, theta, path, no_safety_corridor, current_time
        )
        
        # Register trajectory for dynamic obstacle avoidance by other robots
        robot = self.robots[robot_name]
        radius = robot.get_bounding_radius()
        radius_m = radius.to(Meter) if hasattr(radius, 'to') else float(radius)
        self.physics_engine.register_active_trajectory(
            robot_name, trajectory, current_time, radius_m
        )
        
        changes = [
            ValueChange(robot_state.x, x),
            ValueChange(robot_state.y, y),
            ValueChange(robot_state.heading, theta),
            ValueIncrease(
                self.match.game_state.current_time,
                trajectory.get_travel_time().to(Second),
            ),
        ]

        # Use match to apply changes AND logging logic?
        # Match.process_action handles interactions. 
        # Here we just have changes. 
        self.match.apply_changes(changes)

        Info(
            f"Robot {robot_name} moved to ({x}, {y}, {theta}), taking {trajectory.get_travel_time()} seconds, resulting in {changes}"
        )
        self.match.log(
            f"Robot {robot_name} moved to ({x}, {y}, {theta}), taking {trajectory.get_travel_time()} seconds",
            changes,
        )
        
        return trajectory  # Return trajectory for animation

    def drive_robot_fast(
        self,
        robot_name: str,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        theta: AngularMeasurement,
    ) -> float:
        """Fast mode: teleport robot with time estimation (no trajectory).
        
        Returns the estimated travel time in seconds.
        """
        if robot_name not in self.robots:
            raise ValueError("Robot not added")

        robot_state: RobotState = self.match.game_state.get("robots").get(robot_name)
        
        # Calculate straight-line distance
        current_x = robot_state.x.get().to(Meter)
        current_y = robot_state.y.get().to(Meter)
        target_x = x.to(Meter) if hasattr(x, 'to') else float(x)
        target_y = y.to(Meter) if hasattr(y, 'to') else float(y)
        
        distance = math.hypot(target_x - current_x, target_y - current_y)
        travel_time = distance / self.config.default_max_velocity
        
        # Teleport instantly
        changes = [
            ValueChange(robot_state.x, x),
            ValueChange(robot_state.y, y),
            ValueChange(robot_state.heading, theta),
            ValueIncrease(self.match.game_state.current_time, travel_time),
        ]
        self.match.apply_changes(changes)
        
        return travel_time

    def soft_reset(self) -> None:
        """Reset game state without recreating server (for pooling)."""
        self.match.reset()
        self.physics_engine.clear_all_active_trajectories()

    def prepare_traversal_space(self, robot_name: str) -> TraversalSpace:
        if robot_name not in self.robots:
            raise ValueError("Robot not added")
        
        # PhysicsEngine requires field size.
        # MatchController stores Game, if loaded.
        game = self.match.game
        if game is None:
             raise ValueError("Game not loaded")

        return self.physics_engine.prepare_traversal_space(
            robot_name,
            self.robots[robot_name],
            self.match.get_obstacles(),
            game.get_field_size(),
        )

    def __pathfind(
        self, robot_name: str, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> pathfinding.Path:
        robot_state: RobotState = self.match.game_state.get("robots").get(robot_name)
        traversal_space = self.prepare_traversal_space(robot_name)
        
        return self.physics_engine.pathfind(
            robot_name,
            robot_state.x.get(),
            robot_state.y.get(),
            x,
            y,
            traversal_space
        )

    def __trajectory(
        self,
        robot_name: str,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        heading: AngularMeasurement,
        path: pathfinding.Path,
        no_safety_corridor=False,
        start_time: float = 0.0,
    ) -> SwerveTrajectory:
        robot_state: RobotState = self.match.game_state.get("robots").get(robot_name)
        traversal_space = self.prepare_traversal_space(robot_name)
        
        start_state = (robot_state.x.get(), robot_state.y.get(), robot_state.heading.get())
        target_state = (x, y, heading)
        
        return self.physics_engine.generate_trajectory(
            robot_name,
            self.robots[robot_name],
            start_state,
            target_state,
            path,
            traversal_space,
            no_safety_corridor,
            start_time=start_time,
            avoid_other_robots=True,
        )

    def get_trajectories(self, robot_name) -> Dict:
        return self.physics_engine.trajectories.get(robot_name, {})

    def get_latest_trajectory(self) -> SwerveTrajectory:
        return self.physics_engine.get_latest_trajectory()

    def get_traversal_space(self, robot_name) -> TraversalSpace:
        return self.physics_engine.robot_traversal_space.get(robot_name, None)

    def drive_and_process_action(
        self,
        interactable_name,
        interaction_name,
        robot_name,
        time_cutoff=None,
        no_safety_cooridor=False,
    ) -> Tuple[bool, "SwerveTrajectory"]:
        """Drive to interactable and process action. Returns (success, trajectory)."""
        drive_point = self.match.get_navigation_point(
            interactable_name, interaction_name, robot_name
        )

        if drive_point is None:
             raise ValueError("Interactable does not have a navigation point")

        trajectory = self.drive_robot(
            robot_name,
            drive_point[0],
            drive_point[1],
            drive_point[2],
            no_safety_cooridor,
        )
        result = self.match.process_action(
            interactable_name, interaction_name, robot_name, time_cutoff
        )
        return result, trajectory

        name = robot.name.replace(self.ROBOT_PREFIX, "")
        self.robots[name] = robot

    def init_robot(self, robot_name: str, robot_state: RobotState) -> None:
        """Initializes a robot with the given state.

        :param robot_name: The name of the robot.
        :type robot_name: str
        :param robot_state: The state of the robot.
        :type robot_state: :class:`RobotState`
        """
        if self.game_state is None:
            raise ValueError("Game state not initialized")
        self.game_state.get("robots").registerSpace(robot_name, robot_state)


class ContinuousGameServer(AbstractGameServer):
    """
    Continuous Environment Server.
    Updates robot physics smoothly over time.
    """
    def __init__(self, config: ServerConfig = ServerConfig()):
        super().__init__(config)
        self.physics_engine = PhysicsEngine(config.physics_config)
        # TODO: Add continuous physics state/integrator here if not in PhysicsEngine

    def update(self, dt: float) -> bool:
        # 1. Update Physics (Integrate velocities)
        # self.physics_step(dt) ?
        
        # 2. Update Match Time
        return super().update(dt)

    def drive_robot(self, robot_name: str, x: SpatialMeasurement, y: SpatialMeasurement, theta: AngularMeasurement, no_safety_corridor=False) -> None:
        # In continuous, this might set a setpoint for a controller?
        pass


# Alias GameServer to DiscreteGameServer for backward compatibility
GameServer = DiscreteGameServer
