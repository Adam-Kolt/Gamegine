from dataclasses import dataclass, field
import math
from typing import Dict, List, Tuple
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
)
from gamegine.simulation.rules import RuleEngine
from gamegine.utils.logging import Info, Warn, Debug


@dataclass
class ServerConfig:
    mesh_resolution: SpatialMeasurement = Feet(2)
    discretization_quality: int = 4


@dataclass
class TraversalSpace:
    traversal_map: Map
    obstacles: List[obstacle.Obstacle] = field(default_factory=list)


class GameServer:
    """A class representing a server for a game simulation. Serves as the base container for all game logic and state variables."""

    ROBOT_PREFIX = "Robot "

    def __init__(self) -> None:
        self.game = None
        self.game_state: GameState = None
        self.robots: Dict[str, SwerveRobot] = {}
        self.config = ServerConfig()
        self.field_obstacles = []
        self.game_log = []
        self.physics_engine = PhysicsEngine()
        self.rule_engine = RuleEngine(self.log_cb)

    def log_cb(self, action, changes):
        """Callback for RuleEngine logging."""
        # Check if game_state is initialized to avoid errors during early setup?
        if self.game_state:
             self.game_log.append((self.game_state.current_time.get(), action, changes))
        else:
             self.game_log.append((0, action, changes))
             
    def clear_log(self):
        self.game_log = []

    def log(self, action, changes):
        self.log_cb(action, changes)

    def __is_game_over(self) -> bool:
        """Determines whether the game is over.

        :return: True if the game is over, False otherwise.
        :rtype: bool
        """
        return (
            self.game_state.current_time.get()
            >= self.game_state.teleop_time.get() + self.game_state.auto_time.get()
        )

    def update(self, dt: int) -> bool:
        """Updates the game state based on the given time step.

        :param dt: The time step to update the game state by.
        :type dt: float
        """
        if self.game_state is None:
            raise ValueError("Game state not initialized")
        self.game_state.current_time.set(self.game_state.current_time.get() + dt)
        self.log("Time Update", [ValueIncrease(self.game_state.current_time, dt)])

        return self.__is_game_over()

    def load_from_game(self, game: Game) -> None:
        """Loads the game representation into the server.

        :param game: The game to be simulated.
        :type game: :class:`Game`
        """
        self.game = game
        
        # Initialize default game state locally
        initial_state = GameState()
        initial_state.auto_time.set(15)
        initial_state.teleop_time.set(135)
        initial_state.endgame_time.set(30)
        initial_state.current_time.set(0)
        initial_state.score.set(0)
        
        self.init_game_state(initial_state)
        self.set_obstacles(game.get_obstacles())
        self.clear_log()

        for interactable in game.get_interactables():
            self.add_interactable(interactable)

    def get_obstacles(self) -> List[obstacle.Obstacle]:
        """Returns the obstacles of the game server.

        :return: The obstacles of the game server.
        :rtype: List[:class:`Obstacle`]
        """
        return self.field_obstacles

    def set_obstacles(self, obstacles: List[obstacle.Obstacle]) -> None:
        """Sets the obstacles of the game server.

        :param obstacles: The obstacles to set.
        :type obstacles: List[:class:`Obstacle`]
        """
        self.field_obstacles = obstacles

    def add_interactable(self, interactable: RobotInteractable) -> None:
        """Adds an interactable object to the game server.

        :param interactable: The interactable object to add.
        :type interactable: :class:`Interactable`
        """
        if self.game_state is None:
            raise ValueError("Game state not initialized")
        self.rule_engine.add_interactable(interactable, self.game_state)

    def init_game_state(self, state: GameState) -> None:
        """Initializes the game state with the given state space.

        :param state: The state space to initialize the game state with.
        :type state: :class:`StateSpace`
        """
        self.game_state = state

        self.game_state.createSpace("interactables")
        self.game_state.createSpace("robots")

    def get_actions_set(self, robot_name: str) -> List[Tuple[str, str]]:
        return self.rule_engine.get_actions_set(self.robots, robot_name)

    def drive_robot(
        self,
        robot_name: str,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        theta: float,
        no_safety_corridor=False,
    ) -> None:
        """Drives a robot to a new position.

        :param robot_name: The name of the robot.
        :type robot_name: str
        :param x: The x-coordinate of the new position.
        :type x: float
        :param y: The y-coordinate of the new position.
        :type y: float
        :param theta: The angle of the robot.
        :type theta: float
        """
        if robot_name not in self.robots:
            raise ValueError("Robot not added")

        robot = self.robots[robot_name]
        robot_state: RobotState = self.game_state.get("robots").get(robot_name)

        if (x, y) == (robot_state.x.get(), robot_state.y.get()):
            Debug(f"Robot {robot_name} is already at ({x}, {y}), no need to move")
            return

        path = self.__pathfind(robot_name, x, y)
        trajectory = self.__trajectory(
            robot_name, x, y, Radian(theta), path, no_safety_corridor
        )

        changes = [
            ValueChange(robot_state.x, x),
            ValueChange(robot_state.y, y),
            ValueChange(robot_state.heading, theta),
            ValueIncrease(
                self.game_state.current_time,
                trajectory.get_travel_time().to(Second),
            ),
        ]

        self.process_value_changes(changes)

        Info(
            f"Robot {robot_name} moved to ({x}, {y}, {theta}), taking {trajectory.get_travel_time()} seconds, resulting in {changes}"
        )
        self.log(
            f"Robot {robot_name} moved to ({x}, {y}, {theta}), taking {trajectory.get_travel_time()} seconds",
            changes,
        )

    def prepare_traversal_space(self, robot_name: str) -> TraversalSpace:
        """Generates the expanded obstacles for the game."""
        if robot_name not in self.robots:
            raise ValueError("Robot not added")

        if self.game is None:
            raise ValueError("Game not loaded")

        return self.physics_engine.prepare_traversal_space(
            robot_name,
            self.robots[robot_name],
            self.game.get_obstacles() if self.game else self.get_obstacles(), # Fallback if game not set but obstacles are?
            self.game.get_field_size(),
        )

    def __pathfind(
        self, robot_name: str, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> pathfinding.Path:
        """Pathfinds a robot to a new position."""
        robot_state: RobotState = self.game_state.get("robots").get(robot_name)
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
    ) -> SwerveTrajectory:
        robot_state: RobotState = self.game_state.get("robots").get(robot_name)
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
            no_safety_corridor
        )

    def get_trajectories(self, robot_name) -> Dict:
        return self.physics_engine.trajectories.get(robot_name, {})

    def pickup_gamepiece(self, robot_name: str, gamepiece) -> None:
        self.rule_engine.pickup_gamepiece(robot_name, gamepiece, self.game_state)

    def get_traversal_space(self, robot_name) -> TraversalSpace:
        return self.physics_engine.robot_traversal_space.get(robot_name, None)

    def process_action(
        self, interactable_name, interaction_name, robot_name, time_cutoff=None
    ) -> bool:
        """Processes an action by a robot on an interactable object."""
        return self.rule_engine.process_action(
            interactable_name,
            interaction_name,
            robot_name,
            self.robots,
            self.game_state,
            time_cutoff
        )

    def process_value_changes(self, changes: List[ValueChange]) -> None:
        """Processes a value change."""
        self.rule_engine.process_value_changes(changes)

    def get_latest_trajectory(self) -> SwerveTrajectory:
        return self.physics_engine.get_latest_trajectory()

    def get_log(self):
        return self.game_log

    def drive_and_process_action(
        self,
        interactable_name,
        interaction_name,
        robot_name,
        time_cutoff=None,
        no_safety_cooridor=False,
    ) -> bool:
        """Drives a robot to an interactable object and processes an action."""
        # Use RuleEngine helper to find navigation point
        drive_point = self.rule_engine.get_navigation_point(
            interactable_name, interaction_name, robot_name, self.robots
        )

        if drive_point is None:
             # Should be caught by get_navigation_point or interactable itself logic, 
             # but legacy code raised ValueError.
             # interactable.get_navigation_point() might return None?
             # Let's assume RuleEngine helper handles getting it from config or interactable.
             # If it's still none, legacy code raised error.
             raise ValueError("Interactable does not have a navigation point")

        self.drive_robot(
            robot_name,
            drive_point[0],
            drive_point[1],
            drive_point[2],
            no_safety_cooridor,
        )
        return self.process_action(
            interactable_name, interaction_name, robot_name, time_cutoff
        )

    def add_robot(self, robot: SwerveRobot) -> None:
        """Adds a robot to the game server.

        :param robot: The robot to add.
        :type robot: :class:`SwerveRobot`
        """
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
