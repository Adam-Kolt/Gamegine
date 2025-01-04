from dataclasses import dataclass, field
import math
from typing import Dict, List
from gamegine.analysis import pathfinding
from gamegine.analysis.meshing import Map, TriangulatedGraph
from gamegine.analysis.trajectory.lib.TrajGen import (
    SolverConfig,
    SwerveRobotConstraints,
    SwerveTrajectory,
    SwerveTrajectoryProblemBuilder,
    TrajectoryBuilderConfig,
    Waypoint,
)
from gamegine.analysis.trajectory.lib.constraints.avoidance import SafetyCorridor
from gamegine.analysis.trajectory.lib.constraints.constraints import (
    VelocityEquals,
    AngleEquals,
)
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
from gamegine.utils.NCIM.Dimensions.angular import AngularMeasurement
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
from gamegine.utils.logging import Info, Warn


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
        self.interactables: Dict[str, RobotInteractable] = {}
        self.robots: Dict[str, SwerveRobot] = {}
        self.config = ServerConfig()
        self.field_obstacles = []
        self.robot_traversal_space = {}
        self.trajectories = {}

    def __is_game_over(self) -> bool:
        """Determines whether the game is over.

        :return: True if the game is over, False otherwise.
        :rtype: bool
        """
        return (
            self.game_state.current_time.get()
            >= self.game_state.endgame_time.get()
            + self.game_state.teleop_time.get()
            + self.game_state.auto_time.get()
        )

    def update(self, dt: int) -> bool:
        """Updates the game state based on the given time step.

        :param dt: The time step to update the game state by.
        :type dt: float
        """
        if self.game_state is None:
            raise ValueError("Game state not initialized")
        self.game_state.current_time.set(self.game_state.current_time.get() + dt)

        return self.__is_game_over()

    def prepare_traversal_space(self, robot: str) -> TraversalSpace:
        """Generates the expanded obstacles for the game."""
        if robot in self.robot_traversal_space:
            return self.robot_traversal_space[robot]

        if robot not in self.robots:
            raise ValueError("Robot not added")

        if self.game is None:
            raise ValueError("Game not loaded")

        robot_object = self.robots[robot]
        obstacles = ExpandedObjectBounds(
            self.get_obstacles(),
            robot_object.get_bounding_radius(),
            self.config.discretization_quality,
        )

        map_obstacles = ExpandedObjectBounds(
            self.game.get_obstacles(),
            Inch(2) + robot_object.get_bounding_radius(),
            self.config.discretization_quality,
        )

        triangle_map = TriangulatedGraph(
            map_obstacles,
            self.config.mesh_resolution,
            self.game.get_field_size(),
            self.config.discretization_quality,
        )

        traversal_space = TraversalSpace(
            traversal_map=triangle_map, obstacles=obstacles
        )

        self.robot_traversal_space[robot] = traversal_space

        return traversal_space

    def load_from_game(self, game: Game) -> None:
        """Loads the game representation into the server.

        :param game: The game to be simulated.
        :type game: :class:`Game`
        """
        self.game = game
        self.init_game_state(game.get_initial_state())
        self.set_obstacles(game.get_obstacles())

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
        self.interactables[interactable.name] = interactable
        self.game_state.get("interactables").registerSpace(
            interactable.name, interactable.initializeInteractableState()
        )

    def init_game_state(self, state: GameState) -> None:
        """Initializes the game state with the given state space.

        :param state: The state space to initialize the game state with.
        :type state: :class:`StateSpace`
        """
        self.game_state = state

        self.game_state.createSpace("interactables")
        self.game_state.createSpace("robots")

    def __get_robot_action_config(
        self, robot_name: str, interactable: str, interaction: str
    ) -> RobotInteractionConfig:
        """Gets the action configuration for a robot.

        :param robot_name: The name of the robot.
        :type robot_name: str
        :return: The action configuration for the robot.
        :rtype: :class:`RobotInteractionConfig`
        """
        robot = self.robots[robot_name]
        if interactable not in robot.interaction_configs:
            Warn(
                f"Robot {robot_name} does not have an interaction config for {interactable}"
            )
            return None

        if interaction not in robot.interaction_configs[interactable]:
            Warn(
                f"Robot {robot_name} does not have an interaction config for {interactable} with {interaction}"
            )
            return None
        return robot.interaction_configs[interactable][interaction]

    def drive_robot(
        self,
        robot_name: str,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        theta: float,
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

        path = self.__pathfind(robot_name, x, y)
        trajectory = self.__trajectory(robot_name, x, y, theta, path)

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

    def __pathfind(
        self, robot_name: str, x: SpatialMeasurement, y: SpatialMeasurement
    ) -> pathfinding.Path:
        """Pathfinds a robot to a new position."""

        robot = self.robots[robot_name]
        robot_state: RobotState = self.game_state.get("robots").get(robot_name)
        traversal_space = self.prepare_traversal_space(robot_name)

        path = pathfinding.findPath(
            traversal_space.traversal_map,
            (robot_state.x.get(), robot_state.y.get()),
            (x, y),
            pathfinding.AStar,
            pathfinding.InitialConnectionPolicy.ConnectToClosest,
        )
        path.shortcut(traversal_space.obstacles)
        return path

    def __trajectory(
        self,
        robot_name: str,
        x: SpatialMeasurement,
        y: SpatialMeasurement,
        heading: AngularMeasurement,
        path: pathfinding.Path,
    ) -> SwerveTrajectory:
        robot = self.robots[robot_name]
        robot_state: RobotState = self.game_state.get("robots").get(robot_name)
        traversal_space = self.prepare_traversal_space(robot_name)
        start_x = robot_state.x.get()
        start_y = robot_state.y.get()
        start_heading = robot_state.heading.get()

        if (start_x, start_y, start_heading, x, y, heading) in self.trajectories.get(
            robot_name, {}
        ):
            return self.trajectories[robot_name][(start_x, start_y, x, y)]

        builder = SwerveTrajectoryProblemBuilder()
        builder.waypoint(
            Waypoint(start_x, start_y).given(
                VelocityEquals(MetersPerSecond(0), MetersPerSecond(0)),
                AngleEquals(start_heading),
            )
        )
        builder.waypoint(
            Waypoint(x, y).given(
                VelocityEquals(MetersPerSecond(0), MetersPerSecond(0)),
                AngleEquals(heading),
            )
        )
        builder.guide_pathes([path])
        builder.points_constraint(SafetyCorridor(traversal_space.obstacles))

        trajectory = builder.generate(
            TrajectoryBuilderConfig(
                trajectory_resolution=Centimeter(15),
                stretch_factor=1.5,
                min_spacing=Centimeter(5),
            )
        ).solve(
            SwerveRobotConstraints(
                MeterPerSecondSquared(5),
                MetersPerSecond(6),
                RadiansPerSecondSquared(3.14),
                RadiansPerSecond(3.14),
                robot.get_drivetrain(),
                physical_parameters=robot.get_physics(),
            ),
            SolverConfig(timeout=10, max_iterations=10000, solution_tolerance=1e-9),
        )

        if robot_name not in self.trajectories:
            self.trajectories[robot_name] = {}

        self.trajectories[robot_name][
            (start_x, start_y, start_heading, x, y)
        ] = trajectory

        return trajectory

    def get_trajectories(self, robot_name) -> Dict:
        return self.trajectories.get(robot_name, {})

    def pickup_gamepiece(self, robot_name: str, gamepiece) -> None:
        robot_state: RobotState = self.game_state.get("robots").get(robot_name)
        gamepiece_state = robot_state.gamepieces.get()

        if gamepiece not in gamepiece_state:
            gamepiece_state[gamepiece] = 1

        gamepiece_state[gamepiece] += 1

    def get_traversal_space(self, robot_name) -> TraversalSpace:
        return self.robot_traversal_space.get(robot_name, None)

    def process_action(
        self, interactable_name, interaction_name, robot_name, time_cutoff=None
    ) -> None:
        """Processes an action by a robot on an interactable object.

        :param interactable_name: The name of the interactable object.
        :type interactable_name: str
        :param interaction_name: The name of the interaction.
        :type interaction_name: str
        :param robot_name: The name of the robot performing the interaction.
        :type robot_name: str
        """
        interactable = self.interactables[interactable_name]
        robot = self.robots[robot_name]
        interaction = interactable.get_interaction(interaction_name)

        robot_config = self.__get_robot_action_config(
            robot_name, interactable_name, interaction_name
        )

        if robot_config is not None:
            time = robot_config.time_to_interact(
                self.game_state.get("interactables").get(interactable_name),
                self.game_state.get("robots").get(robot_name),
                self.game_state,
            )
            time_change = ValueIncrease(self.game_state.current_time, time)
            self.process_value_changes([time_change])

        if self.game_state.current_time.get() > time_cutoff:
            Info(
                f"Robot {robot_name} attempted to perform {interaction_name} on {interactable_name}, but the time cutoff was reached"
            )
            return

        changes = interaction.action(
            self.game_state.get("interactables").get(interactable_name),
            self.game_state.get("robots").get(robot_name),
            self.game_state,
        )

        self.process_value_changes(changes)
        Info(
            f"Robot {robot_name} performed {interaction_name} on {interactable_name}, taking {time} seconds, resulting in {changes}"
        )

    def process_value_changes(self, changes: List[ValueChange]) -> None:
        """Processes a value change.

        :param change: The value change to process.
        :type change: :class:`ValueChange`
        """
        for change in changes:
            change.apply()

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
