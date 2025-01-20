from dataclasses import dataclass
from typing import Callable, Dict, List, Tuple
from gamegine.representation.game import Game
from gamegine.representation.interactable import RobotInteractable, RobotInteractionConfig
from gamegine.representation.robot import SwerveRobot
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.simulation.state import StateSpace, ValueChange
from gamegine.utils.logging import Warn


@dataclass
class ServerReceipt:
    type: str
    changes: List[ValueChange]
    description: str
    time: float = 0


class GameServer2:
    def __init__(self, game: Game, discrete=True):
        self.discrete = discrete
        self.game = None
        self.game_state: GameState = None
        self.interactables: Dict[str, RobotInteractable] = {}
        self.robots: Dict[str, SwerveRobot] = {}
        self.game_log = []
        self.game = game
        self.end_condition = None
        self.step_size = 0.5

    def log(self, receipt: ServerReceipt):
        receipt.time = self.get_game_state().current_time.get()
        self.game_log.append(receipt)

    def init(self):
        self.init_interactables()
        self.init_game_state()

    def add_robot(self, robot: SwerveRobot):
        self.robots[robot.name] = robot
        self.get_game_state().add_robot(robot)

    def get_game_state(self):
        if self.game_state is None:
            self.init_game_state()
        return self.game_state

    def load_game(self, game: Game):
        self.game = game

    def init_interactables(self):
        for interactable in self.game.interactables:
            self.interactables[interactable.name] = interactable
            self.get_game_state().add_interactable(interactable)

    def init_game_state(self):
        self.game_state = self.game.get_initial_state()

    def is_game_over(self):
        return self.end_condition(self.game_state)

    def set_end_condition(self, end_condition: Callable[[GameState], bool]):
        self.end_condition = end_condition

    def update_state(
        self,
        changes: List[ValueChange],
        description: str = "",
        type: str = "Generic Update",
    ):
        for change in changes:
            change.apply()
        self.log(ServerReceipt(type, changes, description))

    def step(self) -> Tuple[List[ValueChange], bool]:
        if self.discrete:
            pass
        else:
            pass
        return ([], self.is_game_over())

    def clear_log(self):
        self.game_log = []

    def save_log(self, filename):
        with open(filename, "w") as f:
            for entry in self.game_log:
                f.write(f"{entry.time}: {entry.type} - {entry.description}\n")
                for change in entry.changes:
                    f.write(f"  {change}\n")
                f.write("\n")

    def get_log(self):
        return self.game_log

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

    def robot_do_action(self, interactable_name, interaction_name, robot_name) -> None:

        interactable = self.interactables[interactable_name]
        robot = self.robots[robot_name]
        interaction = interactable.get_interaction(interaction_name)

        robot_config = self.__get_robot_action_config(
            robot_name, interactable_name, interaction_name
        )
