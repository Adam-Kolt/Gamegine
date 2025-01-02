from typing import Dict, List
from gamegine.representation.game import Game
from gamegine.representation.interactable import RobotInteractable
from gamegine.simulation.game import GameState
from gamegine.simulation.robot import RobotState
from gamegine.simulation.state import ValueChange


class GameServer:
    """A class representing a server for a game simulation. Serves as the base container for all game logic and state variables."""

    def __init__(self) -> None:
        self.game = None
        self.game_state = None
        self.interactables: Dict[str, RobotInteractable] = {}
        self.robots: Dict[str, RobotState] = {}

    def load_from_game(self, game: Game) -> None:
        """Loads the game representation into the server.

        :param game: The game to be simulated.
        :type game: :class:`Game`
        """
        self.game = game

    def add_interactable(self, interactable: RobotInteractable) -> None:
        """Adds an interactable object to the game server.

        :param interactable: The interactable object to add.
        :type interactable: :class:`Interactable`
        """
        if self.game_state is None:
            raise ValueError("Game state not initialized")
        self.interactables[interactable.identifier] = interactable
        self.game_state.get("interactables").registerSpace(
            interactable.identifier, interactable.initializeInteractableState()
        )

    def add_robot(self, name, robot_state) -> None:
        """Adds a robot object to the game server.

        :param robot: The robot object to add.
        :type robot: :class:`Robot`
        """
        if self.game_state is None:
            raise ValueError("Game state not initialized")

        self.game_state.get("robots").registerSpace(name, robot_state)

    def init_game_state(self, state: GameState) -> None:
        """Initializes the game state with the given state space.

        :param state: The state space to initialize the game state with.
        :type state: :class:`StateSpace`
        """
        self.game_state = state

        self.game_state.createSpace("interactables")
        self.game_state.createSpace("robots")

    def process_action(self, interactable_name, interaction_name, robot_name) -> None:
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
        changes = interaction.action(
            self.game_state.get("interactables").get(interactable_name),
            self.game_state.get("robots").get(robot_name),
            self.game_state,
        )
        self.process_value_changes(changes)

    def process_value_changes(self, changes: List[ValueChange]) -> None:
        """Processes a value change.

        :param change: The value change to process.
        :type change: :class:`ValueChange`
        """
        for change in changes:
            change.apply()
