from typing import Dict, List, Tuple, Callable, Any

from gamegine.representation.interactable import (
    RobotInteractable,
    RobotInteractionConfig,
)
from gamegine.simulation.game import GameState
from gamegine.simulation.state import ValueChange, ValueIncrease
from gamegine.representation.robot import SwerveRobot
from gamegine.utils.logging import Info, Warn

class RuleEngine:
    """Handles game rules, interactions, and state updates."""

    def __init__(self, logger: Callable[[str, List[ValueChange]], None]):
        """
        :param logger: Callback function to log actions and changes.
        """
        self.interactables: Dict[str, RobotInteractable] = {}
        self.logger = logger

    def add_interactable(self, interactable: RobotInteractable, game_state: GameState) -> None:
        """Adds an interactable object to the engine."""
        self.interactables[interactable.name] = interactable
        if "interactables" in game_state.spaces:
             game_state.get("interactables").registerSpace(
                interactable.name, interactable.initializeInteractableState()
            )
        else:
             # Fallback if space not created yet, though GameServer usually creates it
             pass

    def get_actions_set(self, robots: Dict[str, SwerveRobot], robot_name: str) -> List[Tuple[str, str]]:
        robot = robots[robot_name]
        configs: Dict[str, Dict[str, RobotInteractionConfig]] = (
            robot.interaction_configs
        )

        actions = []
        for interactable, interactions in configs.items():
            for interaction_str, interaction in interactions.items():
                actions.append(
                    (interaction.interactable_name, interaction.interaction_identifier)
                )
        return actions

    def pickup_gamepiece(self, robot_name: str, gamepiece: Any, game_state: GameState) -> None:
        robot_state = game_state.get("robots").get(robot_name)
        gamepiece_state = robot_state.gamepieces.get()

        if gamepiece not in gamepiece_state:
            gamepiece_state[gamepiece] = 0

        gamepiece_state[gamepiece] += 1

        self.logger(f"Robot {robot_name} picked up {gamepiece}", [])

    def process_value_changes(self, changes: List[ValueChange]) -> None:
        """Processes a value change."""
        for change in changes:
            change.apply()

    def _get_robot_action_config(
        self, robots: Dict[str, SwerveRobot], robot_name: str, interactable: str, interaction: str
    ) -> RobotInteractionConfig:
        """Gets the action configuration for a robot."""
        robot = robots[robot_name]
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

    def process_action(
        self,
        interactable_name: str,
        interaction_name: str,
        robot_name: str,
        robots: Dict[str, SwerveRobot],
        game_state: GameState,
        time_cutoff: float = None
    ) -> bool:
        """Processes an action by a robot on an interactable object."""
        interactable = self.interactables[interactable_name]
        
        # We need interaction object. Interactable has get_interaction
        interaction = interactable.get_interaction(interaction_name)

        robot_config = self._get_robot_action_config(
            robots, robot_name, interactable_name, interaction_name
        )

        if not interaction.ableToInteract(
            game_state.get("interactables").get(interactable_name),
            game_state.get("robots").get(robot_name),
            game_state,
        ):
            msg = f"Robot {robot_name} attempted to perform {interaction_name} on {interactable_name}, but was unable to"
            Info(msg)
            self.logger(msg, [])
            return False

        time = 0.0
        if robot_config is not None:
            time = robot_config.time_to_interact(
                game_state.get("interactables").get(interactable_name),
                game_state.get("robots").get(robot_name),
                game_state,
            )
            time_change = ValueIncrease(game_state.current_time, time)
            self.process_value_changes([time_change])
        
        if time_cutoff is not None:
            if game_state.current_time.get() > time_cutoff:
                msg = f"Robot {robot_name} attempted to perform {interaction_name} on {interactable_name}, but the time cutoff was reached"
                Info(msg)
                self.logger(msg, [])
                return False

        changes = interaction.action(
            game_state.get("interactables").get(interactable_name),
            game_state.get("robots").get(robot_name),
            game_state,
        )

        self.process_value_changes(changes)
        
        msg = f"Robot {robot_name} performed {interaction_name} on {interactable_name}, taking {time} seconds"
        Info(f"{msg}, resulting in {changes}")
        self.logger(msg, changes)

        return True

    def get_navigation_point(self, interactable_name: str, interaction_name: str, robot_name: str, robots: Dict[str, SwerveRobot]):
        """Helper to get navigation point for an interaction."""
        interactable = self.interactables[interactable_name]
        
        robot_config = self._get_robot_action_config(
             robots, robot_name, interactable_name, interaction_name
        )
        
        drive_point = None
        if robot_config is not None:
            drive_point = robot_config.navigation_point
        if drive_point is None:
            drive_point = interactable.get_navigation_point()
            
        return drive_point
