from dataclasses import dataclass
from typing import Dict, List, Tuple, Callable

from gamegine.representation.game import Game
from gamegine.representation.interactable import RobotInteractable, RobotInteractionConfig
from gamegine.representation.robot import SwerveRobot
from gamegine.representation import obstacle
from gamegine.simulation.game import GameState
from gamegine.simulation.state import ValueChange, ValueIncrease
from gamegine.simulation.rules import RuleEngine
from gamegine.utils.logging import Info, Warn
from gamegine.simulation.logic import LogicRule

@dataclass
class MatchConfig:
    pass

class MatchController:
    """
    Manages the unified game state, rules, and scoring.
    Decoupled from specific movement implementations (Discrete/Continuous).
    """

    def __init__(self, config: MatchConfig = MatchConfig()):
        self.config = config
        self.game_state: GameState = None
        self.game = None
        
        # State tracking
        self.robots: Dict[str, SwerveRobot] = {}
        self.interactables: Dict[str, RobotInteractable] = {}
        self.field_obstacles = []
        
        # Logs
        self.game_log = []
        
        # Rule Engine
        self.rule_engine = RuleEngine(self.log_cb)
        
        # Logic Rules
        self.logic_rules: List[LogicRule] = []

    def log_cb(self, action, changes):
        """Callback for RuleEngine logging."""
        if self.game_state:
             self.game_log.append((self.game_state.current_time.get(), action, changes))
        else:
             self.game_log.append((0, action, changes))

    def log(self, action, changes):
        self.log_cb(action, changes)
        
    def clear_log(self):
        self.game_log = []
        
    def get_log(self):
        return self.game_log

    def load_game(self, game: Game) -> None:
        """Loads the game representation and initializes state."""
        self.game = game
        
        # Initialize default game state locally
        initial_state = GameState()
        initial_state.auto_time.set(15)
        initial_state.teleop_time.set(135)
        initial_state.endgame_time.set(30)
        initial_state.current_time.set(0)
        initial_state.score.set(0)
        
        self.game_state = initial_state
        self.game_state.createSpace("interactables")
        self.game_state.createSpace("robots")

        self.set_obstacles(game.get_obstacles())
        self.clear_log()

        for interactable in game.get_interactables():
            self.add_interactable(interactable)

    def set_obstacles(self, obstacles: List[obstacle.Obstacle]) -> None:
        self.field_obstacles = obstacles

    def get_obstacles(self) -> List[obstacle.Obstacle]:
        return self.field_obstacles

    def add_interactable(self, interactable: RobotInteractable) -> None:
        if self.game_state is None:
            raise ValueError("Game state not initialized")
        self.interactables[interactable.name] = interactable
        self.rule_engine.add_interactable(interactable, self.game_state)

    def add_robot(self, name: str, robot: SwerveRobot):
        self.robots[name] = robot
        # Could also register robot in game_state via RuleEngine or directly?
        # GameServer didn't explicitly call RuleEngine for robot addition, it just managed dict.
        # But RobotState needs to be in GameState.
        # GameServer assumed something else created RobotState? Or did we miss it?
        # In GameServer, `load_from_game` init `robots` space. `drive_robot` uses `game_state.get("robots")`.
        # Who populates `game_state.get("robots")`? 
        # Ah, `gamegine.simulation.robot.RobotState` is usually created when added?
        # Let's ensure access.
        if "robots" not in self.game_state.spaces:
             self.game_state.createSpace("robots")

    def is_game_over(self) -> bool:
        if self.game_state is None:
            return False
        return (
            self.game_state.current_time.get()
            >= self.game_state.teleop_time.get() + self.game_state.auto_time.get()
        )

    def update_time(self, dt: float) -> bool:
        """Updates the game time."""
        if self.game_state is None:
            raise ValueError("Game state not initialized")
        self.game_state.current_time.set(self.game_state.current_time.get() + dt)
        
        # Removed extensive logging for every tick to reduce noise?
        # self.log("Time Update", [ValueIncrease(self.game_state.current_time, dt)])
        # GameServer did log it.
        
        self._process_logic_rules(dt)
        
        return self.is_game_over()

    def process_action(
        self, interactable_name, interaction_name, robot_name, time_cutoff=None
    ) -> bool:
        return self.rule_engine.process_action(
            interactable_name,
            interaction_name,
            robot_name,
            self.robots,
            self.game_state,
            time_cutoff
        )

    def pickup_gamepiece(self, robot_name: str, gamepiece) -> None:
        self.rule_engine.pickup_gamepiece(robot_name, gamepiece, self.game_state)
        
    def get_actions_set(self, robot_name: str) -> List[Tuple[str, str]]:
        return self.rule_engine.get_actions_set(self.robots, robot_name)
    
    def apply_changes(self, changes: List[ValueChange]):
        self.rule_engine.process_value_changes(changes)

    def get_navigation_point(self, interactable_name, interaction_name, robot_name):
        return self.rule_engine.get_navigation_point(
            interactable_name, interaction_name, robot_name, self.robots
        )
        
    # --- Logic System Integration ---
    def add_logic_rule(self, rule: "LogicRule"):
        """Adds a logic rule to the match."""
        # Avoid circular import issues by importing locally if needed, 
        # but type hint uses string forward ref or we import at top.
        # We'll just append to a list.
        if not hasattr(self, "logic_rules"):
             self.logic_rules = []
        self.logic_rules.append(rule)

    def _process_logic_rules(self, dt: float):
        if not hasattr(self, "logic_rules"):
            return

        all_changes = []
        for rule in self.logic_rules:
            changes = rule.update(dt, self.game_state)
            if changes:
                all_changes.extend(changes)
        
        if all_changes:
            self.apply_changes(all_changes)
            self.log("Logic Rule Triggered", all_changes)

