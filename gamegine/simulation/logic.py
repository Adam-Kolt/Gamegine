from enum import Enum
from typing import Callable, List, Union, Generator, Optional
from gamegine.simulation.state import ValueChange
from gamegine.simulation.game import GameState

# Type alias for a condition function
Condition = Callable[[GameState], bool]

class TriggerType(Enum):
    ON_TRUE = "onTrue"
    WHILE_TRUE = "whileTrue"

class LogicRule:
    """
    Represents a logic rule that triggers an action based on a condition.
    
    :param name: Unique name of the rule.
    :param condition: A function that takes GameState and returns True/False.
    :param trigger_type: How the rule fires (ON_TRUE or WHILE_TRUE).
    :param action: A ValueChange or a callable returning a list of ValueChanges.
    :param delay: Time in seconds the condition must be true before firing.
    :param interval: (For WHILE_TRUE) Minimum time between firings. currently unused but good for future.
    """
    def __init__(
        self,
        name: str,
        condition: Condition,
        trigger_type: TriggerType,
        action: Union[ValueChange, List[ValueChange], Callable[[], List[ValueChange]]],
        delay: float = 0.0,
        interval: float = 0.0
    ):
        self.name = name
        self.condition = condition
        self.trigger_type = trigger_type
        self.action = action
        self.delay = delay
        
        self.interval = interval if interval is not None else 0.0
        
        # Internal State
        self._timer: float = 0.0
        self._is_active: bool = False
        self._has_triggered: bool = False # For ON_TRUE latching
        self._interval_timer: float = self.interval # Trigger immediately when ready

    def update(self, dt: float, game_state: GameState) -> List[ValueChange]:
        """
        Updates the rule state and returns any triggered changes.
        """
        # 1. Evaluate Condition
        is_condition_met = self.condition(game_state)
        
        changes = []
        
        if is_condition_met:
            # Increment timer if condition is met
            self._timer += dt
            
            # Check delay
            if self._timer >= self.delay:
                # Condition Met AND Delay Passed
                
                if self.trigger_type == TriggerType.ON_TRUE:
                    if not self._has_triggered:
                        changes = self._get_changes()
                        self._has_triggered = True
                        
                elif self.trigger_type == TriggerType.WHILE_TRUE:
                    # WhileTrue fires on interval while condition is met (after delay)
                    self._interval_timer += dt
                    if self._interval_timer >= self.interval:
                        changes = self._get_changes()
                        self._interval_timer = 0.0 # Reset interval timer
                    
        else:
            # Condition broken, reset state
            self._timer = 0.0
            self._has_triggered = False
            self._interval_timer = self.interval # Reset so it triggers immediately (or after 1 interval?) 
            # Usually "whileTrue" implies immediate trigger once delay passes, then every interval.
            # If I set strictly to 0, it waits one full interval.
            # If I set to `interval`, it triggers immediately.
            # Let's set to `interval` to ensure first trigger happens immediately upon delay completion.
            
        return changes

    def _get_changes(self) -> List[ValueChange]:
        if isinstance(self.action, list):
            return self.action
        elif isinstance(self.action, ValueChange):
            return [self.action]
        elif callable(self.action):
            res = self.action()
            if isinstance(res, list):
                return res
            return [res]
        return []
