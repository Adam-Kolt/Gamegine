"""REBUILT 2026 Match Logic

Defines LogicRules for:
- Match period transitions (AUTO → TRANSITION → SHIFT1-4 → ENDGAME)
- Hub activation toggling based on AUTO results
- Ranking Points calculation (ENERGIZED, SUPERCHARGED, TRAVERSAL)
"""

from typing import List, Callable
from dataclasses import dataclass
from enum import Enum

from gamegine.simulation.logic import LogicRule, TriggerType, Condition
from gamegine.simulation.state import ValueChange, ValueIncrease
from gamegine.simulation.game import GameState
from gamegine.simulation.conditions import TimeGreaterThan
from gamegine.first.alliance import Alliance


# =============================================================================
# MATCH PERIODS
# =============================================================================

class MatchPeriod(Enum):
    """Match periods for REBUILT."""
    AUTO = "auto"
    TRANSITION = "transition"
    SHIFT_1 = "shift_1"
    SHIFT_2 = "shift_2"
    SHIFT_3 = "shift_3"
    SHIFT_4 = "shift_4"
    ENDGAME = "endgame"


@dataclass
class MatchTiming:
    """Match timing configuration for REBUILT.
    
    All times are cumulative from match start.
    """
    auto_end: float = 20.0           # 0:20 in AUTO -> 0:00
    transition_end: float = 30.0     # 2:20 -> 2:10 (10s)
    shift_1_end: float = 55.0        # 2:10 -> 1:45 (25s)
    shift_2_end: float = 80.0        # 1:45 -> 1:20 (25s)
    shift_3_end: float = 105.0       # 1:20 -> 0:55 (25s)
    shift_4_end: float = 130.0       # 0:55 -> 0:30 (25s)
    match_end: float = 160.0         # 0:30 -> 0:00 (30s ENDGAME)


DEFAULT_TIMING = MatchTiming()


def get_match_period(current_time: float, timing: MatchTiming = DEFAULT_TIMING) -> MatchPeriod:
    """Determine the current match period based on time."""
    if current_time <= timing.auto_end:
        return MatchPeriod.AUTO
    elif current_time <= timing.transition_end:
        return MatchPeriod.TRANSITION
    elif current_time <= timing.shift_1_end:
        return MatchPeriod.SHIFT_1
    elif current_time <= timing.shift_2_end:
        return MatchPeriod.SHIFT_2
    elif current_time <= timing.shift_3_end:
        return MatchPeriod.SHIFT_3
    elif current_time <= timing.shift_4_end:
        return MatchPeriod.SHIFT_4
    else:
        return MatchPeriod.ENDGAME


# =============================================================================
# REBUILT MATCH STATE
# =============================================================================

class RebuiltMatchState:
    """Extended state for REBUILT-specific match tracking.
    
    Should be added to GameState as a subspace.
    """
    
    def __init__(self, game_state: GameState):
        """Initialize REBUILT match state within the game state."""
        self.game_state = game_state
        
        # Create rebuilt-specific space
        rebuilt_space = game_state.createSpace("rebuilt")
        
        # Track current period
        rebuilt_space.setValue("current_period", MatchPeriod.AUTO.value)
        
        # Track which alliance won AUTO (for Hub activation order)
        # None = not determined, "red" or "blue"
        rebuilt_space.setValue("auto_winner", None)
        
        # Track ranking points
        rebuilt_space.setValue("red_rp", 0)
        rebuilt_space.setValue("blue_rp", 0)
        
        # Track if RPs have been awarded (one-time)
        rebuilt_space.setValue("energized_rp_awarded_red", False)
        rebuilt_space.setValue("energized_rp_awarded_blue", False)
        rebuilt_space.setValue("supercharged_rp_awarded_red", False)
        rebuilt_space.setValue("supercharged_rp_awarded_blue", False)
        rebuilt_space.setValue("traversal_rp_awarded_red", False)
        rebuilt_space.setValue("traversal_rp_awarded_blue", False)
    
    @staticmethod
    def get_rebuilt_space(game_state: GameState):
        """Get the rebuilt-specific space from game state."""
        return game_state.get("rebuilt")


# =============================================================================
# HUB ACTIVATION LOGIC
# =============================================================================

def determine_auto_winner(game_state: GameState) -> str:
    """Determine which alliance scored more FUEL in AUTO.
    
    Returns "red", "blue", or randomly chosen if tied.
    """
    # Get Hub states to count AUTO fuel
    try:
        interactables = game_state.get("interactables")
        
        red_hub = interactables.get("Red Hub")
        blue_hub = interactables.get("Blue Hub")
        
        red_fuel = (
            red_hub.getValue("fuel_scored_active").get() + 
            red_hub.getValue("fuel_scored_inactive").get()
        )
        blue_fuel = (
            blue_hub.getValue("fuel_scored_active").get() + 
            blue_hub.getValue("fuel_scored_inactive").get()
        )
        
        if red_fuel > blue_fuel:
            return "red"
        elif blue_fuel > red_fuel:
            return "blue"
        else:
            # Tie: random selection (for simulation, default to red)
            import random
            return random.choice(["red", "blue"])
    except KeyError:
        # Hubs not registered yet, default to red
        return "red"


def get_hub_active_states(period: MatchPeriod, auto_winner: str) -> tuple:
    """Get (red_active, blue_active) based on period and AUTO winner.
    
    Rules:
    - AUTO, TRANSITION, ENDGAME: Both active
    - SHIFT 1: Winner's Hub INACTIVE, loser's Hub ACTIVE
    - SHIFT 2: Flip from SHIFT 1
    - SHIFT 3: Same as SHIFT 1
    - SHIFT 4: Same as SHIFT 2
    """
    both_active = (True, True)
    
    if period in [MatchPeriod.AUTO, MatchPeriod.TRANSITION, MatchPeriod.ENDGAME]:
        return both_active
    
    # Determine base state for SHIFT 1
    # Winner has INACTIVE Hub in SHIFT 1
    if auto_winner == "red":
        shift_1_state = (False, True)  # Red inactive, Blue active
    else:
        shift_1_state = (True, False)  # Red active, Blue inactive
    
    shift_2_state = (not shift_1_state[0], not shift_1_state[1])
    
    if period == MatchPeriod.SHIFT_1:
        return shift_1_state
    elif period == MatchPeriod.SHIFT_2:
        return shift_2_state
    elif period == MatchPeriod.SHIFT_3:
        return shift_1_state
    elif period == MatchPeriod.SHIFT_4:
        return shift_2_state
    
    return both_active


def update_hub_states(game_state: GameState, red_active: bool, blue_active: bool):
    """Update Hub active states in game state."""
    try:
        interactables = game_state.get("interactables")
        interactables.get("Red Hub").setValue("is_active", red_active)
        interactables.get("Blue Hub").setValue("is_active", blue_active)
    except KeyError:
        pass  # Hubs not registered


# =============================================================================
# RANKING POINT THRESHOLDS
# =============================================================================

@dataclass
class RPThresholds:
    """Ranking point thresholds for REBUILT."""
    energized: int = 100       # FUEL scored
    supercharged: int = 360    # FUEL scored
    traversal: int = 50        # TOWER points


DEFAULT_RP_THRESHOLDS = RPThresholds()


def check_energized_rp(game_state: GameState, alliance: Alliance, thresholds: RPThresholds = DEFAULT_RP_THRESHOLDS) -> bool:
    """Check if alliance qualifies for ENERGIZED RP."""
    try:
        hub_name = f"{alliance.name.title()} Hub"
        hub_state = game_state.get("interactables").get(hub_name)
        total_fuel = (
            hub_state.getValue("fuel_scored_active").get() + 
            hub_state.getValue("fuel_scored_inactive").get()
        )
        return total_fuel >= thresholds.energized
    except KeyError:
        return False


def check_supercharged_rp(game_state: GameState, alliance: Alliance, thresholds: RPThresholds = DEFAULT_RP_THRESHOLDS) -> bool:
    """Check if alliance qualifies for SUPERCHARGED RP."""
    try:
        hub_name = f"{alliance.name.title()} Hub"
        hub_state = game_state.get("interactables").get(hub_name)
        total_fuel = (
            hub_state.getValue("fuel_scored_active").get() + 
            hub_state.getValue("fuel_scored_inactive").get()
        )
        return total_fuel >= thresholds.supercharged
    except KeyError:
        return False


def check_traversal_rp(game_state: GameState, alliance: Alliance, thresholds: RPThresholds = DEFAULT_RP_THRESHOLDS) -> bool:
    """Check if alliance qualifies for TRAVERSAL RP."""
    try:
        tower_name = f"{alliance.name.title()} Tower"
        tower_state = game_state.get("interactables").get(tower_name)
        tower_points = tower_state.getValue("tower_score").get()
        return tower_points >= thresholds.traversal
    except KeyError:
        return False


# =============================================================================
# LOGIC RULES FACTORY
# =============================================================================

def create_period_transition_rules(timing: MatchTiming = DEFAULT_TIMING) -> List[LogicRule]:
    """Create LogicRules for period transitions and Hub activation."""
    rules = []
    
    # Store the auto winner for use in later rules
    _auto_winner = {"value": None}
    
    # AUTO End - Determine winner
    def on_auto_end():
        def update(dt, gs):
            if _auto_winner["value"] is None:
                _auto_winner["value"] = determine_auto_winner(gs)
                try:
                    gs.get("rebuilt").setValue("auto_winner", _auto_winner["value"])
                    gs.get("rebuilt").setValue("current_period", MatchPeriod.TRANSITION.value)
                except KeyError:
                    pass
            return []
        return update
    
    # Can't use LogicRule directly for complex state changes, 
    # so we provide a factory for rules that can be added to MatchController
    
    # Period transition conditions
    period_times = [
        (timing.auto_end, MatchPeriod.TRANSITION),
        (timing.transition_end, MatchPeriod.SHIFT_1),
        (timing.shift_1_end, MatchPeriod.SHIFT_2),
        (timing.shift_2_end, MatchPeriod.SHIFT_3),
        (timing.shift_3_end, MatchPeriod.SHIFT_4),
        (timing.shift_4_end, MatchPeriod.ENDGAME),
    ]
    
    for time_threshold, new_period in period_times:
        def make_transition_action(period: MatchPeriod, threshold: float):
            def action():
                # This is a placeholder - actual implementation needs game_state
                return []
            return action
        
        rules.append(LogicRule(
            name=f"transition_to_{new_period.value}",
            condition=TimeGreaterThan(time_threshold),
            trigger_type=TriggerType.ON_TRUE,
            action=make_transition_action(new_period, time_threshold),
        ))
    
    return rules


def create_rp_check_rules(thresholds: RPThresholds = DEFAULT_RP_THRESHOLDS) -> List[LogicRule]:
    """Create LogicRules for ranking point tracking.
    
    Note: These are evaluated at match end, but we track continuously.
    """
    # RP rules would be evaluated at final scoring
    # For continuous tracking, we'd need to integrate with MatchController
    return []


# =============================================================================
# MATCH CONTROLLER INTEGRATION
# =============================================================================

class RebuiltMatchController:
    """Helper class to manage REBUILT-specific match logic.
    
    Call update() each tick to check period transitions and update Hub states.
    """
    
    def __init__(self, game_state: GameState, timing: MatchTiming = DEFAULT_TIMING, thresholds: RPThresholds = DEFAULT_RP_THRESHOLDS):
        self.game_state = game_state
        self.timing = timing
        self.thresholds = thresholds
        
        # Initialize rebuilt state
        self.rebuilt_state = RebuiltMatchState(game_state)
        
        # Track last period to detect transitions
        self._last_period = MatchPeriod.AUTO
        self._auto_winner = None
    
    def update(self, current_time: float) -> List[str]:
        """Update match state based on current time.
        
        Returns list of events that occurred (for logging).
        """
        events = []
        
        current_period = get_match_period(current_time, self.timing)
        
        # Check for period transition
        if current_period != self._last_period:
            events.append(f"Period transition: {self._last_period.value} -> {current_period.value}")
            
            # On AUTO end, determine winner
            if self._last_period == MatchPeriod.AUTO and current_period == MatchPeriod.TRANSITION:
                self._auto_winner = determine_auto_winner(self.game_state)
                try:
                    self.game_state.get("rebuilt").setValue("auto_winner", self._auto_winner)
                except KeyError:
                    pass
                events.append(f"AUTO winner: {self._auto_winner}")
            
            # Update Hub states based on new period
            if self._auto_winner:
                red_active, blue_active = get_hub_active_states(current_period, self._auto_winner)
                update_hub_states(self.game_state, red_active, blue_active)
                events.append(f"Hub states: Red={red_active}, Blue={blue_active}")
            
            # Update period in state
            try:
                self.game_state.get("rebuilt").setValue("current_period", current_period.value)
            except KeyError:
                pass
            
            self._last_period = current_period
        
        # Check ranking points (continuously, but award once)
        self._check_award_rp(Alliance.RED, events)
        self._check_award_rp(Alliance.BLUE, events)
        
        return events
    
    def _check_award_rp(self, alliance: Alliance, events: List[str]):
        """Check and award ranking points for an alliance."""
        try:
            rebuilt_space = self.game_state.get("rebuilt")
            alliance_key = alliance.name.lower()
            
            # ENERGIZED RP
            energized_key = f"energized_rp_awarded_{alliance_key}"
            if not rebuilt_space.getValue(energized_key).get():
                if check_energized_rp(self.game_state, alliance, self.thresholds):
                    rebuilt_space.setValue(energized_key, True)
                    current_rp = rebuilt_space.getValue(f"{alliance_key}_rp").get()
                    rebuilt_space.setValue(f"{alliance_key}_rp", current_rp + 1)
                    events.append(f"{alliance.name} earned ENERGIZED RP")
            
            # SUPERCHARGED RP
            supercharged_key = f"supercharged_rp_awarded_{alliance_key}"
            if not rebuilt_space.getValue(supercharged_key).get():
                if check_supercharged_rp(self.game_state, alliance, self.thresholds):
                    rebuilt_space.setValue(supercharged_key, True)
                    current_rp = rebuilt_space.getValue(f"{alliance_key}_rp").get()
                    rebuilt_space.setValue(f"{alliance_key}_rp", current_rp + 1)
                    events.append(f"{alliance.name} earned SUPERCHARGED RP")
            
            # TRAVERSAL RP
            traversal_key = f"traversal_rp_awarded_{alliance_key}"
            if not rebuilt_space.getValue(traversal_key).get():
                if check_traversal_rp(self.game_state, alliance, self.thresholds):
                    rebuilt_space.setValue(traversal_key, True)
                    current_rp = rebuilt_space.getValue(f"{alliance_key}_rp").get()
                    rebuilt_space.setValue(f"{alliance_key}_rp", current_rp + 1)
                    events.append(f"{alliance.name} earned TRAVERSAL RP")
        except KeyError:
            pass
    
    def get_final_results(self) -> dict:
        """Get final match results including scores and RPs."""
        try:
            rebuilt_space = self.game_state.get("rebuilt")
            return {
                "red_score": self.game_state.red_score.get(),
                "blue_score": self.game_state.blue_score.get(),
                "red_rp": rebuilt_space.getValue("red_rp").get(),
                "blue_rp": rebuilt_space.getValue("blue_rp").get(),
                "auto_winner": rebuilt_space.getValue("auto_winner").get(),
            }
        except KeyError:
            return {}
