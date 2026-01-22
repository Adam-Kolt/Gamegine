import numpy as np
from typing import Dict, Any, List

from gamegine.simulation.game import GameState
from gamegine.first.alliance import Alliance
from examples.Rebuilt.scoring import Fuel, Tower, Hub
from examples.Rebuilt.match_logic import MatchPeriod, get_match_period

class AdvancedRebuiltRewardFunction:
    """Advanced reward function for Rebuilt 2026 Strategy Model.

    Prioritizes:
    1. Winning (Score Differential)
    2. Ranking Points (Secondary objective)
    3. Efficient Scoring (Dense rewards)
    """

    def __init__(self):
        # Tracking state for delta calculations
        self.prev_fuel = {}           # robot_name -> fuel_count
        self.prev_score = {}          # alliance -> score
        self.prev_rp = {}             # alliance -> rp_count
        self.prev_climb_state = {}    # robot_name -> has_climbed

        # Reward Weights
        self.weights = {
            # Dense Actions
            "pickup_fuel": 0.2,
            "score_active": 1.0,
            "score_inactive": -0.2,   # Penalty for wasting fuel
            
            # Climbing
            "climb_auto": 2.0,        # High value for auto climb
            "climb_endgame_l1": 10.0,
            "climb_endgame_l2": 20.0,
            "climb_endgame_l3": 30.0,
            
            # Defense & Strategy
            "successful_defense": 0.1, # Per step while defending effectively
            
            # Team Objectives
            "score_differential": 0.01, # Per step reward for leading
            "win_bonus": 10.0,          # End of match win bonus
            "rp_bonus": 5.0,            # Per RP achieved
            
            # Penalties
            "idle_penalty": -0.001,     # Per step for doing nothing
            "suicide_climb": -10.0,     # Climbing too early (disabling self)
        }

    def reset(self):
        """Reset internal state at start of episode."""
        self.prev_fuel = {}
        self.prev_score = {Alliance.RED: 0, Alliance.BLUE: 0}
        self.prev_rp = {Alliance.RED: 0, Alliance.BLUE: 0}
        self.prev_climb_state = {}

    def __call__(self, game_state: GameState, robot_states: Dict[str, Any], action_valid: Dict[str, bool], action_names: Dict[str, str]) -> Dict[str, float]:
        rewards = {}
        current_time = game_state.current_time.get()
        match_period = get_match_period(current_time)
        
        # Get Team Scores
        red_score = game_state.red_score.get()
        blue_score = game_state.blue_score.get()
        
        # Get RPs
        try:
            rebuilt_space = game_state.get("rebuilt")
            red_rp = rebuilt_space.getValue("red_rp").get()
            blue_rp = rebuilt_space.getValue("blue_rp").get()
        except KeyError:
            red_rp = 0
            blue_rp = 0

        # Calculate Shared Team Rewards
        # 1. Score Differential (Continuous pressure)
        red_diff_reward = (red_score - blue_score) * self.weights["score_differential"]
        blue_diff_reward = (blue_score - red_score) * self.weights["score_differential"]
        
        # 2. RP Delta Reward
        red_rp_gain = (red_rp - self.prev_rp.get(Alliance.RED, 0)) * self.weights["rp_bonus"]
        blue_rp_gain = (blue_rp - self.prev_rp.get(Alliance.BLUE, 0)) * self.weights["rp_bonus"]
        
        # Update prev scores/RP for next step
        self.prev_rp[Alliance.RED] = red_rp
        self.prev_rp[Alliance.BLUE] = blue_rp
        
        # Win Bonus (End of Match only) - approximated by large diff reward at end
        # Could implement explicit check if current_time >= MATCH_END
        
        for name, state in robot_states.items():
            robot_reward = 0.0
            alliance = state.alliance
            
            # --- SHARED REWARDS ---
            if alliance == Alliance.RED:
                robot_reward += red_diff_reward + red_rp_gain
            else:
                robot_reward += blue_diff_reward + blue_rp_gain

            # --- INDIVIDUAL DENSE REWARDS ---
            
            # 1. Fuel Pickup
            fuel_dict = state.gamepieces.get()
            fuel_count = fuel_dict.get(Fuel, 0)
            prev_fuel_count = self.prev_fuel.get(name, 0) # Default to 0 if new
            
            # Handle inventory tracking initialization properly
            if name not in self.prev_fuel:
                 # Initial step, don't reward starting inventory
                 self.prev_fuel[name] = fuel_count
                 pass
            else:
                delta_fuel = fuel_count - self.prev_fuel[name]
                if delta_fuel > 0:
                    robot_reward += delta_fuel * self.weights["pickup_fuel"]
                self.prev_fuel[name] = fuel_count

            # 2. Scoring & Action Analysis
            current_action = action_names.get(name, "")
            
            if "score_fuel" in current_action:
                # Determine target hub
                hub_name = "Blue Hub" if "Blue" in current_action else "Red Hub"
                try:
                    hub = game_state.get("interactables")[hub_name]
                    is_active = hub.getValue("is_active").get()
                    
                    # Reward only if action successfully COMPLETED this step?
                    # The reward function is called every step. "score_fuel" action often takes time.
                    # We might need to check if score INCREASED to attribute the reward accurately,
                    # OR we settle for rewarding the *attempt* if valid.
                    # Better: Check score delta.
                    pass 
                except KeyError:
                    pass

            # Check individual score contribution via Score Delta
            # This is hard because multiple robots score. 
            # But we can approximate: if my alliance score went up and I was performing "score_fuel", I likely did it.
            # OR we rely on `match_logic` events if available? No, access is limited here.
            
            # BETTER APPROACH for Scoring:
            # We already have dense rewards from the `score_fuel` function in `scoring.py`? 
            # No, that modifies state. We need to catch it here.
            # Let's rely on inventory DROP + Score INCREASE correlation.
            
            # If inventory decreased AND score increased, likely a score.
            if name in self.prev_fuel and fuel_count < self.prev_fuel[name]:
                # Potential score or drop
                alliance_score = red_score if alliance == Alliance.RED else blue_score
                prev_alliance_score = self.prev_score.get(alliance, 0)
                
                if alliance_score > prev_alliance_score:
                    # Successful score!
                    # Was it active or inactive?
                    # Check our alliance hub state
                    hub_name = f"{alliance.name.title()} Hub"
                    try:
                        hub = game_state.get("interactables")[hub_name]
                        is_active = hub.getValue("is_active").get()
                        
                        fuel_scored = self.prev_fuel[name] - fuel_count
                        
                        if is_active:
                            robot_reward += fuel_scored * self.weights["score_active"]
                        else:
                            robot_reward += fuel_scored * self.weights["score_inactive"]
                    except KeyError:
                        pass

            # 3. Climbing
            # Check if climbed status changed
            # This requires access to Tower states or robot "gameover" flag
            # Tower state is cleaner
            tower_name = f"{alliance.name.title()} Tower"
            try:
                tower = game_state.get("interactables")[tower_name]
                has_climbed = tower.has_climbed(name)
                
                if has_climbed and not self.prev_climb_state.get(name, False):
                    # Just climbed!
                    # Check period
                    if match_period == MatchPeriod.AUTO:
                        robot_reward += self.weights["climb_auto"]
                    elif match_period == MatchPeriod.ENDGAME:
                        # Which level?
                        # Hard to know exactly which level from here without more logic, 
                        # but we can guess or assign generic "climb" reward + points reward (which is handled by score delta)
                        # Let's just give a fixed "Goal Completion" reward
                        robot_reward += self.weights["climb_endgame_l1"] # Base reward
                    else:
                        # Suicide climb (Teleop)
                        robot_reward += self.weights["suicide_climb"]
                
                self.prev_climb_state[name] = has_climbed
                
            except KeyError:
                pass


            # 4. Auto Foul Penalty (Pickup from Opponent Zone)
            if match_period == MatchPeriod.AUTO:
                if "pickup" in current_action and "Alliance Zone" in current_action:
                    # Check for cross-alliance interaction
                    target_is_blue_zone = "Blue Alliance Zone" in current_action
                    target_is_red_zone = "Red Alliance Zone" in current_action
                    
                    is_foul = False
                    if alliance == Alliance.RED and target_is_blue_zone:
                         is_foul = True
                    elif alliance == Alliance.BLUE and target_is_red_zone:
                         is_foul = True
                         
                    if is_foul:
                        robot_reward += self.weights.get("foul_opponent_zone", -5.0)

            # 5. Idle Penalty
            # If velocity is low and not interacting
            # (Requires accessing robot velocity from state, which might be in physics state not high level state)
            # RobotState has `physics_state` usually
            # Skipping for now to avoid complexity/crashes if attribute missing
            
            rewards[name] = robot_reward

        # Update global prev scores
        self.prev_score[Alliance.RED] = red_score
        self.prev_score[Alliance.BLUE] = blue_score

        return rewards
