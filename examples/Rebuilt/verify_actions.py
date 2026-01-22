import argparse
import logging
import os
import sys
import warnings
import multiprocessing

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

# ... existing imports ...
from gamegine.simulation.game import GameState
from gamegine.representation.game import Game
from gamegine.simulation.robot import RobotState
from gamegine.first.alliance import Alliance
from gamegine.utils.NCIM.ncim import Meter, Radian, Inch
from examples.Rebuilt.scoring import Hub, Tower, Depot, NeutralZone, Fuel
from examples.Rebuilt.match_logic import RebuiltMatchController, MatchPeriod
import unittest

class TestRobotActions(unittest.TestCase):
    def setUp(self):
        # Setup minimal game state
        self.game_state = GameState()
        self.game_state.setValue("current_time", 0.0)
        self.game_state.setValue("auto_time", 15.0) # Correctly set Auto Time
        
        # Setup Robots
        self.red_robot = RobotState(x=Meter(0), y=Meter(0), heading=Radian(0), alliance=Alliance.RED)
        self.red_robot.setValue("name", "Red1")
        self.blue_robot = RobotState(x=Meter(10), y=Meter(5), heading=Radian(0), alliance=Alliance.BLUE)
        self.blue_robot.setValue("name", "Blue1")
        
        # Setup Scoring Objects
        self.red_hub = Hub(center=(Meter(2), Meter(2)), navigation_point=(Meter(1), Meter(1), Radian(0)), alliance=Alliance.RED)
        self.blue_hub = Hub(center=(Meter(14), Meter(6)), navigation_point=(Meter(13), Meter(6), Radian(0)), alliance=Alliance.BLUE)
        
        self.red_tower = Tower(center=(Meter(1), Meter(4)), navigation_point=(Meter(2), Meter(4), Radian(0)), alliance=Alliance.RED)
        self.blue_tower = Tower(center=(Meter(15), Meter(4)), navigation_point=(Meter(14), Meter(4), Radian(0)), alliance=Alliance.BLUE)
        
        self.neutral_zone = NeutralZone(center=(Meter(8), Meter(4)), navigation_point=(Meter(8), Meter(3), Radian(0)), initial_fuel=10)
        
        # Register interactables in game state
        self.game_state.createSpace("interactables")
        self.interactables = self.game_state.get("interactables")
        
        # Register Hubs
        self.interactables.registerSpace("Red Hub", self.red_hub.initializeInteractableState())
        self.interactables.registerSpace("Blue Hub", self.blue_hub.initializeInteractableState())
        
        # Register Towers
        self.interactables.registerSpace("Red Tower", self.red_tower.initializeInteractableState())
        self.interactables.registerSpace("Blue Tower", self.blue_tower.initializeInteractableState())
        
        # Register Neutral Zone
        self.interactables.registerSpace("Neutral Zone", self.neutral_zone.initializeInteractableState())

    def test_climb_distance_constraint(self):
        print("\n--- Testing Climb Distance Constraint ---")
        # Move robot FAR AWAY
        self.red_robot.setValue("x", Meter(100)) # Far
        self.red_robot.setValue("y", Meter(100))
        
        interactions = self.red_tower.get_interactions()
        climb_action = next(i for i in interactions if i.identifier == "climb_level_1")
        
        # Condition should be FALSE
        can_climb = climb_action.condition(self.interactables.get("Red Tower"), self.red_robot, self.game_state)
        print(f"Can climb from far away? {can_climb}")
        self.assertFalse(can_climb, "Should NOT be able to climb from far away")
        
        # Move robot CLOSE
        # Tower at (1,4), Nav at (2,4)
        self.red_robot.setValue("x", Meter(2.0))
        self.red_robot.setValue("y", Meter(4.0))
        
        can_climb = climb_action.condition(self.interactables.get("Red Tower"), self.red_robot, self.game_state)
        print(f"Can climb from close? {can_climb}")
        self.assertTrue(can_climb, "Should be able to climb from close")

if __name__ == "__main__":
    unittest.main()
