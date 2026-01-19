
import sys
import os
sys.path.insert(0, os.path.abspath("examples/ReefScapeGamegine"))

import unittest
import numpy as np
from gamegine.representation.capabilities import RobotCapabilities
from gamegine.rl.envs.alliance_env import AllianceEnv
from gamegine.rl.config import EnvConfig, RobotConfig, TrainingConfig
from gamegine.first.alliance import Alliance
from gamegine.simulation.robot import RobotState
from examples.ReefScapeGamegine.Reefscape import Reefscape
from examples.Rebuilt.Rebuilt import create_rebuilt_game # Use Rebuilt for Game Obs
from examples.ReefScapeGamegine.ai_robot import SWERVE_ROBOT, init_robot_interaction

init_robot_interaction()

class TestCapabilitiesAndGameObs(unittest.TestCase):
    def test_randomization_and_game_obs(self):
        # Setup config
        robots = [
            RobotConfig(SWERVE_ROBOT, RobotState(alliance=Alliance.RED), name="Red1", team="red")
        ]
        
        config = EnvConfig(
            mode="discrete",
            robots=robots,
            training=TrainingConfig(
                mode="solo",
                use_capability_context=True,
                randomize_capabilities=True,
                capability_ranges={"max_speed": (10.0, 20.0)}
            ),
            fast_mode=True
        )
        
        # Use create_rebuilt_game which returns RebuiltGame instance
        # Note: We need to ensure Env creation uses this
        # AllianceEnv accepts a Game class or factory. Reefscape is a class.
        # We need to monkeypatch or modify for test to use RebuiltGame logic with existing Env flow
        # OR: Just pass create_rebuilt_game if allowed
        
        # Actually AllianceEnv.__init__ takes `game_module_or_class`. 
        # Usually we pass 'Reefscape' module which has 'create_game' function?
        # Looking at AllianceEnv code: "if hasattr(game, 'create_game'): self.game = game.create_game()"
        
        # Create game instance
        game = create_rebuilt_game()
        
        # Create env
        env = AllianceEnv(game, config=config)
        
        # Reset
        obs, _ = env.reset()
        
        # Check robot
        robot = env.server.robots["Red1"]
        self.assertIsNotNone(robot.capabilities)
        self.assertTrue(10.0 <= float(robot.capabilities.max_speed) <= 20.0)
        
        # Check observation size
        base_dim = 10
        cap_dim = RobotCapabilities.vector_size()
        game_dim = 5  # RebuiltGame observation size
        
        print(f"Obs Shape: {obs['red_0'].shape[0]}")
        print(f"Expected: {base_dim + cap_dim + game_dim}")
        
        self.assertEqual(obs["red_0"].shape[0], base_dim + cap_dim + game_dim)
        
        # Verify capability value (speed at index 10)
        expected_norm_speed = float(robot.capabilities.max_speed) / 10.0
        self.assertAlmostEqual(obs["red_0"][10], expected_norm_speed, places=4)
        
        # Verify game obs (last 5 should be roughly 0.0 or normalized values)
        # Neutral zone starts with 360 fuel -> 360/360 = 1.0 (assuming config matches)
        # Check index: 10 + 24 = 34. Game obs are 34, 35, 36, 37, 38
        
        # Specifically check Neutral Zone (index 34)
        # Should be 1.0 since it starts full
        # RebuiltMatchConfig.NEUTRAL_ZONE_FUEL = 360
        # RebuiltGame.get_observation -> nz_fuel = 360/360
        
        print(f"Game Obs (NZ): {obs['red_0'][34]}")
        # Note: create_rebuilt_game adds NeutralZone with initial_fuel=0 in code above?
        # Re-checked code: "initial_fuel=0 # Starts empty, fills from Hub scores"
        # Wait, implementation of Rebuilt.py says:
        # "neutral_zone = NeutralZone(..., initial_fuel=0, ...)"
        # So it should be 0.0
        
        self.assertAlmostEqual(obs["red_0"][34], 0.0, places=4)
        
        # Depots start with 24 -> 24/24 = 1.0
        # Blue Depot is index 35
        self.assertAlmostEqual(obs["red_0"][35], 1.0, places=4)

        print("Test passed!")
        env.close()

if __name__ == "__main__":
    unittest.main()
