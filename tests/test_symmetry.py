
import unittest
import numpy as np
from gamegine.rl import make_alliance_env
from examples.Rebuilt.Rebuilt import create_rebuilt_game, FIELD_LENGTH, FIELD_WIDTH
from examples.Rebuilt.robot import create_robot, setup_robot_interactions
from gamegine.utils.NCIM.ncim import Inch, Degree, Meter
from gamegine.simulation.robot import RobotState
from gamegine.first.alliance import Alliance
from gamegine.rl.config import RobotConfig

class TestAllianceEnvSymmetry(unittest.TestCase):
    def setUp(self):
        self.game = create_rebuilt_game()
        
        # Create one blue and one red robot
        self.blue_robot = create_robot("Blue1", Alliance.BLUE)
        setup_robot_interactions(self.blue_robot, self.game)
        
        self.red_robot = create_robot("Red1", Alliance.RED)
        setup_robot_interactions(self.red_robot, self.game)
        
        # Symmetric positions:
        # Blue at (2m, 2m), Heading 0
        # Red at (L-2m, W-2m), Heading 180 (mirrored 0)
        self.blue_start = RobotState(
            x=Meter(2.0), y=Meter(2.0), heading=Degree(0),
            alliance=Alliance.BLUE
        )
        
        L = float(FIELD_LENGTH.to(Meter))
        W = float(FIELD_WIDTH.to(Meter))
        
        self.red_start = RobotState(
            x=Meter(L - 2.0), y=Meter(W - 2.0), heading=Degree(180),
            alliance=Alliance.RED
        )
        
        self.env = make_alliance_env(
            game=self.game,
            blue_robots=[RobotConfig(self.blue_robot, self.blue_start, "Blue1", "blue")],
            red_robots=[RobotConfig(self.red_robot, self.red_start, "Red1", "red")],
            mode="self_play",
            fast_mode=True
        )
        
        # Ensure mirroring is ON
        self.env.training_config.mirror_red_alliance = True
        
    def test_observation_symmetry(self):
        """Test that symmetric physical states result in identical observations."""
        obs, _ = self.env.reset()
        
        blue_obs = obs["blue_0"]
        red_obs = obs["red_0"]
        
        # Check basic dimensions (x, y, heading)
        # obs[0] = x, obs[1] = y, obs[2] = heading
        
        print(f"Blue Obs (Pos): {blue_obs[0]:.3f}, {blue_obs[1]:.3f}, {blue_obs[2]:.3f}")
        print(f"Red Obs (Mirrored): {red_obs[0]:.3f}, {red_obs[1]:.3f}, {red_obs[2]:.3f}")
        
        # Should be identical because Red's (L-2, W-2, 180) is mirrored to (2, 2, 0)
        np.testing.assert_allclose(blue_obs[:3], red_obs[:3], atol=1e-5, err_msg="Position/Heading not symmetric")
        
    def test_score_swap(self):
        """Test that score inputs are swapped for Red."""
        # Set scores manually
        self.env.server.game_state.blue_score.set(10)
        self.env.server.game_state.red_score.set(5)
        
        obs = self.env._get_observation("blue_0")
        # index 6=Red(Opp), 7=Blue(Own) for Blue
        # If env keeps 6=Red, 7=Blue constant:
        # Blue Agent sees: Red=5, Blue=10
        
        obs_red = self.env._get_observation("red_0")
        # Red Agent should see: Own Score(7) at index 7? 
        # Wait, mirroring logic swaps 6 and 7.
        # Original Red Obs: 6=Red(5), 7=Blue(10)
        # Swapped Red Obs: 6=Blue(10), 7=Red(5)
        # Goal: Index 7 always "Own Score", Index 6 always "Opp Score"
        
        # Blue: Own=10, Opp=5
        # obs[7] should be 10, obs[6] should be 5
        self.assertEqual(obs[7], 10, "Blue own score mismatch")
        self.assertEqual(obs[6], 5, "Blue opp score mismatch")
        
        # Red: Own=5, Opp=10
        # If symmetric: obs[7] should be 5, obs[6] should be 10
        self.assertEqual(obs_red[7], 5, "Red own score mismatch (should be at index 7)")
        self.assertEqual(obs_red[6], 10, "Red opp score mismatch (should be at index 6)")
        
    def test_action_mapping(self):
        """Test that abstract actions map to correct alliance objects."""
        # Check Blue mapping
        blue_maps = self.env._action_maps["Blue1"]
        blue_concrete_map = self.env._abstract_to_concrete["Blue1"]
        
        # Find index for OWN_HUB:shoot
        idx = -1
        for i, (act, interaction) in enumerate(blue_maps):
            if act == "OWN_Hub" and interaction == "shoot":
                idx = i
                break
        
        # Wait, checking naming convention. "Blue Hub" -> "OWN_Hub"
        # My implementation: "Blue " -> "OWN_" suffix -> "OWN_Hub"
        
        # Find ANY action with OWN_ and check mapping
        found = False
        for abstract, concrete in blue_concrete_map.items():
            if "OWN_" in abstract[0]:
                found = True
                # For Blue, OWN_ means Blue
                self.assertTrue("Blue" in concrete[0], f"Blue agent OWN mapped to {concrete[0]}")
            if "OPP_" in abstract[0]:
                self.assertTrue("Red" in concrete[0], f"Blue agent OPP mapped to {concrete[0]}")
                
        self.assertTrue(found, "No abstract actions found for Blue")
        
        # Check Red mapping
        red_concrete_map = self.env._abstract_to_concrete["Red1"]
        found = False
        for abstract, concrete in red_concrete_map.items():
            if "OWN_" in abstract[0]:
                found = True
                # For Red, OWN_ means Red
                self.assertTrue("Red" in concrete[0], f"Red agent OWN mapped to {concrete[0]}")
            if "OPP_" in abstract[0]:
                self.assertTrue("Blue" in concrete[0], f"Red agent OPP mapped to {concrete[0]}")
        
        self.assertTrue(found, "No abstract actions found for Red")

if __name__ == '__main__':
    unittest.main()
